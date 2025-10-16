import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import socket, threading, json, time, math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ---- helpers -------------------------------------------------

def ndjson_send(conn, obj):
    try:
        line = json.dumps(obj, separators=(',', ':')) + '\n'
        conn.sendall(line.encode('utf-8'))
    except Exception:
        pass  # client may be disconnected

def yaw_from_quat(q):
    # q is geometry_msgs/Quaternion
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

# ---- bridge node ---------------------------------------------

class TCPROSBridge(Node):
    def __init__(self, telemetry_port=9090, command_port=9091, scan_decimate=4, scan_max_len=720):
        super().__init__('tcp_ros_bridge')

        # Publishers (ROS <- MATLAB)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions (ROS -> MATLAB)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Heartbeat topic (also helps discovery)
        self.hb_pub = self.create_publisher(String, '/bridge_heartbeat', 10)
        self.create_timer(2.0, self.heartbeat_cb)

        # Socket servers
        self.telemetry_conn = None
        self.command_conn = None

        self.scan_decimate = max(1, int(scan_decimate))
        self.scan_max_len = scan_max_len

        # Start servers in background threads so node starts immediately
        threading.Thread(target=self._serve_telemetry, args=(telemetry_port,), daemon=True).start()
        threading.Thread(target=self._serve_commands, args=(command_port,), daemon=True).start()

        self.get_logger().info(f'✅ tcp_ros_bridge up: telemetry@{telemetry_port}  commands@{command_port}')

    # ---- ROS callbacks → MATLAB (telemetry) -------------------
    def heartbeat_cb(self):
        # publish ROS topic and also push to MATLAB if connected
        msg = String()
        msg.data = 'bridge alive'
        self.hb_pub.publish(msg)
        self._send_telemetry({'topic': 'heartbeat', 'data': msg.data, 't': self.get_clock().now().nanoseconds})

    def odom_cb(self, odom: Odometry):
        pose = odom.pose.pose
        twist = odom.twist.twist
        yaw = yaw_from_quat(pose.orientation)
        payload = {
            'topic': 'odom',
            't': odom.header.stamp.sec + odom.header.stamp.nanosec*1e-9,
            'pose': {'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw},
            'twist': {'vx': twist.linear.x, 'vy': twist.linear.y, 'vz': twist.linear.z,
                      'wx': twist.angular.x, 'wy': twist.angular.y, 'wz': twist.angular.z}
        }
        self._send_telemetry(payload)

    def scan_cb(self, scan: LaserScan):
        # decimate & clip for lighter messages
        ranges = scan.ranges[::self.scan_decimate]
        if self.scan_max_len and len(ranges) > self.scan_max_len:
            ranges = ranges[:self.scan_max_len]
        payload = {
            'topic': 'scan',
            't': scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9,
            'angle_min': scan.angle_min,
            'angle_max': scan.angle_max,
            'angle_increment': scan.angle_increment * self.scan_decimate,
            'ranges': ranges
        }
        self._send_telemetry(payload)

    # ---- MATLAB → ROS (commands) ------------------------------
    def _handle_command_line(self, line: str):
        try:
            msg = json.loads(line)
        except Exception:
            return
        topic = msg.get('topic', '')
        if topic == 'cmd_vel':
            m = msg.get('msg', {})
            lin = m.get('linear', [0,0,0])
            ang = m.get('angular', [0,0,0])
            t = Twist()
            t.linear.x, t.linear.y, t.linear.z = float(lin[0]), float(lin[1]), float(lin[2])
            t.angular.x, t.angular.y, t.angular.z = float(ang[0]), float(ang[1]), float(ang[2])
            self.cmd_vel_pub.publish(t)

    # ---- sockets ----------------------------------------------
    def _serve_telemetry(self, port):
        self.get_logger().info(f'[telemetry] waiting on 0.0.0.0:{port}')
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', port))
        srv.listen(1)
        while rclpy.ok():
            conn, addr = srv.accept()
            self.get_logger().info(f'[telemetry] connected from {addr}')
            self.telemetry_conn = conn
            # keep it open until client disconnects
            try:
                while True:
                    time.sleep(1.0)
            except Exception:
                pass
            finally:
                try: conn.close()
                except: pass
                self.telemetry_conn = None
                self.get_logger().info('[telemetry] disconnected')

    def _serve_commands(self, port):
        self.get_logger().info(f'[commands]  waiting on 0.0.0.0:{port}')
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', port))
        srv.listen(1)
        while rclpy.ok():
            conn, addr = srv.accept()
            self.get_logger().info(f'[commands] connected from {addr}')
            self.command_conn = conn
            try:
                with conn.makefile('r') as f:
                    for line in f:
                        self._handle_command_line(line.strip())
            except Exception:
                pass
            finally:
                try: conn.close()
                except: pass
                self.command_conn = None
                self.get_logger().info('[commands] disconnected')

    def _send_telemetry(self, obj):
        if self.telemetry_conn:
            ndjson_send(self.telemetry_conn, obj)

def main():
    rclpy.init()
    node = TCPROSBridge()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
