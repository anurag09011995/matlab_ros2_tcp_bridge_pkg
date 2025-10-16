% read_telemetry.m
% Run after connect_bridge.m
disp("Listening for ROS2 telemetry...");

while true
    if t_telemetry.NumBytesAvailable > 0
        line = readline(t_telemetry);
        data = jsondecode(char(line));

        switch string(data.topic)
            case "heartbeat"
                fprintf("[HB] %s\n", data.data);

            case "odom"
                p = data.pose; v = data.twist;
                fprintf("ODOM x=%.2f y=%.2f yaw=%.2f vx=%.2f wz=%.2f\n", ...
                    p.x, p.y, p.yaw, v.vx, v.wz);

            case "scan"
                fprintf("SCAN with %d beams\n", numel(data.ranges));
        end
    else
        pause(0.05);
    end
end
