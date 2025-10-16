global t_cmd
if isempty(t_cmd)
    error("T_CMD not connected. Run connect_bridge.m first!");
end

disp("Sending velocity commands...");

% Create JSON and ensure newline
send_cmd = @(vx, wz) ...
    write(t_cmd, uint8([jsonencode(struct( ...
        'topic', 'cmd_vel', ...
        'msg', struct('linear', [vx 0 0], 'angular', [0 0 wz]) ...
    )) newline]), "uint8");

% Try sending small test pattern
send_cmd(0.2, 0.0); pause(2.0);   % forward
send_cmd(0.0, 0.5); pause(1.5);   % rotate
send_cmd(0.0, 0.0);               % stop

disp("✅ Commands sent");
