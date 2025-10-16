% connect_bridge.m
% Connect MATLAB to the WSL2 TCP bridge
clc; clear; close all;

% WSL IP address (check with WSL)
WSL_IP = '172.17.180.26';  % change if it differs

t_telemetry = tcpclient(WSL_IP, 9090, "Timeout", 20);
t_cmd       = tcpclient(WSL_IP, 9091, "Timeout", 20);

disp("Connected to TCP bridge between MATLAB and ROS2");
