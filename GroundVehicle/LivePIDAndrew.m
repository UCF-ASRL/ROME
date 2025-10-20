close all

%% --- Circle Parameters --- %%     
R     = 0.5;               % radius [m]
T     = 15;                 % period [s]
omega = (2*pi)/T;           % angular speed [rad/s]
dt    = 0.05;               % MATLAB control step [s] (Arduino base step can be 0.10 s)
numFrames = int16(T*(1/dt)) + 1;

%% --- Buffers --- %%
x       = zeros(1,numFrames); 
y       = zeros(1,numFrames);
yaw     = zeros(1,numFrames);
time_s  = zeros(1,numFrames);
ex_log  = zeros(1,numFrames);
ey_log  = zeros(1,numFrames);
xr_log  = zeros(1,numFrames);
yr_log  = zeros(1,numFrames);
wcmd_log = zeros(numFrames,4);
vx_ref_log = zeros(1,numFrames);
vy_ref_log = zeros(1,numFrames);
clc;

%% --- Bluetooth Serials --- %% 
bt = serialport("COM10", 115200);   % adjust baud as needed
configureTerminator(bt,"LF");
flush(bt);

%% --- Wait for user input --- %%
resp = upper(strtrim(input('Type Y to start experiment: ','s')));
while (resp ~= 'Y')
    fprintf("Wrong input. Please type 'Y' to start motion.\n");
    resp = upper(strtrim(input('Type Y to start experiment: ','s')));
end
fprintf("Sending 'Y' to ESP32...\n");
writeline(bt,'Y');   % trigger robot start (keep port open)

fprintf('NatNet Polling Sample Start\n');

%% --- NatNet Setup --- %%
fprintf('Creating natnet class object\n');
natnetclient = natnet;

fprintf('Connecting to the server\n');
natnetclient.HostIP         = '127.0.0.1';
natnetclient.ClientIP       = '127.0.0.1';
natnetclient.ConnectionType = 'Unicast';
natnetclient.connect;

if (natnetclient.IsConnected == 0)
    fprintf('Client failed to connect\n');
    fprintf('\tCheck network and IP settings\n\n');
    return
end

model = natnetclient.getModelDescription;
natnetclient.enable(0);

if (model.RigidBodyCount < 1)
    error('No Rigid Bodies Detected!');
end

fprintf('\nPrinting rigid body frame data approximately every %.2fs for %.1f seconds...\n\n', dt, double(numFrames-1)*dt);

%% --- Zero Origins --- %%
data        = natnetclient.getFrame;
x_origin    = data.RigidBodies(1).x;
y_origin    = data.RigidBodies(1).y;
time_origin = data.fTimestamp;
frame_og    = data.iFrame;

x_cap_prev = -(data.RigidBodies(1).x - x_origin - R);
y_cap_prev = -(data.RigidBodies(1).y - y_origin);

x(1) = R; y(1) = 0; time_s(1) = 0; %assuming starting at (R,0) and doing CCW circle

%% --- PID State --- %%
prev_ex = 0; prev_ey = 0;
int_ex  = 0; int_ey  = 0;

%% --- Main Control Loop --- %%
t0 = tic;
for idx = 2:numFrames

    while toc(t0) < dt
        %wait until dt passed%
    end 

    t0 = tic;

    data = natnetclient.getFrame;

    % Time since start
    t = data.fTimestamp - time_origin;
    time_s(idx) = t;

    % Measured position in meters, zeroed at origin
    x_cap = -(data.RigidBodies(1).x - x_origin - R);
    y_cap = -(data.RigidBodies(1).y - y_origin);

    %pull all captured quaternions into matrix. Convert to eularian values
    %(yaw, pitch, roll) and save in eul_cap%

    q_cap = [data.RigidBodies(1).qw, data.RigidBodies(1).qx, data.RigidBodies(1).qy, data.RigidBodies(1).qz];
    eul_cap = [quat2eul(q_cap, "XYZ")];
    yaw_cap = (eul_cap(3));
    fprintf('Yaw: %0.5f degrees\n', rad2deg(yaw_cap));
  
    x(idx) = x_cap;  y(idx) = y_cap; yaw(idx) = yaw_cap;

    % Console print (keep your style)
    fprintf('Name:"%s"  ', model.RigidBody(1).Name);
    fprintf('Frame #:"%d"  ', data.iFrame - frame_og);
    fprintf('X:%0.1fmm  ', x_cap * 1000); %Motive sends data in Meters, this converts to mm
    fprintf('Y:%0.1fmm  ', y_cap * 1000);
    fprintf('Yaw:%0.1fmm  ', rad2deg(yaw_cap));
    fprintf('T:%0.2f\n', t);

    %% --- Reference (circle) position & velocity --- %%
    % Keep center at (0,0) after zeroing originsframe_og
    x_ref = (R*cos(omega*t)); 
    y_ref = (R*sin(omega*t));
    xr_log(idx) = x_ref;  yr_log(idx) = y_ref;

    % Reference body velocities for ROME path
    vx_ref = -R*omega*sin(omega*t);
    vy_ref =  R*omega*cos(omega*t);
    vx_ref_log(idx) = vx_ref; vy_ref_log(idx) = vy_ref;
    V = [vx_ref; vy_ref; yaw_cap];  %yaw_cap passed along to IK to accomodate for unwanted rotation of GV
    %% --- Error --- %%
    ex = x_ref - x_cap;
    ey = y_ref - y_cap;
    ex_log(idx) = ex;  ey_log(idx) = ey;
    Err = [ex ; ey ; 0];  %%zero in yaw spot to represent no corrections along this DOF yet

    %% --- PID correction velocities (m/s) --- %%
    Kp1 = 0.000;   Ki1 = 0.000;
    Vcorr = (-Kp1 * Err) + (-Ki1 * (Err * dt));

    %% --- Combine Ref w/ Corr --- %%
    Vfinal = V + Vcorr;

    %% --- Map to wheel speeds --- %%
    w_cmd_rads = InverseKinematics(Vfinal);
    w_cmd_rpm  = w_cmd_rads * (60/(2*pi));

    % Send to MEGA
    wcmd_log(idx,:) = w_cmd_rpm(:).';
    
    msg = sprintf('%.2f,%.2f,%.2f,%.2f', w_cmd_rpm(1), w_cmd_rpm(2), w_cmd_rpm(3), w_cmd_rpm(4));
    writeline(bt, msg);
 
end
writeline(bt, '0,0,0,0');
pause(0.01);
writeline(bt, '!,!,!,!'); %used to reset Arduino
clear bt;


%% --------------------- PRINT SECTIONS --------------------- %%

% 1) Path error printout (X and Y)
numRows = numFrames;
fid = fopen('pathERR.txt','wt');

fprintf(fid, 'Error in X Direction: {');
for j = 1:numRows
    if j == 1, fprintf(fid, '%.3f', ex_log(j));
    else,      fprintf(fid, ', %.3f', ex_log(j));
    end
end
fprintf(fid, '};\n\n');

fprintf(fid, 'Error in Y Direction: {');
for j = 1:numRows
    if j == 1, fprintf(fid, '%.3f', ey_log(j));
    else,      fprintf(fid, ', %.3f', ey_log(j));
    end
end
fprintf(fid, '};\n\n');
fclose(fid);

% 2) XYT data (measured + reference)
fid = fopen('xytData.txt','wt');
for j = 1:numRows
    fprintf(fid, 'Frame %d Data: ', j);
    fprintf(fid, 'X_meas = %.3f, Y_meas = %.3f, X_ref = %.3f, Y_ref = %.3f, Time = %.2f\n\n', ...
            x(j), y(j), xr_log(j), yr_log(j), time_s(j));
    if j == numRows
        fprintf(fid,'------------------------------ALL DATA PRINTED FOR THIS RUN------------------------------');
    end
end
fclose(fid);

% 3) Wheel speeds (combined command sent)
fid = fopen('wheelSpeeds_cmd.txt','wt');
fprintf(fid, 'Format per line: w1_cmd, w2_cmd, w3_cmd, w4_cmd (RPM)\n\n');
for j = 1:numRows
    fprintf(fid, '%.2f, %.2f, %.2f, %.2f\n', ...
        wcmd_log(j,1), wcmd_log(j,2), wcmd_log(j,3), wcmd_log(j,4));
end
fclose(fid);

fprintf('\nALL DONE, CHECK OUT THOSE TEXT FILES\n');

% Plot commanded vs tracked position
figure;
hold on;
plot(xr_log, yr_log, 'b', 'LineWidth', 2)
plot(x, y, 'r', 'LineWidth', 2)
title('Commanded vs Tracked Position')
xlabel('x pos (m)')
ylabel('y pos (m)')
legend({'Commanded','Tracked'},'Location','southwest')
axis square
axis equal
hold off

% Add 10 percent padding to plot
xlim_current = xlim;
ylim_current = ylim;
x_buffer = (xlim_current(2) - xlim_current(1)) * 0.1;
y_buffer = (ylim_current(2) - ylim_current(1)) * 0.1;
xlim([xlim_current(1) - x_buffer, xlim_current(2) + x_buffer]);
ylim([ylim_current(1) - y_buffer, ylim_current(2) + y_buffer]);

% Plot commanded wheel speeds
% figure;
% hold on;
% plot(time_s, wcmd_log(:, 1).', 'b', 'LineWidth', 2)
% plot(time_s, wcmd_log(:, 2).', 'r', 'LineWidth', 2)
% plot(time_s, wcmd_log(:, 3).', 'g', 'LineWidth', 2)
% plot(time_s, wcmd_log(:, 4).', 'y', 'LineWidth', 2)
% title('Wheel Speed vs Time')
% xlabel('Time (sec)')
% ylabel('Wheel Speed (RPM)')
% legend({'Wheel 1','Wheel 2', 'Wheel 3', 'Wheel 4'},'Location','southwest')