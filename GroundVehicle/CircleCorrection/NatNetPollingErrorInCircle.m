%%---Circle Parameters---%%  
R = 0.25;         %radius in meters 
T = 15;           %period in seconds
omega = (2*pi)/T; %angular velo  
dt = 0.1;         %Arduino timesteps in seconds
numFrames = int16(T * (1/dt)) + 1; %number of datapoints total used for x,y,t. Used to index through data

x = zeros(1,numFrames);
y = zeros(1,numFrames);
time = zeros(1,numFrames); 
clc; 


    % % --- Setup Live Plot --- %
    % global px py a1
    % figure('Name','Live Robot Position Tracking','Color','w');
    % a1 = gca;
    % hold on
    % px = animatedline('Color','b','LineWidth',1.5, 'DisplayName','X Position');
    % py = animatedline('Color','r','LineWidth',1.5, 'DisplayName','Y Position');
    % legend

    % --- Setup Bluetooth Serial Connection --- %

    bt = serialport("COM10", 9600);   % lab computer uses COM10 as 'Outgoing; to ESP32-WROOM
    configureTerminator(bt,"LF");
    flush(bt);

    %% --- Wait for user input --- %%
    resp = upper(strtrim(input('Type Y to start experiment: ','s')));
 while(resp ~= 'Y')
        fprintf("Wrong input. Please type in 'Y' to start motion.\n");
        resp = upper(strtrim(input('Type Y to start experiment: ','s')));
 end
    fprintf("Sending 'Y' to ESP32...\n");
    writeline(bt,'Y');   % trigger robot start
    clear bt;

	fprintf( 'NatNet Polling Sample Start\n' )

	% create an instance of the natnet client class
	fprintf( 'Creating natnet class object\n' )
	natnetclient = natnet;

	% connect the client to the server (multicast over local loopback) -
	% modify for your network
	fprintf( 'Connecting to the server\n' )
	natnetclient.HostIP = '127.0.0.1';
	natnetclient.ClientIP = '127.0.0.1';
	natnetclient.ConnectionType = 'Unicast';
	natnetclient.connect;
	if ( natnetclient.IsConnected == 0 )
		fprintf( 'Client failed to connect\n' )
		fprintf( '\tMake sure the host is connected to the network\n' )
		fprintf( '\tand that the host and client IP addresses are correct\n\n' ) 
		return
	end

    % --- Live Frame Listener for Plotting --- %
    % natnetclient.addlistener(1,'plotpositionXYcircle');    
    model = natnetclient.getModelDescription;
	natnetclient.enable(0);

	if ( model.RigidBodyCount < 1 )
        error('No Rigid Bodies Detected!')
		return %#ok<UNRCH>
	end

	% Poll for the rigid body data a regular intervals (~1 sec) for 10 sec.
	fprintf( '\nPrinting rigid body frame data approximately every 0.1s for 15 seconds...\n\n' )

    %%   need these for correct data standarization (eg. Ground Vehicle not captured at (0,0) at start)  %%
    data = natnetclient.getFrame;
    x_origin = data.RigidBodies(1).x;
    y_origin = data.RigidBodies(1).y; 
    time_origin = data.fTimestamp; %%divide by 1000 to get seconds
    frame_og = data.iFrame;

    % fprintf('x_origin = %.2f, y_origin = %.2f, time_origin = %.2f, frame_og = %d\n\n' , x_origin, y_origin, time_origin, frame_og)

    x(1) = x_origin;
    y(1) = y_origin;
    time(1) = time_origin;

	for idx = 2 : numFrames

        pause(0.93 * dt);   %%0.93 found exp. to result in accurate pauses%%
		data = natnetclient.getFrame; % method to get current frame

		% for i = 1:model.RigidBodyCount
        %%% using 1 instead of i loop. Simplified since single Ground Vehicle %%%

            fprintf( 'Name:"%s"  ', model.RigidBody( 1 ).Name )
    		fprintf( 'Frame #:"%d"  ', data.iFrame - frame_og)
			fprintf( 'X:%0.1fmm  ', data.RigidBodies( 1 ).x * 1000 )
			fprintf( 'Y:%0.1fmm  ', data.RigidBodies( 1 ).y * 1000 )
            fprintf( 'T:%0.2f  \n',data.fTimestamp - time_origin)
            
            x(idx) = data.RigidBodies(1).x;
            y(idx) = data.RigidBodies(1).y;  
            time(idx) = data.fTimestamp;
    end
    
    
    %normalize data

    for idx = 1:numFrames
        x(idx) = x(idx)-x_origin;
        y(idx) = y(idx)-y_origin;
        time(idx) = time(idx) - time_origin;
    end

t_uniform = 0:dt:T;

% resample captured x,y using interpolation. This gives us data for exactly
% 0.1s time steps
x_uniform = interp1(time, x, t_uniform, 'linear');
y_uniform = interp1(time, y, t_uniform, 'linear');

% fprintf('MADE IT PAST INTERPOLATION CORRECTLY\n\n') %debug statement

ERR = Path_Error(t_uniform, x_uniform, y_uniform, x_origin, y_origin, R, omega, numFrames);
fprintf('Error has been calculated\n\n');
close all force;
pause(0.1);
natnetclient.disable(0);

% Print arrays out into ERR file
%first take a look at ERR(:,1) and go down collumn 1 row by row for x-err
numRows = size(ERR, 1);
numCols = size(ERR,2);

fid = fopen('pathERR.txt', 'wt');

% Write X errors
fprintf(fid, 'Error in X Direction: {');
for j = 1:numRows
    if j == 1
        fprintf(fid, '%.3f', ERR(j,1));
    else
        fprintf(fid, ', %.3f', ERR(j,1));
    end
end
fprintf(fid, '};\n\n');

% Write Y errors
fprintf(fid, 'Error in Y Direction: {');
for j = 1:numRows
    if j == 1
        fprintf(fid, '%.3f', ERR(j,2));
    else
        fprintf(fid, ', %.3f', ERR(j,2));
    end
end
fprintf(fid, '};\n\n');

fclose(fid);

%%%% --- Path Data --- %%%%

n = numFrames;

fid = fopen('xytData.txt', 'wt');

% Write data
for j = 1:n
fprintf(fid, 'Frame %d Data: ', j);
fprintf(fid, 'X = %.2f, Y = %.2f, at Time: %.2f\n\n', x_uniform(j), y_uniform(j), t_uniform(j));
    if j == n
        fprintf(fid, '------------------------------ALL DATA PRINTED FOR THIS RUN------------------------------');
    end
end
fclose(fid); 









 