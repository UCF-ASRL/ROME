function NatNetEventHandlerSample_XY
    
    global hf1 a1
    global px py 
    
    fprintf( '\nNatNet Event Handler Sample (X/Y only)\n' )
    fprintf( '=========================================================\n' )
    pause(2);
    
    CreatePlotsXY;

    % create an instance of the natnet client class
    fprintf( 'Creating natnet class object\n' )
    natnetclient = natnet;

    % connect the client to the server (multicast over local loopback)
    fprintf( 'Connecting to the server\n' )
    natnetclient.HostIP = '127.0.0.1';
    natnetclient.ClientIP = '127.0.0.1';
    natnetclient.ConnectionType = 'Unicast';
    natnetclient.connect;
    if ( natnetclient.IsConnected == 0 )
        fprintf( 'Client failed to connect\n' )
        return
    end

    % only add XY position callback
    fprintf( 'Adding callback function to execute with each frame of mocap\n' )
    addpath('event handlers')

    natnetclient.addlistener(1,'plotpositionXY');
    natnetclient.enable(0);

    %enable listeners
    fprintf( 'Enabling the listeners for asynchronous callback execution\n' )
    natnetclient.enable(0)
    fprintf( '(Enter any key to quit)\n\n' )
    pause;

    fprintf( 'Disabling the listeners\n')
    natnetclient.disable(0)
    disp('NatNet Event Handler Sample End' )
    pause(1);
    
    close(hf1)

end


function CreatePlotsXY
    % sets up plot for X and Y position only
    global hf1 a1
    global px py
    
    hf1 = figure;
    hf1.WindowStyle = 'docked';

    a1 = subplot(1,1,1);
    title('Position (X/Y only)');
    xlabel('Time')
    ylabel('Position (m)')

    px = animatedline;
    px.MaximumNumPoints = 1000;
    px.Marker = '.';
    px.LineWidth = 0.5;
    px.Color = [1 0 0]; % Red for X

    py = animatedline;
    py.MaximumNumPoints = 1000;
    py.LineWidth = 0.5;
    py.Color = [0 1 0]; % Green for Y
    py.Marker = '.';
end
