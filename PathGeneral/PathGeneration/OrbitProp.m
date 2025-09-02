function [pos,e,traj,time] = OrbitProp(Motor1, a, eccen, RA, incl, w, TA)

% h = Angular Momentum of the Orbit
% e = eccentricity of the Orbit
% RA = Right Ascension of the Orbit
% incl = Inclination of the orbit
% w = Argument of perigee of the Orbit
% TA = Theta
% MU = Gravitational Paramter of the orbit

%example [position,error,trajectory,time] = elipse(Motor1, h, e, RA, incl, w, TA, mu)

Period = 2*pi*a^(3/2)*1/(sqrt(mu));

pause(5);

%NatNet Connection
natnetclient = natnet;
natnetclient.HostIP = '127.0.0.1';
natnetclient.ClientIP = '127.0.0.1';
natnetclient.ConnectionType = 'Multicast';
natnetclient.connect;

if ( natnetclient.IsConnected == 0 )
	fprintf( 'Client failed to connect\n' )
	fprintf( '\tMake sure the host is connected to the network\n' )
	fprintf( '\tand that the host and client IP addresses are correct\n\n' ) 
	return
end

model = natnetclient.getModelDescription;
if ( model.RigidBodyCount < 1 )
	return
end

%Platform Geometry
L = .13; %distance from platform centroid to wheel
R = .05;  %radius of the wheel

%PID
Kp = 8;
Ki = .4;

Kp = [Kp 0 0;
      0 Kp 0;
      0 0 -Kp;]; %Gains for theta must be negative, not sure why yet
  
Ki = [Ki 0 0;
      0 Ki 0;
      0 0 -Ki;];

%Main Loop
timestep = .02; %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
index = 0;
errorSum = 0;   %A forward euler integration will be performed.
numrevs = 1; %Determines the number of revolutions the platform will cycle
                %through before stopping.
                
tic;
while(toc < Period * numrevs)
    
    if (toc >= timestep*index)
        
        %Trajectory is the XYZ Positions
        %Trajectory prime is the XYZ Velocities
        [trajectory, trajectoryPrime] = PERI2XYZ(a, eccen, incl, O, w, (TA + toc));

    
        data = natnetclient.getFrame;	
		if (isempty(data.RigidBody(1)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end
        
        
        yaw = data.RigidBody(1).qy;
        pitch = data.RigidBody(1).qz;
        roll = data.RigidBody(1).qx;
        scalar = data.RigidBody(1).qw;
        q = quaternion(roll,yaw,pitch,scalar);
        qRot = quaternion(0,0,0,1);
        q = mtimes(q,qRot);
        a = EulerAngles(q,'zyx');
        theta = a(2); %yaw angle/rotation about the vertical

        position = [data.RigidBody(1).x;-data.RigidBody(1).z;theta;];
        error = position - trajectory;
        errorSum = errorSum + error*timestep;

        
        pTheta = [-sin(theta)               cos(theta)          L;
                  -sin((pi/3)-theta)       -cos((pi/3)-theta)   L;
                   sin((pi/3)+theta)       -cos((pi/3)+theta)   L;];
        
        setpointMetSec = pTheta*(trajectoryPrime - Kp*error - Ki*errorSum);
        setpointRadSec = setpointMetSec/R;
        
        Motor1.updateMotors(setpointRadSec);
        
        index = index + 1;
        time(index,:) = toc;
        pos(index,:) = position;
        e(index,:) = error;
        traj(index,:) = trajectory;
    end
end

%Set motors back to zero
Motor1.updateMotors([0.0,0.0,0.0]);

% plot(pos);
% hold on;
% plot(traj);
% hold off;

%Will need to look at the trajectoryprime value as it will be extremely
%high for normal values of mu as velocioty for a circular orbit is sqrt(mu/r).
% Likely will need to bring mu down to a very 
% small value so we can scale down the orbits

end


