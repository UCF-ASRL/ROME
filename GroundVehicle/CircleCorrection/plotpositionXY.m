function plotpositionXY(rb, evnt)
    % Globals from main script
    global px py a1
    persistent lastTime
    
    rbnum = rb;    % Rigid body index (first one)
    
    % Extract timestamp in seconds
    t = double(evnt.data.fTimestamp);   % NatNet gives seconds, high precision
    
    % Reset if stream restarts (timestamps jumped backwards)
    if ~isempty(lastTime) && t < lastTime
        clearpoints(px);
        clearpoints(py);
    end
    
    % Get rigid body position
    x = double(evnt.data.RigidBodies(rbnum).x);
    y = double(evnt.data.RigidBodies(rbnum).y);
    
    % Append to animated lines
    addpoints(px, t, x);   % time vs x
    addpoints(py, t, y);   % time vs y
    
    % Update axis: keep ~last 10s visible
    set(gcf,'CurrentAxes',a1);
    window = 10; % seconds
    axis([t-window t min([x y])-0.1 max([x y])+0.1]);
    
    drawnow;
    lastTime = t;
end