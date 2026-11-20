function h = gui_stub(kind, varargin)
%GUI_STUB  Stand-in for the App Designer calls of the TA tools, so a test can run a tool's real code in -batch with no
%   window (fixes5, 20 Nov 2026; tests/test_prerun_wiring.m (h)). The test generates the tool's copy with
%   uifigure( / uilabel( / uilamp( / uialert( replaced by gui_stub('<name>', ...); nothing else of the tool changes.
%     'uifigure'  returns a started timer that deletes itself after 1 s: the tool's "while isvalid(fig)" loop then
%                 ends as if the window had been closed (its pause() lets the timer run), and its onCleanup runs
%     'uilamp'    returns gobjects(1) (the tool stores it in a gobjects array)
%     otherwise   returns [] (uilabel, uialert: nothing to show)
switch kind
    case 'uifigure'
        h = timer('Name', 'gui_stub_fig', 'StartDelay', 1, 'TimerFcn', @(t, ~) set(t, 'UserData', 1), 'StopFcn', @(t, ~) delete(t));
        start(h);
    case 'uilamp'
        h = gobjects(1);
    otherwise
        h = [];
end
end
