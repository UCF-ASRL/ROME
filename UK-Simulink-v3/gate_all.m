function gate_all()
%GATE_ALL  Run the acceptance gate on every case study, in the model.
here = fileparts(mfilename('fullpath')); cd(here); addpath(here);
orig = claim_constants();                  % see the function, foot of file
restore = onCleanup(@() release_constants(orig));
failed = {};
for sc = 1:5
    txt = regexprep(orig, 'scenario = \d+;', sprintf('scenario = %d;', sc), 'once');
    assert(contains(txt, sprintf('scenario = %d;', sc)), 'gate_all:select', ...
           'could not set scenario = %d in define_constants.m', sc);
    wf('define_constants.m', txt);
    [~,~,~,~,nm] = scenario_defaults(sc);
    fprintf('\n=== %d  %s ===\n', sc, nm);
    try
        check_9dof();
    catch ME
        fprintf('  GATE FAILED: %s\n', ME.message);
        failed{end+1} = sprintf('%d %s: %s', sc, nm, ME.message); %#ok<AGROW>
    end
end
if ~isempty(failed)
    error('gate_all:failed', '%d of 5 scenarios failed the gate:\n  %s', ...
          numel(failed), strjoin(failed, sprintf('\n  ')));
end
fprintf('\n  all five scenarios passed the gate\n');
end
function wf(p,s), f=fopen(p,'w'); fwrite(f,s); fclose(f); end


function orig = claim_constants()
% Take custody of define_constants.m, leaving a pristine copy on disk.
%
% define_constants.m is a tracked source file, and this harness rewrites it.
% onCleanup restores it when the function returns or errors, but a hard kill
% -- the process killed under memory pressure, a crash of the LXE engine --
% skips onCleanup entirely and leaves the file modified with nothing to record
% that fact. The next run would then read the wrong scenario, or the wrong
% dist_scale, and report a result for a case nobody asked for.
%
% So: the unmodified text is written to define_constants.PRISTINE before the
% first rewrite, and that file's existence is the signal that a previous run
% did not finish. Finding one here is refused rather than repaired, because
% the pristine copy is the only record of what the file looked like, and
% overwriting it silently would destroy that record.
lock = 'define_constants.PRISTINE';
if isfile(lock)
    error('constants:interrupted', ...
        ['a previous run was interrupted and left define_constants.m ' ...
         'modified.\n  The unmodified text is in %s.\n  Restore it with:  ' ...
         'copyfile(''%s'', ''define_constants.m''); delete(''%s'')\n  ' ...
         'Compare them first if you have edited define_constants.m since.'], ...
        lock, lock, lock);
end
orig = fileread('define_constants.m');
wf(lock, orig);
end

function release_constants(orig)
% Put define_constants.m back, then drop the pristine copy -- in that order, so
% an interruption between the two still leaves the record behind.
wf('define_constants.m', orig);
lock = 'define_constants.PRISTINE';
if isfile(lock), delete(lock); end
end
