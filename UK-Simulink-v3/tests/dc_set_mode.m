function txt = dc_set_mode(txt, n, want)
%DC_SET_MODE  Test helper (focus plan F12): set the run mode in a define_constants.m TEXT, count-asserted.
%   txt = dc_set_mode(txt, n)        the line 'run_mode = <k>;' becomes 'run_mode = n;' (only the digits change).
%   txt = dc_set_mode(txt, n, want)  also forces run flags the mode does not give, for regression paths outside the
%                                    author's four modes (EnableMotive = 0; arm_freeze = 0 in simulation; a bad
%                                    motive_source). want is a struct with any of the fields EnableHardware,
%                                    EnableMotive, motive_source, arm_freeze. A field equal to the mode's own value
%                                    (run_mode_table(n)) is dropped; each other one becomes one line
%                                      <field> = <value>;  run_mode_test_override{end+1} = '<field>';  fprintf(...)
%                                    inserted right after the derivation line OVR_ANCHOR, so it wins over the mode, and
%                                    registered in run_mode_test_override, which arm_freeze_banner reads: only the
%                                    fields listed there may differ from what run_mode gives.
%   For TEMP COPIES of define_constants.m only; the real file never carries an override.
%   Errors (never a silent no-op, unlike the regexprep(...,'once') edits this replaces):
%     dc_set_mode:count   not exactly one line '^run_mode[ \t]*=[ \t]*\d+[ \t]*;' or not exactly one line assigning
%                         run_mode (same rule as set_run_mode.m)
%     dc_set_mode:anchor  an override is asked and OVR_ANCHOR is not on exactly one line start
%     dc_set_mode:field   an unknown field in want
%     run_mode_table:bad  n is not 1-4
%   The line ending of the inserted lines is the text's own (CRLF if the text has any CRLF, else LF).
OVR_ANCHOR = 'arm_freeze     = run_mode_cfg.arm_freeze;';   % define_constants derivation line the overrides follow (char)
M = run_mode_table(n);                                    % validates n; the mode's flags (struct)
if nargin < 3, want = struct(); end
txt = char(txt);
[~, tok] = regexp(txt, '^run_mode[ \t]*=[ \t]*(\d+)[ \t]*;', 'match', 'tokenExtents', 'lineanchors');   % strict matches
nLoose = numel(regexp(txt, '^[ \t]*run_mode[ \t]*=(?!=)', 'start', 'lineanchors'));   % lines assigning run_mode (count)
if numel(tok) ~= 1 || nLoose ~= 1
    error('dc_set_mode:count', 'define_constants text: %d lines "run_mode = <digits>;" and %d lines assigning run_mode (expected 1 and 1)', ...
          numel(tok), nLoose);
end
d = tok{1}(1, :);                                         % [first last] index of the digits (1x2)
txt = [txt(1:d(1)-1) sprintf('%d', M.n) txt(d(2)+1:end)];
fn = fieldnames(want);                                    % requested flags (cell of char)
known = {'EnableHardware', 'EnableMotive', 'motive_source', 'arm_freeze'};
bad = setdiff(fn, known);
if ~isempty(bad), error('dc_set_mode:field', 'unknown override field(s): %s', strjoin(bad, ', ')); end
ovr = '';                                                 % override lines (char)
if any(contains(txt, char([13 10]))), eol = char([13 10]); else, eol = char(10); end   % the text's line ending (char)
for k = 1:numel(fn)
    v = want.(fn{k});
    if isequal(v, M.(fn{k})), continue; end               % the mode already gives it
    if ischar(v), vt = ['''' v '''']; else, vt = sprintf('%g', v); end   % value as MATLAB source (char)
    ovr = [ovr fn{k} ' = ' vt ';  run_mode_test_override{end+1} = ''' fn{k} ''';  ' ...
           'fprintf(''TEST OVERRIDE (dc_set_mode, temp copy only): ' fn{k} ' = %s\n'', mat2str(' fn{k} '));' eol]; %#ok<AGROW>
end
if isempty(ovr), return; end
a = regexp(txt, ['^' regexptranslate('escape', OVR_ANCHOR) '[^\r\n]*\r?\n'], 'end', 'lineanchors');   % end of the anchor line (index)
if numel(a) ~= 1
    error('dc_set_mode:anchor', 'define_constants text: the override anchor "%s" starts %d lines (expected 1)', OVR_ANCHOR, numel(a));
end
txt = [txt(1:a) ovr txt(a+1:end)];
end
