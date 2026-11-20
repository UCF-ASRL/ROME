"""Make-it-fail copy of tests/test_motive_reader.m: parser poseOffset 41 -> 45 (Plan 2 Task 1).

Three edits, each asserted to match exactly once: function name -> test_motive_reader_mif, the parser
layout's poseOffset 41 -> 45, and `here` pinned to the real tests folder (the copy runs from scratch).
The real test is only read. Run the copy with: cd <dst folder>; matlab -batch "test_motive_reader_mif"
Expected: check (i) FAILS (log line "layout needs 72" = 45 + 27 shows the shifted layout is in force).
Used for docs/verification/2026-11-19_p2t1_makefail_poseOffset45.log (originally run from the
session scratchpad as make_mif.py, same content apart from this docstring).

usage: python -I p2t1_make_mif.py <real test_motive_reader.m> <dst .m> <real tests folder, forward slashes>
"""
import sys, pathlib
src = pathlib.Path(sys.argv[1])            # the real test (read only)
dst = pathlib.Path(sys.argv[2])            # scratch copy to write
tests_dir = sys.argv[3]                    # real ROME_merged/tests folder (MATLAB path, forward slashes)
t = src.read_text(encoding='utf-8')
edits = [
    ("function test_motive_reader()", "function test_motive_reader_mif()"),
    ("lay = struct('msgIdOffset', 1, 'rbIdOffset', 37, 'poseOffset', 41);",
     "lay = struct('msgIdOffset', 1, 'rbIdOffset', 37, 'poseOffset', 45);   % MAKE-IT-FAIL: shifted by 4 bytes"),
    ("here = fileparts(mfilename('fullpath'));    % ROME_merged/tests",
     "here = '" + tests_dir + "';    % ROME_merged/tests (make-it-fail copy runs from scratch)"),
]
for old, new in edits:
    n = t.count(old)
    if n != 1:
        sys.exit(f"pattern found {n} times, expected 1: {old!r}")
    t = t.replace(old, new)
dst.write_text(t, encoding='utf-8')
print("wrote", dst)
for old, new in edits:
    print("applied once:", new)
