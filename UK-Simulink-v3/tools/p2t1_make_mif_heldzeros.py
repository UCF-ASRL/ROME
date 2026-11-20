"""Make-it-fail copy for test_motive_reader check (vi) (Plan 2 Task 1, fix round 1).

Copies the five files the test needs into DST/ROME_merged/{Optitrack,tests} and, in the COPY of
MotiveData.m only, initialises the held pose to zeros instead of NaN (property default and resetImpl).
Each edit must match exactly once; nothing is written unless both do. The real files are only read.
Run the copy with:  cd DST/ROME_merged/tests; matlab -batch "test_motive_reader"
Expected: check (vi) step1_all_nan FAILS (outputs are 0, not NaN, before the first valid frame).

usage: python -I p2t1_make_mif_heldzeros.py <ROME_merged root> <empty destination folder>
"""
import sys, pathlib, shutil
src = pathlib.Path(sys.argv[1])            # real ROME_merged (read only)
dst = pathlib.Path(sys.argv[2]) / 'ROME_merged'   # scratch copy
files = ['Optitrack/MotiveData.m', 'Optitrack/natnet_parse_rigidbody.m', 'Optitrack/quat2eul.m',
         'tests/test_motive_reader.m', 'tests/MotiveDataFake.m']
edits = [
    ("        lastPose = NaN(7, 1);           % last accepted",
     "        lastPose = zeros(7, 1);         % MAKE-IT-FAIL (was NaN(7, 1)) % last accepted"),
    ("            obj.lastPose = NaN(7, 1);\n",
     "            obj.lastPose = zeros(7, 1);   % MAKE-IT-FAIL (was NaN(7, 1))\n"),
]
raw = (src / 'Optitrack/MotiveData.m').read_bytes().decode('utf-8')
was_crlf = '\r\n' in raw                   # CRLF (lab repo working tree) or LF (review copy): both work
t = raw.replace('\r\n', '\n')              # match on LF, write back with the file's own line ending
for old, new in edits:
    n = t.count(old)
    if n != 1:
        sys.exit(f"pattern found {n} times, expected 1: {old!r}")
for old, new in edits:
    t = t.replace(old, new)
if dst.exists():
    sys.exit(f"destination exists, refusing to overwrite: {dst}")
for f in files:
    (dst / f).parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src / f, dst / f)
(dst / 'Optitrack/MotiveData.m').write_bytes((t.replace('\n', '\r\n') if was_crlf else t).encode('utf-8'))
print("copied:", ", ".join(files))
for old, new in edits:
    print("applied once in the copy:", new.strip())
