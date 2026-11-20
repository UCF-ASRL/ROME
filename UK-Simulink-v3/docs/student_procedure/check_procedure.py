"""Static checks of the student sheet and the TA manual (Plan 3 Task 5 Step 3; split for focus plan F14b, 20 Nov 2026).

Run with an isolated interpreter, from any folder (on Windows add -X utf8):

    python -I check_procedure.py                       # checks ROME_student_sheet.tex AND ROME_TA_manual.tex here
    python -I check_procedure.py --tex <copy.tex> [--role student|ta|legacy] [--pdf <file.pdf>]
                                                       # checks one file (make-it-fail runs; the old procedure)
    python -I check_procedure.py --list-keys           # prints the TRACKER section 0 keys and exits
    python -I check_procedure.py --repo                # lab-repo mode (also detected automatically)
    python -I check_procedure.py --update-lock         # rewrites citation_lock.tsv from BOTH files (refused with --tex)

The documents and their roles (chosen by file name, or by --role for a copy):
  ROME_student_sheet.tex   role student   the short sheet a student follows at the robot (<= 6 pages)
  ROME_TA_manual.tex       role ta        everything else: setup, sign-offs, lab_settings.m, troubleshooting, sources
  ROME_lab_procedure.tex   role legacy    the single procedure of 19 Nov, superseded; checked only with --tex
Every check runs on every file unless the table below says otherwise. The citation lock is ONE file
(citation_lock.tsv) shared by both documents: --update-lock writes the union of their cited lines, so it is refused
when only one --tex is given (that run would drop the other document's keys).

Two modes. REVIEW mode (the author's review folder, where ROME_merged/ exists) runs every check as described below.
REPO mode (this folder sits at <repo>/UK-Simulink-v3/docs/student_procedure, so HERE/../.. is the model root with
define_constants.m and has no ROME_merged/ inside) differs in three ways: LAB is the repo root (the folder that holds
UK-Arduino/), the ROME_merged/ prefix of a citation maps to the model root, and --tracker is optional (no TRACKER.md in
the repo: C5 prints SKIP). A citation into a review-only document (TRACKER, plans, run logs, CLAUDE.md, the
unpublished manuscript) is reported as "review-only, not checkable from the repo", not as a failure. A cited file that
is in the repo is still checked line by line; a wrong line FAILS in both modes.

Printed citations versus review citations (proc4, 19 Nov 2026). The printed text (\\src) cites only files that exist
in the lab repo. Citations into review-only documents live in LaTeX comment lines of their own,
"% review-src: path:lines; path:lines" (never after text on the same line, CLAUDE.md section 22). REVIEW mode resolves
and locks every review-src entry exactly like a \\src; REPO mode lists them as review-only without resolving them. In
both modes a printed \\src that names a review-only document, or the review folder ROME_merged/, FAILS, and a
"% review-src:" with text before the % FAILS. The student sheet sets \\showsrcfalse, so its \\src are checked here but
not printed.

Checks (each prints PASS, FAIL or SKIP with the offending items, prefixed by the role; exit code 1 if any FAIL):

  C1 packages    documentclass is article; every \\usepackage is in the allowed set
                 {graphicx, amsmath, siunitx, enumitem, booktabs, hyperref, tikz}; no \\RequirePackage.
  C2 inputs      every \\input, \\include and \\includegraphics target exists beside the .tex, OR sits in the first
                 branch of \\IfFileExists{<same path>}{...}{...} whose second branch holds \\todo{BLOCKED ...}.
  C3 numbers     in the checked part (\\begin{document} to \\appendix, or to the end when there is no \\appendix) no
                 number appears unless the allow-list (number_allowlist.tsv) has it. Each allow-list row cites
                 file:line, and the cited line must contain the number. Removed before the scan: comments,
                 tikzpicture environments, \\todo{}, \\src{} and the path arguments of \\input, \\includegraphics,
                 \\IfFileExists, \\label, \\ref. Node labels of the TikZ files under figures/ that the document \\input-s are scanned the same way.
                 Generated fragments under generated/ are exempt (they come from code).
  C4 citations   every \\src{path:lines} names a file that exists, its lines exist, and the text of each cited line
                 equals the text recorded in citation_lock.tsv when the citation was checked by hand. A line that
                 moved or changed under a citation FAILS. The same for every "% review-src:" entry (review mode).
                 --rev HEAD reads the cited files from git (this repo, or the lab repo for UK-Arduino/...) instead of
                 the working tree.
  C5 tracker     (roles ta, legacy) every row of TRACKER.md section 0 has a "% tracker: <key>" comment inside the
                 troubleshooting section, every such comment names a current row, keys are unique, and the parsed row
                 count equals an independent count. The troubleshooting section is the \\section that carries
                 \\label{sec:trouble} (role ta) or the 11th \\section (role legacy), up to the next \\section or
                 \\appendix.
  C6 structure   braces and \\begin/\\end balanced; \\todo is defined. Role student: \\showsrcfalse is set and
                 \\showsrctrue is not, and the text names neither Fast Restart nor GETTING_STARTED (both removed from
                 the student path, focus plan F14b). Role ta: a \\label{sec:trouble} section before \\appendix and at
                 least one \\section after it. Role legacy: exactly 11 \\section before \\appendix, at least one after.
  C7 pages       (role student) the compiled PDF beside the .tex (or --pdf) exists, is not older than the .tex, has at
                 most MAX_STUDENT_PAGES pages, and its text holds no printed source bracket (no "<file>.m:<digits>",
                 .md, .ino or .slx citation). Needs pymupdf; without it the check FAILS in review mode and is
                 SKIPped in repo mode (never a silent PASS). The "not older" test runs in review mode only: in the
                 repo the file times come from the port (which writes the .pdf before the .tex) or a clone.
  C8 order       nowhere may the arm be calibrated before the mode is set. Role student: in every \\section* that has a
                 step "Calibrate the arm", a step "Set the mode" comes first. Role ta: \\label{sec:setmode} comes
                 before \\label{sec:calib} (the TA notes follow the card order). Role legacy: not checked (its
                 Section 6, calibration, comes before Section 8, set the mode: the defect F13 found).

Blind spots (stated so nobody reads more into a PASS): a number written as a word ("two") is not seen; an
allow-listed number is accepted wherever it appears, whatever it means there; the check cannot tell whether a cited
line says what the sentence claims, only that the line is unchanged since it was read by hand; a review-src entry is
checked for existence and lock only, like a \\src, not for what it says; C7 counts pages of the PDF that is there, it
does not compile the .tex (a PDF newer than the .tex but built from another copy passes); C8 reads the step titles
"Set the mode" and "Calibrate the arm" (student) and two labels (ta), not the meaning of free prose.
"""
import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CMS = os.path.normpath(os.path.join(HERE, '..', '..'))            # ConstrainedMultibodySubmission
UCF = os.path.normpath(os.path.join(CMS, '..', '..', '..'))       # .../UCF
LAB = os.path.join(UCF, 'ucf-asrl', 'ROME')                        # lab repo, read-only
REPO_MODE = False                                                  # set in main(): lab-repo layout
REVIEW_ONLY = []                                                   # citations not checkable in repo mode
ALLOWED_PACKAGES = {'graphicx', 'amsmath', 'siunitx', 'enumitem', 'booktabs', 'hyperref', 'tikz'}
N_SECTIONS = 11
MONTHS = 'Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec'
NUM_RE = re.compile(r'(?<![A-Za-z0-9_.\\])\d+(?:\.\d+)*')         # a number not glued to an identifier

results = []                                                       # (check name, ok, detail lines)


def report(name, ok, details):
    results.append((name, ok, details))


def read(path):
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


# ----------------------------------------------------------------------------- LaTeX helpers
def strip_comments(text):
    """Blank from an unescaped % to the end of each line. Offsets stay the same as in the raw text."""
    out = []
    for line in text.split('\n'):
        m = re.search(r'(?<!\\)%', line)
        out.append(line[:m.start()] + ' ' * (len(line) - m.start()) if m else line)
    return '\n'.join(out)


def group_end(text, i):
    """text[i] == '{' ; return the index just after its matching '}' (escaped braces skipped)."""
    assert text[i] == '{', 'group_end called off a brace at %d' % i
    depth = 0
    k = i
    while k < len(text):
        c = text[k]
        if c == '\\':
            k += 2
            continue
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return k + 1
        k += 1
    raise ValueError('unbalanced brace starting at offset %d' % i)


def skip_ws(text, k):
    while k < len(text) and text[k] in ' \t\n':
        k += 1
    return k


def macro_calls(text, name, nargs, optional=True):
    """Yield (start, end, [args]) for each \\name[opt]{a1}...{an}."""
    for m in re.finditer(r'\\' + name + r'(?![A-Za-z])', text):
        k = skip_ws(text, m.end())
        if optional and k < len(text) and text[k] == '[':
            depth = 0
            while k < len(text):
                if text[k] == '[':
                    depth += 1
                elif text[k] == ']':
                    depth -= 1
                    if depth == 0:
                        k += 1
                        break
                k += 1
        args = []
        ok = True
        for _ in range(nargs):
            k = skip_ws(text, k)
            if k >= len(text) or text[k] != '{':
                ok = False
                break
            e = group_end(text, k)
            args.append((k, e, text[k + 1:e - 1]))
            k = e
        if ok:
            yield m.start(), k, args


def blank_spans(text, spans):
    """Replace each (start, end) span by spaces (keeps offsets and line count)."""
    chars = list(text)
    for s, e in spans:
        for k in range(s, e):
            if chars[k] != '\n':
                chars[k] = ' '
    return ''.join(chars)


def norm_path(p):
    p = p.strip().replace('\\', '/')
    return p[:-4] if p.endswith('.tex') else p


# ----------------------------------------------------------------------------- TRACKER keys
def tracker_rows(tracker_path):
    lines = read(tracker_path).split('\n')
    start = next(i for i, l in enumerate(lines) if l.startswith('## 0.'))
    end = next(i for i in range(start + 1, len(lines)) if lines[i].startswith('## '))
    block = lines[start:end]
    rows = []
    for l in block:
        if not l.startswith('|'):
            continue
        if re.match(r'^\|[\s\-|]+\|\s*$', l) or l.startswith('| Date seen'):
            continue
        cells = [c.strip() for c in l.strip().strip('|').split('|')]
        rows.append(cells)
    # independent count: lines whose first cell starts with "<day> <month>"
    indep = sum(1 for l in block if re.match(r'^\| *\d{1,2} (%s)\b' % MONTHS, l))
    return rows, indep, start + 1


def row_key(cells):
    date = '-'.join(re.findall(r'[a-z0-9]+', cells[0].lower()))
    words = re.findall(r'[a-z0-9_]+', cells[1].lower().replace('`', ''))
    return date + '/' + '-'.join(words[:8])


# ----------------------------------------------------------------------------- citation files
AMBIGUOUS = set()                                                  # cited paths found under more than one base


def is_review_only(path):
    """Repo mode: True for a cited path that belongs to a review-only document (not in the lab repo)."""
    p = path.strip().replace('\\', '/')
    if p.startswith('ROME_merged/'):
        p = p[len('ROME_merged/'):]
    return (p.startswith(('docs/superpowers/', 'docs/verification/', '../'))
            or re.match(r'^docs/\d{4}-\d\d-\d\d', p) is not None
            or re.match(r'^\d{4}-\d\d-\d\d_.*\.log$', p) is not None
            or p in ('TRACKER.md', 'CLAUDE.md'))


def resolve(path):
    path = path.strip().replace('\\', '/')
    if REPO_MODE:                                                  # ROME_merged/ prefix -> model root
        cands = [os.path.join(CMS, path), os.path.join(CMS, 'docs', 'verification', path), os.path.join(LAB, path)]
        if path.startswith('ROME_merged/'):
            cands.append(os.path.join(CMS, path[len('ROME_merged/'):]))
        hits = []
        for f in cands:
            if os.path.isfile(f) and f not in hits:
                hits.append(f)
        if len(hits) > 1:
            AMBIGUOUS.add(path)
            return None
        return hits[0] if hits else None
    hits =[os.path.join(b, path) for b in (CMS, os.path.join(CMS, 'ROME_merged'),
                                            os.path.join(CMS, 'docs', 'verification'), LAB)
            if os.path.isfile(os.path.join(b, path))]
    if len(hits) > 1:
        AMBIGUOUS.add(path)
        return None
    return hits[0] if hits else None


def parse_cite(c):
    """'path:12-14,20' -> (path, [12,13,14,20]); 'path free text' -> (path, [])."""
    c = c.strip()
    m = re.match(r'^(.+?):(\d[\d,\- ]*)$', c)
    if m:
        lines = []
        for part in m.group(2).split(','):
            part = part.strip()
            if '-' in part:
                a, b = part.split('-')
                lines.extend(range(int(a), int(b) + 1))
            elif part:
                lines.append(int(part))
        return m.group(1), lines
    return c.split(' ')[0], []


ROOT = os.path.dirname(CMS)                                        # local review repo (git)
REV = None                                                         # --rev: read cited files from this git revision


def file_lines(p):
    """Lines of a cited file, from the working tree, or from git at REV when --rev is given."""
    if REV is None:
        with open(p, 'r', encoding='utf-8', errors='replace') as f:
            return f.read().split('\n')
    pa = os.path.normcase(os.path.abspath(p))
    repo = LAB if pa.startswith(os.path.normcase(os.path.abspath(LAB)) + os.sep) else ROOT
    rel = os.path.relpath(os.path.abspath(p), repo).replace('\\', '/')
    r = subprocess.run(['git', '-C', repo, 'show', '%s:%s' % (REV, rel)], capture_output=True)
    if r.returncode != 0:
        raise FileNotFoundError('%s not in git %s at %s' % (rel, repo, REV))
    return r.stdout.decode('utf-8', errors='replace').replace('\r\n', '\n').split('\n')


def lock_key(path, line):
    return '%s:%d' % (path.strip().replace('\\', '/'), line)


# ----------------------------------------------------------------------------- per-file checks
DEFAULT_FILES = (('ROME_student_sheet.tex', 'student'), ('ROME_TA_manual.tex', 'ta'))
ROLE_BY_NAME = {'ROME_student_sheet.tex': 'student', 'ROME_TA_manual.tex': 'ta', 'ROME_lab_procedure.tex': 'legacy'}
MAX_STUDENT_PAGES = 6                                              # focus plan F14b: the student sheet is <= 6 pages
SRC_BRACKET_RE = re.compile(r'[A-Za-z0-9_./-]+\.(?:m|md|ino|slx)\s*:\s*\d')   # a printed "file.m:12"-style citation


def load_allow(allow_path):
    """Allow-list -> ({token: [(cites, ctx)]}, [problems]). Each row's citation must hold the token."""
    allow, adet = {}, []
    for ln, line in enumerate(read(allow_path).split('\n'), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        parts = line.split('\t')
        if len(parts) < 3:
            adet.append('allow-list line %d: needs token<TAB>citation<TAB>context' % ln)
            continue
        tok, cites, ctx = parts[0].strip(), parts[1].strip(), parts[2].strip()
        for c in cites.split(';'):
            path, lines = parse_cite(c)
            p = resolve(path)
            if p is None and REPO_MODE and path.strip() not in AMBIGUOUS and is_review_only(path):
                REVIEW_ONLY.append('allow-list line %d: %s' % (ln, path.strip()))
                continue
            if p is None:
                adet.append('allow-list line %d: cited file %s: %s' % (ln, 'is ambiguous' if path.strip() in AMBIGUOUS else 'not found', path))
                continue
            try:
                fl = file_lines(p)
            except FileNotFoundError as ex:
                adet.append('allow-list line %d: %s' % (ln, ex))
                continue
            if not lines:
                adet.append('allow-list line %d: citation %s has no line number' % (ln, c))
                continue
            if max(lines) > len(fl):
                adet.append('allow-list line %d: %s has %d lines, cited %d' % (ln, path, len(fl), max(lines)))
                continue
            pat = re.compile(r'(?<![0-9.])' + re.escape(tok) + r'(?![0-9])')
            if not any(pat.search(fl[i - 1]) for i in lines):
                adet.append('allow-list line %d: token %s not on %s' % (ln, tok, c))
        allow.setdefault(tok, []).append((cites, ctx))
    return allow, adet


def pdf_facts(pdf):
    """(page count, full text) of a PDF with pymupdf; raises ImportError when pymupdf is missing."""
    import pymupdf                                                 # noqa: E402 (optional dependency, C7 only)
    d = pymupdf.open(pdf)
    try:
        return d.page_count, '\n'.join(pg.get_text() for pg in d)
    finally:
        d.close()


def check_file(tex, role, a, keys, rows, indep, allow, adet, pdf=None):
    """Run C1-C8 on one .tex. Returns {lock key: cited line text now} for the shared lock (C4 compares it later)."""
    tag = '[%s %s]' % (role, os.path.basename(tex))
    texdir = os.path.dirname(os.path.abspath(tex))
    raw = read(tex)
    nc = strip_comments(raw)

    # ---- C1 packages
    det = []
    dc = re.search(r'\\documentclass(?:\[[^\]]*\])?\{([^}]*)\}', nc)
    if not dc or dc.group(1).strip() != 'article':
        det.append('documentclass is %r, expected article' % (dc.group(1) if dc else None))
    pk = []
    for m in re.finditer(r'\\usepackage(?:\[[^\]]*\])?\{([^}]*)\}', nc):
        pk.extend(p.strip() for p in m.group(1).split(','))
    for p in pk:
        if p not in ALLOWED_PACKAGES:
            det.append('package %r not in the allowed set' % p)
    if re.search(r'\\RequirePackage', nc):
        det.append('\\RequirePackage used')
    report(tag + ' C1 packages', not det, det or ['packages: ' + ', '.join(pk)])

    # ---- C2 inputs
    det = []
    info = []
    guards = []                                                    # (true_start, true_end, path, else_text)
    for s, e, args in macro_calls(nc, 'IfFileExists', 3, optional=False):
        guards.append((args[1][0], args[1][1], norm_path(args[0][2]), args[2][2]))
    targets = []
    for name in ('input', 'include'):
        for s, e, args in macro_calls(nc, name, 1, optional=False):
            targets.append((s, args[0][2], ('', '.tex')))
    for s, e, args in macro_calls(nc, 'includegraphics', 1):
        targets.append((s, args[0][2], ('', '.pdf', '.png', '.jpg')))
    for s, tgt, exts in targets:
        exists = any(os.path.isfile(os.path.join(texdir, tgt + x)) for x in exts)
        guarded = any(gs <= s < ge and norm_path(gp) == norm_path(tgt) and re.search(r'\\todo\{\s*BLOCKED', gelse)
                      for gs, ge, gp, gelse in guards)
        if exists:
            info.append('exists   %s%s' % (tgt, ' (guarded)' if guarded else ''))
        elif guarded:
            info.append('guarded  %s (missing, BLOCKED fallback shown)' % tgt)
        else:
            det.append('MISSING and unguarded: %s' % tgt)
    report(tag + ' C2 inputs', not det, det + info)

    # ---- C3 numbers
    det = list(adet)
    b = nc.find('\\begin{document}')
    ap_ = nc.find('\\appendix')
    body_start = b if b >= 0 else 0
    body_end = ap_ if ap_ >= 0 else len(nc)
    spans = []
    for m in re.finditer(r'\\begin\{tikzpicture\}.*?\\end\{tikzpicture\}', nc, re.S):
        spans.append((m.start(), m.end()))
    for name, n, opt in (('todo', 1, False), ('src', 1, False), ('input', 1, False), ('include', 1, False),
                         ('includegraphics', 1, True), ('label', 1, False), ('ref', 1, False),
                         ('pageref', 1, False), ('autoref', 1, False), ('url', 1, False), ('href', 1, False)):
        for s, e, args in macro_calls(nc, name, n, optional=opt):
            spans.append((s, e))
    for s, e, args in macro_calls(nc, 'IfFileExists', 1, optional=False):
        spans.append((s, e))
    for s, e, args in macro_calls(nc, 'placeholder', 2, optional=False):
        spans.append((args[0][0], args[1][1]))                     # the two size arguments of a placeholder box
    for m in re.finditer(r'\\begin\{[A-Za-z*]+\}(\[[^\]]*\])?(\{[^}]*\})?', nc):
        spans.append((m.start(), m.end()))                         # environment options and column specs
    for m in re.finditer(r'\\(v|h)space\*?\{[^}]*\}|\\\\\[[^\]]*\]', nc):
        spans.append((m.start(), m.end()))                         # lengths
    scan = blank_spans(nc, spans)
    seen = {}
    for m in NUM_RE.finditer(scan, body_start, body_end):
        tok = m.group(0)
        line_no = scan.count('\n', 0, m.start()) + 1
        if tok not in allow:
            det.append('line %d: number %s not on the allow-list: ...%s...'
                       % (line_no, tok, scan[max(0, m.start() - 40):m.end() + 20].replace('\n', ' ').strip()))
        seen[tok] = seen.get(tok, 0) + 1
    figdir = os.path.join(texdir, 'figures')
    used_figs = {norm_path(args[0][2]) for s, e, args in macro_calls(nc, 'input', 1, optional=False)}
    if os.path.isdir(figdir):
        for fn in sorted(os.listdir(figdir)):
            if not fn.endswith('.tex') or norm_path('figures/' + fn) not in used_figs:
                continue
            ft = strip_comments(read(os.path.join(figdir, fn)))
            for m in re.finditer(r'node\s*(\[[^\]]*\])?\s*(\([^)]*\))?\s*(at\s*\([^)]*\))?\s*\{', ft):
                k = m.end() - 1
                lab = ft[k + 1:group_end(ft, k) - 1]
                for nn in NUM_RE.finditer(lab):
                    if nn.group(0) not in allow:
                        det.append('figures/%s: node label number %s not on the allow-list: %s' % (fn, nn.group(0), lab))
    report(tag + ' C3 numbers', not det,
           det + ['numbers found: ' + ', '.join('%s x%d' % (k, v) for k, v in sorted(seen.items()))])

    # ---- C4 citations (resolution here; the lock comparison is done once for all files in main)
    det = []
    n = 0
    now = {}
    cites = []
    ro_start = len(REVIEW_ONLY)
    for s, e, args in macro_calls(nc, 'src', 1, optional=False):
        for c in args[0][2].split(';'):
            n += 1
            path = parse_cite(c)[0].strip().replace('\\', '/')
            if path.startswith('ROME_merged/'):
                det.append('printed \\src names the review folder ROME_merged/: %s' % c.strip())
            elif is_review_only(path):
                det.append('printed \\src cites a review-only document (move it to a "%% review-src:" line): %s'
                           % c.strip())
            else:
                cites.append(('\\src', c))
    n_rs = 0
    for ln, line in enumerate(raw.split('\n'), 1):
        m = re.search(r'(?<!\\)%\s*review-src:(.*)$', line)
        if not m:
            continue
        if line[:m.start()].strip():
            det.append('line %d: "%% review-src:" after text on the same line (CLAUDE.md section 22): %s'
                       % (ln, line.strip()[:110]))
        for c in m.group(1).split(';'):
            if not c.strip():
                continue
            n_rs += 1
            if REPO_MODE:
                REVIEW_ONLY.append('review-src (line %d) %s' % (ln, c.strip()))
            else:
                cites.append(('review-src', c))
    for kind, c in cites:
        if c.strip():
            path, lines = parse_cite(c)
            p = resolve(path)
            if p is None and REPO_MODE and path.strip() not in AMBIGUOUS and is_review_only(path):
                REVIEW_ONLY.append('%s %s' % (kind, c.strip()))
                continue
            if p is None:
                det.append('cited file %s: %s' % ('is ambiguous (found under more than one base; give a longer path)' if path.strip() in AMBIGUOUS else 'not found', c.strip()))
                continue
            try:
                fl = file_lines(p)
            except FileNotFoundError as ex:
                det.append(str(ex))
                continue
            if lines and max(lines) > len(fl):
                det.append('%s: line %d beyond the end (%d lines)' % (path, max(lines), len(fl)))
                continue
            for i in lines:
                now[lock_key(path, i)] = fl[i - 1].strip()
    C4_PENDING.append((tag, det, n, n_rs, now, REVIEW_ONLY[ro_start:]))

    # ---- C5 tracker
    secs = [m.start() for m in re.finditer(r'\\section\*?\{', nc)]
    sec_before = [s for s in secs if ap_ < 0 or s < ap_]
    if role in ('ta', 'legacy'):
        det = []
        if len(rows) == 0 or len(rows) != indep:
            det.append('TRACKER section 0: parsed %d rows, independent count %d' % (len(rows), indep))
        dup = sorted({k for k in keys if keys.count(k) > 1})
        if dup:
            det.append('duplicate TRACKER keys (lengthen row_key): ' + ', '.join(dup))
        trouble = ''
        if role == 'legacy':
            if len(sec_before) >= N_SECTIONS and ap_ > 0:
                trouble = raw[sec_before[N_SECTIONS - 1]:ap_]
        else:
            lab = nc.find('\\label{sec:trouble}')
            if lab >= 0:
                start = max([s for s in secs if s < lab] or [lab])
                after = [s for s in secs if s > lab] + ([ap_] if ap_ > lab else [])
                trouble = raw[start:min(after) if after else len(raw)]
        if not trouble:
            det.append('cannot locate the troubleshooting section (%s)'
                       % ('11th section' if role == 'legacy' else '\\label{sec:trouble}'))
        tex_keys = re.findall(r'^\s*%\s*tracker:\s*(\S+)\s*$', trouble, re.M)
        for k in [k for k in keys if k not in tex_keys]:
            det.append('TRACKER row without a troubleshooting entry: %s' % k)
        for k in [k for k in tex_keys if k not in keys]:
            det.append('troubleshooting key matches no current TRACKER row (stale?): %s' % k)
        for k in sorted({k for k in tex_keys if tex_keys.count(k) > 1}):
            det.append('key used twice in the troubleshooting section: %s' % k)
        if a.tracker is None:
            report(tag + ' C5 tracker', None, ['no TRACKER.md in the repo (a review document); pass --tracker to run this check'])
        else:
            report(tag + ' C5 tracker', not det, det or ['%d TRACKER section 0 rows (independent count %d), all mapped'
                                                         % (len(rows), indep)])

    # ---- C6 structure
    det = []
    after = [s for s in secs if ap_ >= 0 and s > ap_]
    depth = 0
    k = 0
    while k < len(nc):
        c = nc[k]
        if c == '\\':
            k += 2
            continue
        depth += (c == '{') - (c == '}')
        if depth < 0:
            det.append('closing brace without an opening one near line %d' % (nc.count('\n', 0, k) + 1))
            depth = 0
        k += 1
    if depth != 0:
        det.append('%d unclosed brace(s)' % depth)
    stack = []
    for m in re.finditer(r'\\(begin|end)\{([^}]*)\}', nc):
        if m.group(1) == 'begin':
            stack.append((m.group(2), nc.count('\n', 0, m.start()) + 1))
        else:
            if not stack or stack[-1][0] != m.group(2):
                det.append('\\end{%s} at line %d does not close %s' % (m.group(2), nc.count('\n', 0, m.start()) + 1,
                                                                     stack[-1] if stack else 'anything'))
            else:
                stack.pop()
    for env, ln in stack:
        det.append('\\begin{%s} at line %d never closed' % (env, ln))
    if not re.search(r'\\newcommand\{?\\todo\}?', nc):
        det.append('\\todo is not defined')
    if role == 'student':
        if not re.search(r'^\s*\\showsrcfalse\b', nc, re.M) or re.search(r'^\s*\\showsrctrue\b', nc, re.M):
            det.append('the student sheet must set \\showsrcfalse (no printed source brackets) and not \\showsrctrue')
        body = nc[body_start:]
        for word in ('Fast Restart', 'GETTING_STARTED', 'GETTING\\_STARTED'):
            for m in re.finditer(re.escape(word), body):
                det.append('line %d: the student sheet names %s (removed from the student path, F14b)'
                           % (nc.count('\n', 0, body_start + m.start()) + 1, word))
        shape = '%d sections' % len(secs)
    elif role == 'ta':
        if '\\label{sec:trouble}' not in nc[:ap_ if ap_ >= 0 else len(nc)]:
            det.append('no \\label{sec:trouble} before \\appendix')
        if ap_ < 0 or not after:
            det.append('no \\appendix with a section after it (TA reference)')
        shape = '%d sections before \\appendix, %d after' % (len(sec_before), len(after))
    else:
        if len(sec_before) != N_SECTIONS:
            det.append('%d \\section before \\appendix, expected %d' % (len(sec_before), N_SECTIONS))
        if ap_ < 0 or not after:
            det.append('no \\appendix with a section after it (TA appendix)')
        shape = '%d sections before \\appendix, %d after' % (len(sec_before), len(after))
    report(tag + ' C6 structure', not det, det or [shape + '; braces and environments balanced'])

    # ---- C7 pages (student sheet)
    if role == 'student':
        det = []
        pdf = pdf or os.path.splitext(os.path.abspath(tex))[0] + '.pdf'
        skip = None
        npg = None
        if not os.path.isfile(pdf):
            det.append('no PDF %s: build it (pdflatex in a copy of this folder) and put it beside the .tex' % pdf)
        else:
            # Freshness only in REVIEW mode: in the repo the file times come from the port or a clone (port_to_repo
            # writes the .pdf before the .tex, sorted by name), so there they say nothing about the build.
            if not REPO_MODE and os.path.getmtime(pdf) < os.path.getmtime(tex):
                det.append('the PDF is older than the .tex: rebuild it (%s)' % pdf)
            try:
                npg, text = pdf_facts(pdf)
            except ImportError:
                text = ''
                if REPO_MODE:
                    skip = 'pymupdf is not installed: page count and printed citations NOT CHECKED (never a PASS)'
                else:
                    det.append('pymupdf is not installed: the page count cannot be read (install it; this check never '
                               'passes without reading the PDF)')
            if npg is not None:
                if npg > MAX_STUDENT_PAGES:
                    det.append('%d pages, at most %d allowed' % (npg, MAX_STUDENT_PAGES))
                for m in SRC_BRACKET_RE.finditer(text):
                    det.append('printed source citation in the PDF: %s' % m.group(0))
        if skip and not det:
            report(tag + ' C7 pages', None, [skip])
        else:
            report(tag + ' C7 pages', not det,
                   det or ['%s: %d pages (at most %d), %s, no printed source citation'
                           % (os.path.basename(pdf), npg, MAX_STUDENT_PAGES,
                              'file times not compared (repo mode)' if REPO_MODE else 'not older than the .tex')])

    # ---- C8 order (never calibrate before setting the mode)
    if role in ('student', 'ta'):
        det = []
        info = []
        if role == 'student':
            bounds = secs + [len(nc)]
            for i in range(len(secs)):
                part = nc[bounds[i]:bounds[i + 1]]
                title = re.match(r'\\section\*?\{([^}]*)\}', part)
                cal = part.find('Calibrate the arm')
                if cal < 0:
                    continue
                st = part.find('Set the mode')
                name = title.group(1) if title else '?'
                if st < 0 or st > cal:
                    det.append('section "%s": "Calibrate the arm" (line %d) comes before any "Set the mode" step'
                               % (name, nc.count('\n', 0, bounds[i] + cal) + 1))
                else:
                    info.append('section "%s": set the mode, then calibrate' % name)
            if not info and not det:
                det.append('no card with a "Calibrate the arm" step found (the check would be empty)')
        else:
            sm, cb = nc.find('\\label{sec:setmode}'), nc.find('\\label{sec:calib}')
            if sm < 0 or cb < 0:
                det.append('labels sec:setmode and sec:calib both needed (found %s, %s)' % (sm >= 0, cb >= 0))
            elif sm > cb:
                det.append('\\label{sec:calib} (line %d) comes before \\label{sec:setmode} (line %d)'
                           % (nc.count('\n', 0, cb) + 1, nc.count('\n', 0, sm) + 1))
            else:
                info.append('sec:setmode (line %d) before sec:calib (line %d)'
                            % (nc.count('\n', 0, sm) + 1, nc.count('\n', 0, cb) + 1))
        report(tag + ' C8 order', not det, det or info)

    blocked = len(re.findall(r'\\todo\{\s*BLOCKED', nc))
    notver = len(re.findall(r'\\todo\{\s*NOT VERIFIED', nc))
    TODO_COUNTS.append('%s %d BLOCKED, %d NOT VERIFIED' % (tag, blocked, notver))
    return now


C4_PENDING = []                                                    # (tag, det, n src, n review-src, now, review-only)
TODO_COUNTS = []


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tex', action='append', default=None,
                    help='a .tex to check (repeatable); default: ROME_student_sheet.tex and ROME_TA_manual.tex here')
    ap.add_argument('--role', choices=('student', 'ta', 'legacy'), default=None,
                    help='role of every --tex (default: by file name; an unknown name needs --role)')
    ap.add_argument('--pdf', default=None, help='C7: the PDF of the student sheet (default: beside its .tex)')
    ap.add_argument('--allow', default=None, help='allow-list (default: number_allowlist.tsv beside this script)')
    ap.add_argument('--lock', default=None, help='citation lock (default: citation_lock.tsv beside this script)')
    ap.add_argument('--tracker', default=None,
                    help='TRACKER.md (default CMS/TRACKER.md; optional in repo mode, C5 is then SKIPped)')
    ap.add_argument('--repo', action='store_true',
                    help='lab-repo mode (automatic when CMS has define_constants.m and no ROME_merged/)')
    ap.add_argument('--list-keys', action='store_true')
    ap.add_argument('--rev', default=None, help='read cited files from this git revision (e.g. HEAD)')
    ap.add_argument('--update-lock', action='store_true',
                    help='rewrite citation_lock.tsv from the cited lines of BOTH documents as they are now '
                         '(check them first); refused with --tex')
    a = ap.parse_args()
    global REV, REPO_MODE, LAB, ROOT
    REV = a.rev
    REPO_MODE = a.repo or (os.path.isfile(os.path.join(CMS, 'define_constants.m'))
                           and not os.path.isdir(os.path.join(CMS, 'ROME_merged')))
    if REPO_MODE:
        LAB = os.path.dirname(CMS)
        ROOT = LAB
    if a.tracker is None:
        a.tracker = os.path.join(CMS, 'TRACKER.md')
        if REPO_MODE and not os.path.isfile(a.tracker):
            a.tracker = None
    if a.tracker is None:
        if a.list_keys:
            print('--list-keys needs a TRACKER.md (not in the repo); pass --tracker')
            return 2
        rows, indep = [], 0
    else:
        rows, indep, _ = tracker_rows(a.tracker)
    keys = [row_key(r) for r in rows]
    if a.list_keys:
        for r, k in zip(rows, keys):
            print(k)
        print('rows parsed %d, independent count %d' % (len(rows), indep))
        return 0
    if a.update_lock and a.tex:
        print('--update-lock is refused with --tex: the lock is shared by the student sheet and the TA manual, and a '
              'one-file run would drop the other file\'s keys. Run it without --tex.')
        return 2
    if a.tex:
        files = []
        for t in a.tex:
            role = a.role or ROLE_BY_NAME.get(os.path.basename(t))
            if role is None:
                print('unknown file name %s: pass --role student|ta|legacy' % t)
                return 2
            files.append((os.path.abspath(t), role))
    else:
        files = [(os.path.join(HERE, f), r) for f, r in DEFAULT_FILES]
    if a.pdf and not any(r == 'student' for f, r in files):
        print('--pdf given but no student-role file is checked')
        return 2
    allow_path = a.allow or os.path.join(HERE, 'number_allowlist.tsv')
    lock_path = a.lock or os.path.join(HERE, 'citation_lock.tsv')
    print('check_procedure: %s' % ', '.join('%s (%s)' % (f, r) for f, r in files))
    print('  allow-list %s' % allow_path)
    print('  lock       %s' % lock_path)
    print('  tracker    %s' % (os.path.abspath(a.tracker) if a.tracker else '(none, repo mode)'))
    if REPO_MODE:
        print('  mode       repo (LAB %s; ROME_merged/ -> %s)' % (LAB, CMS))
    allow, adet = load_allow(allow_path)
    ro_allow = list(REVIEW_ONLY)
    now_all = {}
    for f, r in files:
        now_all.update(check_file(f, r, a, keys, rows, indep, allow, adet, pdf=a.pdf if r == 'student' else None))

    # ---- C4 lock comparison, one lock for all files
    lock = {}
    if os.path.isfile(lock_path):
        for line in read(lock_path).split('\n'):
            if line.strip() and not line.startswith('#'):
                k, _, v = line.partition('\t')
                lock[k] = v
    if a.update_lock:
        with open(lock_path, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write('# cited line -> its text when last checked by hand. Rewrite with --update-lock only after\n'
                     '# re-reading every changed citation. Generated by check_procedure.py from %s.\n'
                     % ' and '.join(os.path.basename(f) for f, r in files))
            for k in sorted(now_all):
                fh.write('%s\t%s\n' % (k, now_all[k]))
        print('citation lock rewritten: %d lines -> %s' % (len(now_all), lock_path))
        lock = dict(now_all)
    for tag, det, n, n_rs, now, ro in C4_PENDING:
        det = list(det)
        for k in sorted(now):
            if k not in lock:
                det.append('cited line not in %s: %s' % (os.path.basename(lock_path), k))
            elif lock[k] != now[k]:
                det.append('cited line changed since it was checked: %s\n            was: %s\n            now: %s'
                           % (k, lock[k][:110], now[k][:110]))
        ro4 = ['review-only, not checkable from the repo: %s' % x for x in ro]
        report(tag + ' C4 citations', not det,
               det + ro4 or ['%d printed citations and %d review-src entries (%d cited lines), all files and lines '
                             'exist and match %s%s' % (n, n_rs, len(now), os.path.basename(lock_path),
                                                       ' (read from git %s)' % REV if REV else '')])
    if ro_allow:
        report('[allow-list] review-only rows', None, ['review-only, not checkable from the repo: %s' % x for x in ro_allow])

    # ---- summary
    bad = 0
    for name, ok, d in results:
        print('%s  %s' % ('SKIP' if ok is None else 'PASS' if ok else 'FAIL', name))
        for line in d:
            print('        ' + line)
        bad += ok is False
    for t in TODO_COUNTS:
        print('todo markers: ' + t)
    if bad:
        print('check_procedure FAIL: %d of %d checks failed' % (bad, len(results)))
        return 1
    skipped = sum(1 for r in results if r[1] is None)
    if skipped:
        print('check_procedure PASS: %d of %d checks (%d skipped)' % (len(results) - skipped, len(results), skipped))
        return 0
    print('check_procedure PASS: %d of %d checks' % (len(results), len(results)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
