"""Static checks of the student lab procedure (Plan 3 Task 5, Step 3).

Run with an isolated interpreter, from any folder:

    python -I check_procedure.py                      # checks ROME_lab_procedure.tex beside this file
    python -I check_procedure.py --tex <copy.tex>     # checks another copy (make-it-fail runs)
    python -I check_procedure.py --list-keys          # prints the TRACKER section 0 keys and exits
    python -I check_procedure.py --repo               # lab-repo mode (also detected automatically)

Two modes. REVIEW mode (the author's review folder, where ROME_merged/ exists) runs every check
as described below. REPO mode (this folder sits at <repo>/UK-Simulink-v3/docs/student_procedure, so
HERE/../.. is the model root with define_constants.m and has no ROME_merged/ inside) differs in three
ways: LAB is the repo root (the folder that holds UK-Arduino/), the ROME_merged/ prefix of a citation
maps to the model root, and --tracker is optional (no TRACKER.md in the repo: C5 prints SKIP).
A citation into a review-only document (TRACKER, plans, run logs, CLAUDE.md, the unpublished
manuscript) is reported as "review-only, not checkable from the repo", not as a failure. A cited file
that is in the repo is still checked line by line; a wrong line FAILS in both modes.

Printed citations versus review citations (proc4, 19 Nov 2026). The printed text (\\src) cites only
files that exist in the lab repo. Citations into review-only documents live in LaTeX comment lines of
their own, "% review-src: path:lines; path:lines" (never after text on the same line, CLAUDE.md
section 22: a mid-line % comments out the rest of the paragraph). REVIEW mode resolves and locks every
review-src entry exactly like a \\src; REPO mode lists them as review-only without resolving them.
In both modes a printed \\src that names a review-only document, or the review folder ROME_merged/,
FAILS, and a "% review-src:" with text before the % FAILS.

Checks (each prints PASS or FAIL with the offending items; exit code 1 if any FAIL):

  C1 packages    documentclass is article; every \\usepackage is in the allowed set
                 {graphicx, amsmath, siunitx, enumitem, booktabs, hyperref, tikz};
                 no \\RequirePackage.
  C2 inputs      every \\input, \\include and \\includegraphics target exists beside the .tex,
                 OR sits in the first branch of \\IfFileExists{<same path>}{...}{...} whose
                 second branch holds \\todo{BLOCKED ...}.
  C3 numbers     in the student part (\\begin{document} to \\appendix) no number appears
                 unless the allow-list (number_allowlist.tsv) has it. Each allow-list row
                 cites file:line, and the cited line must contain the number. Removed before
                 the scan: comments, tikzpicture environments, \\todo{}, \\src{} and the path
                 arguments of \\input, \\includegraphics, \\IfFileExists, \\label, \\ref.
                 Node labels of the TikZ files under figures/ are scanned the same way.
                 Generated fragments under generated/ are exempt (they come from code).
  C4 citations   every \\src{path:lines} names a file that exists, its lines exist, and the text of
                 each cited line equals the text recorded in citation_lock.tsv when the citation
                 was checked by hand. A line that moved or changed under a citation FAILS.
                 The same for every "% review-src:" entry (review mode). A printed \\src into a
                 review-only document or ROME_merged/ FAILS; a mid-line "% review-src:" FAILS.
                 --rev HEAD reads the cited files from git (this repo, or the lab repo for
                 UK-Arduino/...) instead of the working tree. --update-lock rewrites the lock.
  C5 tracker     every row of TRACKER.md section 0 has a "% tracker: <key>" comment inside
                 the troubleshooting section (section 11), every such comment names a current
                 row, keys are unique, and the parsed row count equals an independent count.
  C6 structure   exactly 11 \\section before \\appendix, at least one after it; braces and
                 \\begin/\\end balanced; \\todo is defined.

Blind spots (stated so nobody reads more into a PASS): a number written as a word ("two")
is not seen; an allow-listed number is accepted wherever it appears, whatever it means
there; the check cannot tell whether a cited line says what the sentence claims, only that
the number is on it; the checker does not compile the .tex (no LaTeX on this PC on 19 Nov);
a review-src entry is checked for existence and lock only, like a \\src, not for what it says.
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


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tex', default=os.path.join(HERE, 'ROME_lab_procedure.tex'))
    ap.add_argument('--allow', default=None, help='allow-list (default: number_allowlist.tsv beside the .tex)')
    ap.add_argument('--tracker', default=None,
                    help='TRACKER.md (default CMS/TRACKER.md; optional in repo mode, C5 is then SKIPped)')
    ap.add_argument('--repo', action='store_true',
                    help='lab-repo mode (automatic when CMS has define_constants.m and no ROME_merged/)')
    ap.add_argument('--list-keys', action='store_true')
    ap.add_argument('--rev', default=None, help='read cited files from this git revision (e.g. HEAD)')
    ap.add_argument('--update-lock', action='store_true',
                    help='rewrite citation_lock.tsv from the cited lines as they are now (check them first)')
    a = ap.parse_args()
    global REV, REPO_MODE, LAB, ROOT
    REV = a.rev
    REPO_MODE = a.repo or (os.path.isfile(os.path.join(CMS, 'define_constants.m'))
                           and not os.path.isdir(os.path.join(CMS, 'ROME_merged')))
    if REPO_MODE:
        LAB = os.path.dirname(CMS)                                 # repo root (holds UK-Arduino/)
        ROOT = LAB
    if a.tracker is None:
        a.tracker = os.path.join(CMS, 'TRACKER.md')
        if REPO_MODE and not os.path.isfile(a.tracker):
            a.tracker = None                                       # C5 is skipped
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

    texdir = os.path.dirname(os.path.abspath(a.tex))
    allow_path = a.allow or os.path.join(texdir, 'number_allowlist.tsv')
    raw = read(a.tex)
    print('check_procedure: %s' % os.path.abspath(a.tex))
    print('  allow-list %s' % allow_path)
    print('  tracker    %s' % (os.path.abspath(a.tracker) if a.tracker else '(none, repo mode)'))
    if REPO_MODE:
        print('  mode       repo (LAB %s; ROME_merged/ -> %s)' % (LAB, CMS))
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
    report('C1 packages', not det, det or ['packages: ' + ', '.join(pk)])

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
    report('C2 inputs', not det, det + info)

    # ---- C3 numbers
    det = []
    allow = {}                                                     # token -> list of (cite, context)
    adet = []
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
    det.extend(adet)

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
    # TikZ node labels in figures/*.tex
    figdir = os.path.join(texdir, 'figures')
    if os.path.isdir(figdir):
        for fn in sorted(os.listdir(figdir)):
            if not fn.endswith('.tex'):
                continue
            ft = strip_comments(read(os.path.join(figdir, fn)))
            for m in re.finditer(r'node\s*(\[[^\]]*\])?\s*(\([^)]*\))?\s*(at\s*\([^)]*\))?\s*\{', ft):
                k = m.end() - 1
                lab = ft[k + 1:group_end(ft, k) - 1]
                for n in NUM_RE.finditer(lab):
                    if n.group(0) not in allow:
                        det.append('figures/%s: node label number %s not on the allow-list: %s' % (fn, n.group(0), lab))
    unused = sorted(set(allow) - set(seen))
    n_ro3 = len(REVIEW_ONLY)
    report('C3 numbers', not det,
           det + ['review-only, not checkable from the repo: %s' % r for r in REVIEW_ONLY]
           + ['numbers found: ' + ', '.join('%s x%d' % (k, v) for k, v in sorted(seen.items()))]
           + (['allow-list entries not used in the text: ' + ', '.join(unused)] if unused else []))

    # ---- C4 citations
    # Each cited line is also compared with citation_lock.tsv (text of the line when the citation was
    # checked by hand), so a line that moved or changed under a citation is reported, not passed.
    det = []
    n = 0
    lock_path = os.path.join(texdir, 'citation_lock.tsv')
    lock = {}
    if os.path.isfile(lock_path):
        for line in read(lock_path).split('\n'):
            if line.strip() and not line.startswith('#'):
                k, _, v = line.partition('\t')
                lock[k] = v
    now = {}                                                       # lock key -> current text
    cites = []                                                     # (kind, citation text) to check
    for s, e, args in macro_calls(nc, 'src', 1, optional=False):
        for c in args[0][2].split(';'):
            n += 1
            path = parse_cite(c)[0].strip().replace('\\', '/')
            if path.startswith('ROME_merged/'):                    # printed text must name repo paths only
                det.append('printed \\src names the review folder ROME_merged/: %s' % c.strip())
            elif is_review_only(path):
                det.append('printed \\src cites a review-only document (move it to a "%% review-src:" line): %s'
                           % c.strip())
            else:
                cites.append(('\\src', c))
    n_rs = 0                                                       # review-src entries
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
        if c.strip():                                              # every printed \src and review-src entry
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
    if a.update_lock:
        with open(lock_path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('# cited line -> its text when last checked by hand. Rewrite with --update-lock only after\n'
                    '# re-reading every changed citation. Generated by check_procedure.py.\n')
            for k in sorted(now):
                f.write('%s\t%s\n' % (k, now[k]))
        print('citation lock rewritten: %d lines -> %s' % (len(now), lock_path))
        lock = dict(now)
    changed = 0
    for k in sorted(now):
        if k not in lock:
            det.append('cited line not in citation_lock.tsv: %s' % k)
            changed += 1
        elif lock[k] != now[k]:
            det.append('cited line changed since it was checked: %s\n            was: %s\n            now: %s'
                       % (k, lock[k][:110], now[k][:110]))
            changed += 1
    ro4 = ['review-only, not checkable from the repo: %s' % r for r in REVIEW_ONLY[n_ro3:]]
    report('C4 citations', not det,
           det + ro4 or ['%d printed citations and %d review-src entries (%d cited lines), all files and lines '
                         'exist and match citation_lock.tsv%s'
                         % (n, n_rs, len(now), ' (read from git %s)' % REV if REV else '')])

    # ---- C5 tracker
    det = []
    if len(rows) == 0 or len(rows) != indep:
        det.append('TRACKER section 0: parsed %d rows, independent count %d' % (len(rows), indep))
    dup = sorted({k for k in keys if keys.count(k) > 1})
    if dup:
        det.append('duplicate TRACKER keys (lengthen row_key): ' + ', '.join(dup))
    secs = [m.start() for m in re.finditer(r'\\section\{', nc)]   # nc keeps the offsets of raw
    sec_before = [s for s in secs if ap_ < 0 or s < ap_]
    if len(sec_before) >= N_SECTIONS and ap_ > 0:
        trouble = raw[sec_before[N_SECTIONS - 1]:ap_]              # raw slice: keeps the % tracker lines
    else:
        trouble = ''
        det.append('cannot locate section %d (troubleshooting)' % N_SECTIONS)
    tex_keys = re.findall(r'^\s*%\s*tracker:\s*(\S+)\s*$', trouble, re.M)
    missing = [k for k in keys if k not in tex_keys]
    unknown = [k for k in tex_keys if k not in keys]
    for k in missing:
        det.append('TRACKER row without a troubleshooting entry: %s' % k)
    for k in unknown:
        det.append('troubleshooting key matches no current TRACKER row (stale?): %s' % k)
    tdup = sorted({k for k in tex_keys if tex_keys.count(k) > 1})
    for k in tdup:
        det.append('key used twice in the troubleshooting section: %s' % k)
    if a.tracker is None:                                          # repo mode without TRACKER.md; secs is still used by C6
        report('C5 tracker', None, ['no TRACKER.md in the repo (a review document); pass --tracker to run this check'])
    else:
        report('C5 tracker', not det, det or ['%d TRACKER section 0 rows (independent count %d), all mapped'
                                              % (len(rows), indep)])

    # ---- C6 structure
    det = []
    before = [s for s in secs if ap_ < 0 or s < ap_]
    after = [s for s in secs if ap_ >= 0 and s > ap_]
    if len(before) != N_SECTIONS:
        det.append('%d \\section before \\appendix, expected %d' % (len(before), N_SECTIONS))
    if ap_ < 0 or not after:
        det.append('no \\appendix with a section after it (TA appendix)')
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
    report('C6 structure', not det, det or ['%d sections before \\appendix, %d after; braces and environments balanced'
                                            % (len(before), len(after))])

    # ---- summary
    bad = 0
    for name, ok, d in results:
        print('%s  %s' % ('SKIP' if ok is None else 'PASS' if ok else 'FAIL', name))
        for line in d:
            print('        ' + line)
        bad += ok is False
    blocked = len(re.findall(r'\\todo\{\s*BLOCKED', nc))
    notver = len(re.findall(r'\\todo\{\s*NOT VERIFIED', nc))
    print('todo markers: %d BLOCKED, %d NOT VERIFIED' % (blocked, notver))
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
