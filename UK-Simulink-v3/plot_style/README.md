# plot_style

Copies of the author's house figure style, unchanged, so that `fig_9dof_cases.m` runs from the lab repo
without the author's review folder.

Source: `Research/plot_style/` of the author's review tree (not in this repository),
copied 19 November 2026. The rules are in `PLOT_STYLE_RULES.md` there (not copied; no code needs it).

| File | SHA-256 | Called by |
|---|---|---|
| paper_style.m | 162b6e4cf266dd54844020e819a5e15199531a01195e91ea665e00fa6c37a456 | fig_9dof_cases.m |
| paper_figure.m | dd1f189d721e3646d0119bfc346991eeb5e2cde19fc76f98bcd29f59f295beb4 | fig_9dof_cases.m |
| paper_export.m | c4c555af752534a2774449c54617c2381c307078f1bf4a4a42910240707e33d5 | fig_9dof_cases.m |
| paper_check.m | 3748f45b7c22916bfaf3995901963c106a422361edf1ea0cd00150012d53ef59 | paper_export.m:24 (unconditional call) |

paper_check.m was not in the list of three first asked for. It is added because paper_export calls it on every export,
so paper_export fails without it. paper_legend.m (also in the source folder) is not called by any of these files or by
fig_9dof_cases.m and is not copied.

Do not edit these files here. Change the source and copy again, then update the hashes.
`fig_9dof_cases.m` adds this folder to the path and falls back to the review-folder path only if this folder is missing.
