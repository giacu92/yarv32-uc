# Linux bring-up — running log

Companion to `docs/linux_assessment.md` (plan, criteria). This file records,
per milestone: what was done, what was measured, what was decided without
supervision (with the reasoning), and what remains. Newest entries are at the
bottom of each milestone. Dates are absolute.

Execution mode from 2026-11-22: **unattended**. The owner cannot answer
questions. Ambiguities are resolved conservatively and recorded here.

---

## Needs Giacomo

Physical or host actions that this session cannot do. Each has exact
commands. Nothing below blocks the milestones that do not depend on it.

*(entries are added as milestones produce them)*

---

## Standing decisions (2026-11-22)

| # | Decision | Reasoning |
|---|---|---|
| D1 | Docs and the progress log live on `linux-soc`. That branch starts from `main` at `b96a49b`, and `main-fixes` is merged into it (merge, never rebase) after M1 | `main` is never committed to. The plan documents are Linux work. A merge keeps both branches' history intact |
| D2 | `main-fixes` branches from `main` at `b96a49b` | It carries only the M1 fixes for the owner's review |

---

## M0 — Baseline

*(in progress)*
