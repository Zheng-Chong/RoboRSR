# RoboDojo — Agent Guide

Project conventions, layout, eval commands, and validation loops live in `CLAUDE.md`; follow it as well.

## Session Handoff (required)

`HANDOFF.md` at the repo root carries context across sessions.

- **Before** editing: read `HANDOFF.md` for current branch, goal, decisions, and open issues.
- **After** finishing any task that changes files: update `HANDOFF.md` — date, current state, what was done, key decisions, validation actually run (or why not), remaining issues/risks, and the entry point for the next session. Keep its existing sections; trim stale detail instead of only appending.
- Include the updated `HANDOFF.md` in the same commit as the change. If it conflicts with the code, Git, or test results, those win — fix the file.
