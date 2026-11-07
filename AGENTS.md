# Repository instructions for Codex

When the user provides a robot or software fault resolution, or when you verify
one while working in this repository, add the reusable finding to
`docs/TROUBLESHOOTING.md` in the same turn. Include the symptom, confirmed cause
(or label a hypothesis), exact diagnostic check, fix, verification result, and
date when known. Update an existing entry instead of duplicating it. Do not
record passwords, tokens, private keys, or other secrets in the document.

Before editing the troubleshooting document, fetch the latest repository state
and integrate upstream changes without discarding local work. After editing,
commit and push the documentation change to the Git remote in the same turn.
If working on a feature branch, open or update its pull request for `main`.
If Git synchronization fails, preserve the local commit and report precisely
what remains unsynchronized.
