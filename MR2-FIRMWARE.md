# mr2-firmware

This repository holds rusEFI firmware for a Formula SAE car running a uaEFI ECU
with an electronic throttle body and four wheel speed sensors. The work in
progress is closed-loop traction control; see
[docs/traction-control-plan.md](docs/traction-control-plan.md).

Everything except this file and `docs/traction-control-plan.md` is upstream
rusEFI. The upstream project README is [README.md](README.md).

## Upstream base

The first commit is a snapshot of upstream rusEFI, not a fork with history.
Its tree is byte-identical to the upstream commit below. The ~44k commits of
upstream history were left out to keep the repository small.

| Synced on  | Upstream commit (rusefi/rusefi `master`)   | Commit in this repo |
|------------|--------------------------------------------|---------------------|
| 2026-10-16 | `6cf917c59e68c75c8fa0ea0c08f841f116629b25` | `aa19d3ca81d3` (import) |

Each import or sync commit also carries an `Upstream-Commit:` trailer.

## Getting started

```bash
git clone https://github.com/adam-munch/mr2-firmware.git
cd mr2-firmware
git submodule update --init --recursive   # ChibiOS, libfirmware, lua, openblt, googletest, ...

# Host unit tests (gcc or clang)
cd unit_tests && ./test.sh

# uaEFI (STM32F4, 1 MB flash) and uaEFI Pro (STM32F7, 2 MB flash).
# These need the Arm GNU toolchain; see .github/workflows/build-firmware.yaml for the pinned version.
cd firmware/config/boards/hellen/uaefi
./compile_firmware.sh        # -> firmware/deliver/rusefi.bin, rusefi_update.srec
./compile_firmware_pro.sh
```

## Syncing with upstream rusEFI

This procedure was tested on a scratch copy: two consecutive syncs kept the
local changes and otherwise produced exactly the upstream tree. There is no
shared history with upstream, so a temporary, local-only graft tells git which
upstream commit the last import or sync corresponds to. The squash merge then
gets the correct merge base without pulling upstream history into this
repository.

```bash
# one-time
git remote add upstream https://github.com/rusefi/rusefi.git

# Blobless fetch: commits and trees only; file contents are fetched on demand.

git fetch --filter=blob:none upstream master
BASE=<upstream commit from the last row of the table above>
LAST=<commit in this repo from the same row>

# Local-only graft: pretend LAST also has BASE as a parent.
git replace --graft "$LAST" $(git rev-parse "$LAST^@") "$BASE"
git merge --squash upstream/master
# resolve conflicts, then rebuild and run unit_tests/test.sh
git commit    # "Sync rusEFI upstream to <sha>", with an "Upstream-Commit: <sha>" trailer
git replace -d "$LAST"
```

Then add a row to the table above with the new upstream SHA and the sync
commit. Never push `refs/replace/*`.

## Sending changes upstream

Traction control is meant to be general enough to upstream. To build a
pull-request branch, fork `rusefi/rusefi`, branch it at the upstream commit
this repository is synced to, and replay the local commits:

```bash
git format-patch <last import/sync commit>..HEAD -- . ':!MR2-FIRMWARE.md' ':!docs/traction-control-plan.md'
# in the rusefi fork, on a branch at the same upstream commit:
git am -3 /path/to/*.patch
```

## Upstream agent guidance

`CLAUDE.md` and `AGENTS.md` are upstream files and apply to code in this tree
(build quirks, test-driven bugfixing, no heap, tagging conventions, ...). One
of their rules is upstream policy rather than a technical constraint: AI agents
must never commit or push. Decide whether that rule should apply here. If AI
sessions should commit to this repository, say so when you start the session,
or add a short override to `CLAUDE.md`. That edit can then conflict during
upstream syncs.
