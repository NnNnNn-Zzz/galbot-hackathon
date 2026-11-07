#!/bin/bash
# Run the staged charger wrist test on the HPU with its local YOLO runtime.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export PYTHONPATH="${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"

# shellcheck disable=SC1091
source "$ROOT/scripts/env_hpu.sh" >/dev/null

export PYTHONPATH="$ROOT:$HOME/charger-yolo-runtime:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$HOME/charger-yolo-libs:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1

exec python3 "$ROOT/scripts/test_charger_wrist_pregrasp.py" "$@"
