#!/usr/bin/env bash
# CPU checks for every commit, also run by CI (.github/workflows/checks.yml): bash scripts/check.sh [BASE_REF]
# Lints only the Python files changed since BASE_REF (default origin/main; older files predate the formatter), then
# runs the unit tests and the task inventory check. Needs ruff==0.11.7 (as .pre-commit-config.yaml), pytest, numpy,
# scipy, h5py, opencv-python-headless, pyyaml, shapely, rtree and transforms3d; the XPolicyLab submodule for the
# trajectory format test.
# GPU checks (scripted demos, collection, replay) run on a server; see scripts/datagen/SKILL.md.
set -euo pipefail
cd "$(dirname "$0")/.."
base="${1:-origin/main}"
git rev-parse -q --verify "$base^{commit}" >/dev/null || base=origin/main
files=$(git diff --name-only --diff-filter=ACMR "$base" -- '*.py' | grep -vE '^(third_party|XPolicyLab|real2sim)/' || true)
if [ -n "$files" ]; then
    ruff check $files
    ruff format --check $files
fi
git diff --check "$base"
python -m pytest -q scripts/datagen scripts/scenes scripts/g1d/test_office_beans.py scripts/g1d/test_g1d_body.py scripts/g1d/test_randomize.py \
    src/real2sim_platform/test_server.py
python scripts/internal/task_inventory.py --format json --check >/dev/null
echo "checks passed"
