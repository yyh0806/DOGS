#!/usr/bin/env bash
set -u

script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(CDPATH= cd -- "${script_dir}/.." && pwd -P)"
cd "${repo_root}" || exit 1

PASS_COUNT=0
FAIL_COUNT=0

find_python() {
  if [ -n "${PYTHON:-}" ]; then
    if "${PYTHON}" -c "import sys" >/dev/null 2>&1; then
      printf '%s\n' "${PYTHON}"
      return 0
    fi
  fi

  for candidate in python3 python py; do
    if command -v "${candidate}" >/dev/null 2>&1 \
      && "${candidate}" -c "import sys" >/dev/null 2>&1; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

PYTHON_BIN="$(find_python || true)"
NODE_BIN="$(command -v node || true)"

run_group() {
  name="$1"
  shift

  printf '\n==> %s\n' "${name}"
  if "$@"; then
    PASS_COUNT=$((PASS_COUNT + 1))
    printf 'PASS: %s\n' "${name}"
  else
    status=$?
    FAIL_COUNT=$((FAIL_COUNT + 1))
    printf 'FAIL: %s (exit %s)\n' "${name}" "${status}"
  fi
}

run_pytest() {
  if [ -z "${PYTHON_BIN}" ]; then
    printf 'python interpreter not found\n' >&2
    return 127
  fi
  "${PYTHON_BIN}" -m pytest "$@"
}

run_python_script() {
  if [ -z "${PYTHON_BIN}" ]; then
    printf 'python interpreter not found\n' >&2
    return 127
  fi
  "${PYTHON_BIN}" "$@"
}

run_node_script() {
  if [ -z "${NODE_BIN}" ]; then
    printf 'node executable not found\n' >&2
    return 127
  fi
  "${NODE_BIN}" "$@"
}

printf 'Product room person search verification\n'
printf 'Repo: %s\n' "${repo_root}"
printf 'Python: %s\n' "${PYTHON_BIN:-not found}"
printf 'Node: %s\n' "${NODE_BIN:-not found}"

product_tests=(
  web/test_product_command.py
  web/test_person_localizer.py
  web/test_person_mission.py
  web/test_active_search.py
  web/test_ai_snapshot_contract.py
  web/test_scan_snapshot_contract.py
  web/test_product_room_orchestrator.py
)

for test_file in "${product_tests[@]}"; do
  run_group "python ${test_file}" run_pytest "${test_file}"
done

run_group "node web/test_map_contract.js" run_node_script web/test_map_contract.js
run_group "python tools/test_stage_e.py" run_python_script tools/test_stage_e.py

TOTAL_COUNT=$((PASS_COUNT + FAIL_COUNT))

printf '\nSummary: %s PASS, %s FAIL, %s total\n' \
  "${PASS_COUNT}" "${FAIL_COUNT}" "${TOTAL_COUNT}"

if [ "${FAIL_COUNT}" -ne 0 ]; then
  exit 1
fi

exit 0
