#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run this script with 'source', for example:" >&2
  echo "  source scripts/use_local_isaacgym.sh /path/to/isaacgym roboschool" >&2
  exit 1
fi

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

find_isaacgym_root() {
  local candidate
  for candidate in \
    "${1:-}" \
    "${ISAACGYM_PATH:-}" \
    "${REPO_ROOT}/docker/isaac-gym/isaacgym" \
    "${HOME}/isaacgym" \
    "/opt/isaacgym"
  do
    if [[ -n "${candidate}" && -d "${candidate}/python/isaacgym" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

prepend_path_var() {
  local var_name="$1"
  local new_path="$2"
  local current_value="${!var_name:-}"

  if [[ -z "${new_path}" || ! -e "${new_path}" ]]; then
    return 0
  fi

  case ":${current_value}:" in
    *":${new_path}:"*) ;;
    *)
      if [[ -n "${current_value}" ]]; then
        printf -v "${var_name}" '%s:%s' "${new_path}" "${current_value}"
      else
        printf -v "${var_name}" '%s' "${new_path}"
      fi
      export "${var_name}"
      ;;
  esac
}

ISAACGYM_ROOT="$(find_isaacgym_root "${1:-}")" || {
  echo "Isaac Gym not found." >&2
  echo "Pass the path explicitly or set ISAACGYM_PATH." >&2
  echo "Expected layout: <isaacgym>/python/isaacgym" >&2
  return 1
}

ENV_NAME="${2:-roboschool}"

if ! command -v conda >/dev/null 2>&1; then
  echo "conda is not available in PATH." >&2
  return 1
fi

CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "${CONDA_BASE}/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  echo "Creating conda environment '${ENV_NAME}' with Python 3.8"
  conda create -y -n "${ENV_NAME}" python=3.8
fi

conda activate "${ENV_NAME}"

python -m pip install --upgrade pip setuptools wheel
python -m pip install -e "${REPO_ROOT}"
python -m pip install -e "${ISAACGYM_ROOT}/python"

prepend_path_var LD_LIBRARY_PATH "${CONDA_PREFIX}/lib"

export ISAACGYM_PATH="${ISAACGYM_ROOT}"

echo
echo "Local Isaac Gym environment is ready."
echo "  conda env:      ${ENV_NAME}"
echo "  ISAACGYM_PATH:  ${ISAACGYM_PATH}"
echo "  CONDA_PREFIX:   ${CONDA_PREFIX}"
echo "  LD_LIBRARY_PATH includes: ${CONDA_PREFIX}/lib"
echo
echo "Run one of:"
echo "  python scripts/controller.py --steps 15000 --seed 0"
echo "  python ros2_isaac_bridge/sim_side/isaac_controller.py"
