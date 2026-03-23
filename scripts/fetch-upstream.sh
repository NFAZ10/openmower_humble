#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DIR="${ROOT_DIR}/upstream/openmowernext"
REPO_URL="${1:-https://github.com/NFAZ10/openmowernext.git}"
REF="${2:-3eb9edb2f8d29f5e28fdf242de78332c192d787a}"

if [ -d "${TARGET_DIR}/.git" ]; then
  git -C "${TARGET_DIR}" fetch --prune origin
else
  git clone "${REPO_URL}" "${TARGET_DIR}"
fi

git -C "${TARGET_DIR}" checkout "${REF}"
git -C "${TARGET_DIR}" rev-parse HEAD > "${ROOT_DIR}/upstream/COMMIT.txt"
