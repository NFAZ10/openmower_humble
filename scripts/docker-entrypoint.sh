#!/usr/bin/env bash
set -euo pipefail

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

export OM_CONFIG_DIR="${OM_CONFIG_DIR:-/config}"
export OM_DATUM_LAT="${OM_DATUM_LAT:-30.0}"
export OM_DATUM_LONG="${OM_DATUM_LONG:-0.5}"
export OM_MAP_PATH="${OM_MAP_PATH:-/data/map.json}"

mkdir -p "$(dirname "${OM_MAP_PATH}")"

if [ ! -f "${OM_MAP_PATH}" ]; then
  printf '%s\n' '{"type":"FeatureCollection","features":[]}' > "${OM_MAP_PATH}"
fi

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
  source_setup "${WORKSPACE}/install/local_setup.bash"
elif [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source_setup "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
