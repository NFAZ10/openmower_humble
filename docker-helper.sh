#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${SCRIPT_DIR}/.env"
ENV_EXAMPLE="${SCRIPT_DIR}/.env.example"

CORE_SERVICES=(mosquitto open_mower_humble openmower_ui_bridge_humble)
ALL_SERVICES=(mosquitto open_mower_humble openmower_ui_bridge_humble openmower_ui_humble)

die() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

info() {
  printf '%s\n' "$*"
}

require_no_extra_args() {
  if [ "$#" -gt 0 ]; then
    die "Unexpected extra arguments: $*"
  fi
}

detect_compose() {
  if docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD=(docker compose)
  elif docker-compose version >/dev/null 2>&1; then
    COMPOSE_CMD=(docker-compose)
  else
    die "Docker Compose not found. Install Docker Compose v2 ('docker compose') or legacy 'docker-compose'."
  fi
}

require_docker() {
  if ! docker info >/dev/null 2>&1; then
    die "Docker daemon is not running or not reachable."
  fi
}

ensure_env_file() {
  if [ -f "${ENV_FILE}" ]; then
    return
  fi

  if [ ! -f "${ENV_EXAMPLE}" ]; then
    die "Missing ${ENV_FILE} and ${ENV_EXAMPLE}; cannot initialize configuration."
  fi

  cp "${ENV_EXAMPLE}" "${ENV_FILE}"
  info "Created ${ENV_FILE} from ${ENV_EXAMPLE}."
}

compose() {
  ensure_env_file
  "${COMPOSE_CMD[@]}" \
    --project-directory "${SCRIPT_DIR}" \
    -f "${SCRIPT_DIR}/docker-compose.yaml" \
    -f "${SCRIPT_DIR}/docker-compose.ui.yml" \
    "$@"
}

container_name_for_target() {
  case "${1:-main}" in
    main|core|open_mower_humble)
      printf 'open_mower_humble\n'
      ;;
    bridge|ui-bridge|openmower_ui_bridge_humble)
      printf 'openmower_ui_bridge_humble\n'
      ;;
    mqtt|mosquitto|openmower_mqtt_humble)
      printf 'openmower_mqtt_humble\n'
      ;;
    ui|web|openmower_ui_humble)
      printf 'openmower_ui_humble\n'
      ;;
    *)
      return 1
      ;;
  esac
}

shell_for_container() {
  case "$1" in
    openmower_ui_humble|openmower_mqtt_humble)
      printf 'sh\n'
      ;;
    *)
      printf 'bash\n'
      ;;
  esac
}

services_for_target() {
  case "${1:-all}" in
    all)
      printf '%s\n' "${ALL_SERVICES[@]}"
      ;;
    core)
      printf '%s\n' "${CORE_SERVICES[@]}"
      ;;
    main|open_mower_humble)
      printf 'open_mower_humble\n'
      ;;
    bridge|ui-bridge|openmower_ui_bridge_humble)
      printf 'openmower_ui_bridge_humble\n'
      ;;
    mqtt|mosquitto)
      printf 'mosquitto\n'
      ;;
    ui|web|openmower_ui_humble)
      printf 'openmower_ui_humble\n'
      ;;
    *)
      return 1
      ;;
  esac
}

load_target_services() {
  local target="$1"
  mapfile -t TARGET_SERVICES < <(services_for_target "${target}") || die "Unknown target '${target}'."
}

parse_detached_flag() {
  DETACHED_ARGS=()
  if [ "${1:-}" = "-d" ] || [ "${1:-}" = "--detached" ]; then
    DETACHED_ARGS=(-d)
    shift
  fi
  REMAINING_ARGS=("$@")
}

show_help() {
  cat <<EOF
OpenMower Humble Docker Helper

Usage:
  $(basename "$0") <command> [options]

Commands:
  init-env                  Create .env from .env.example if needed
  build [core|ui|all]       Build images (default: core)
  up [-d]                   Start the core stack: mosquitto, mower, UI bridge
  up-build [-d]             Start the core stack and rebuild first
  run [-d]                  Alias for 'up'
  run-build [-d]            Alias for 'up-build'
  ui [-d]                   Start the optional web UI
  up-ui [-d]                Alias for 'ui'
  all [-d]                  Start core stack plus optional web UI
  all-build [-d]            Start everything and rebuild first
  logs [target]             Follow logs for all, core, main, bridge, mqtt, or ui
  ps                        Show compose container status
  status                    Alias for 'ps'
  shell [target]            Open a shell in main, bridge, mqtt, or ui (default: main)
  exec <target> <command>   Run an arbitrary command in a running container
  restart [target]          Restart all, core, main, bridge, mqtt, or ui
  stop [target]             Stop all, core, main, bridge, mqtt, or ui
  down                      Stop and remove the compose project
  clean                     Down the project and remove locally built images
  compose <args...>         Pass arguments straight to docker compose
  help                      Show this help text

Examples:
  $(basename "$0") init-env
  $(basename "$0") build
  $(basename "$0") up-build -d
  $(basename "$0") ui -d
  $(basename "$0") logs core
  $(basename "$0") shell bridge
  $(basename "$0") exec main ros2 topic list
EOF
}

detect_compose

command="${1:-help}"
if [ "$#" -gt 0 ]; then
  shift
fi

case "${command}" in
  init-env)
    require_no_extra_args "$@"
    ensure_env_file
    ;;

  build)
    require_docker
    build_target="${1:-core}"
    if [ "$#" -gt 0 ]; then
      shift
    fi
    require_no_extra_args "$@"
    case "${build_target}" in
      core)
        info "Building OpenMower core image..."
        compose build open_mower_humble
        ;;
      ui)
        info "Building optional OpenMower UI image..."
        compose build openmower_ui_humble
        ;;
      all)
        info "Building OpenMower core and UI images..."
        compose build open_mower_humble openmower_ui_humble
        ;;
      *)
        die "Unknown build target '${build_target}'. Use core, ui, or all."
        ;;
    esac
    ;;

  up|run)
    require_docker
    parse_detached_flag "$@"
    require_no_extra_args "${REMAINING_ARGS[@]}"
    info "Starting OpenMower core stack..."
    compose up "${DETACHED_ARGS[@]}" "${CORE_SERVICES[@]}"
    ;;

  up-build|run-build)
    require_docker
    parse_detached_flag "$@"
    require_no_extra_args "${REMAINING_ARGS[@]}"
    info "Starting OpenMower core stack with rebuild..."
    compose up --build "${DETACHED_ARGS[@]}" "${CORE_SERVICES[@]}"
    ;;

  ui|up-ui)
    require_docker
    parse_detached_flag "$@"
    require_no_extra_args "${REMAINING_ARGS[@]}"
    info "Starting optional OpenMower UI..."
    compose up "${DETACHED_ARGS[@]}" openmower_ui_humble
    ;;

  all)
    require_docker
    parse_detached_flag "$@"
    require_no_extra_args "${REMAINING_ARGS[@]}"
    info "Starting OpenMower core stack and optional UI..."
    compose up "${DETACHED_ARGS[@]}" "${ALL_SERVICES[@]}"
    ;;

  all-build)
    require_docker
    parse_detached_flag "$@"
    require_no_extra_args "${REMAINING_ARGS[@]}"
    info "Starting OpenMower core stack and optional UI with rebuild..."
    compose up --build "${DETACHED_ARGS[@]}" "${ALL_SERVICES[@]}"
    ;;

  logs)
    require_docker
    target="${1:-all}"
    if [ "$#" -gt 0 ]; then
      shift
    fi
    require_no_extra_args "$@"
    if [ "${target}" = "core" ]; then
      info "Following core service logs..."
      compose logs -f "${CORE_SERVICES[@]}"
    elif [ "${target}" = "all" ]; then
      info "Following all compose logs..."
      compose logs -f
    else
      load_target_services "${target}"
      info "Following logs for ${target}..."
      compose logs -f "${TARGET_SERVICES[@]}"
    fi
    ;;

  ps|status)
    require_docker
    require_no_extra_args "$@"
    info "Compose service status:"
    compose ps --all
    printf '\nBuilt images:\n'
    docker image ls --format 'table {{.Repository}}\t{{.Tag}}\t{{.CreatedSince}}\t{{.Size}}' \
      | grep -E '^(REPOSITORY|openmower-humble|openmower-humble-openmower_ui_humble)' || true
    ;;

  shell)
    require_docker
    target="${1:-main}"
    if [ "$#" -gt 0 ]; then
      shift
    fi
    require_no_extra_args "$@"
    container="$(container_name_for_target "${target}")" || die "Unknown shell target '${target}'."
    shell_bin="$(shell_for_container "${container}")"
    info "Opening ${shell_bin} in ${container}..."
    exec docker exec -it "${container}" "${shell_bin}"
    ;;

  exec)
    require_docker
    [ "$#" -ge 2 ] || die "Usage: $(basename "$0") exec <target> <command> [args...]"
    target="$1"
    shift
    container="$(container_name_for_target "${target}")" || die "Unknown exec target '${target}'."
    info "Running command in ${container}..."
    exec docker exec -it "${container}" "$@"
    ;;

  restart)
    require_docker
    target="${1:-all}"
    if [ "$#" -gt 0 ]; then
      shift
    fi
    require_no_extra_args "$@"
    if [ "${target}" = "core" ]; then
      load_target_services core
    else
      load_target_services "${target}"
    fi
    info "Restarting ${target}..."
    compose restart "${TARGET_SERVICES[@]}"
    ;;

  stop)
    require_docker
    target="${1:-all}"
    if [ "$#" -gt 0 ]; then
      shift
    fi
    require_no_extra_args "$@"
    if [ "${target}" = "core" ]; then
      load_target_services core
    else
      load_target_services "${target}"
    fi
    info "Stopping ${target}..."
    compose stop "${TARGET_SERVICES[@]}"
    ;;

  down)
    require_docker
    require_no_extra_args "$@"
    info "Stopping and removing the OpenMower compose project..."
    compose down --remove-orphans
    ;;

  clean)
    require_docker
    require_no_extra_args "$@"
    info "Stopping the project and removing locally built images..."
    compose down --rmi local --remove-orphans
    ;;

  compose)
    require_docker
    info "Passing through to docker compose..."
    compose "$@"
    ;;

  help|-h|--help)
    show_help
    ;;

  *)
    show_help
    die "Unknown command '${command}'."
    ;;
esac
