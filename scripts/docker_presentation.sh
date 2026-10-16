#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ADAS_UID="$(id -u)"
export ADAS_GID="$(id -g)"
ADAS_CONTEXT="${ADAS_DOCKER_CONTEXT:-desktop-linux}"
COMPOSE_FILE="$REPO_ROOT/docker/presentation.compose.yaml"
if [[ "$ADAS_CONTEXT" == desktop-linux ]]; then
  COMPOSE_FILE="$REPO_ROOT/docker/presentation.desktop.compose.yaml"
fi
COMPOSE=(docker --context "$ADAS_CONTEXT" compose -f "$COMPOSE_FILE")
action="${1:-status}"
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
  build) "${COMPOSE[@]}" build adas ;;
  start)
    mkdir -p "$REPO_ROOT/.runtime/carla-saved"
    "${COMPOSE[@]}" build adas
    if [[ "$ADAS_CONTEXT" == desktop-linux ]]; then
      "${COMPOSE[@]}" up -d adas
    else
      "${COMPOSE[@]}" up -d adas carla
    fi
    ;;
  shell) "${COMPOSE[@]}" exec adas /usr/local/bin/adas-entrypoint bash ;;
  exec) "${COMPOSE[@]}" exec -T adas /usr/local/bin/adas-entrypoint "$@" ;;
  preview)
    PREVIEW_CARLA_HOST=127.0.0.1
    PREVIEW_LISTEN_HOST=127.0.0.1
    if [[ "$ADAS_CONTEXT" == desktop-linux ]]; then
      PREVIEW_CARLA_HOST=host.docker.internal
      PREVIEW_LISTEN_HOST=0.0.0.0
    fi
    "${COMPOSE[@]}" exec -T adas /usr/local/bin/adas-entrypoint python scripts/preview_carla_comparison.py \
      --host "$PREVIEW_CARLA_HOST" --port 2020 \
      --listen-host "$PREVIEW_LISTEN_HOST" --listen-port 8765 "$@"
    ;;
  simulator)
    mkdir -p "$REPO_ROOT/.runtime/carla-saved"
    docker --context default compose -f "$REPO_ROOT/docker/presentation.compose.yaml" build adas
    docker --context default compose -f "$REPO_ROOT/docker/presentation.compose.yaml" up -d carla
    ;;
  simulator-stop)
    docker --context default compose -f "$REPO_ROOT/docker/presentation.compose.yaml" stop carla
    ;;
  status) "${COMPOSE[@]}" ps ;;
  stop) "${COMPOSE[@]}" stop ;;
  *) printf 'Usage: %s {build|start|shell|exec COMMAND...|preview|status|stop|simulator|simulator-stop}\n' "$0" >&2; exit 2 ;;
esac
