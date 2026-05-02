#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/opt/deos}"
cd "$REPO_ROOT"

exec docker compose \
  -f docker-compose.yml \
  -f ops/compose/docker-compose.autostart.yml \
  up --remove-orphans

