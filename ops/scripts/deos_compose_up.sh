#!/usr/bin/env bash
set -euo pipefail

# Repo kökü: ortamda REPO_ROOT verilmediyse bu scriptin bulunduğu .../ops/scripts/ -> iki üst dizin
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
cd "$REPO_ROOT"

exec docker compose \
  -f docker-compose.yml \
  -f ops/compose/docker-compose.autostart.yml \
  up --remove-orphans

