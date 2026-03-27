#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "Usage: $0 <env_name> <map_kind>"
  echo "  env_name: env_office | env_simple_maze | env_warehouse"
  echo "  map_kind: true | test"
  exit 1
fi

ENV_NAME="$1"
MAP_KIND="$2"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAP_TARGET="$REPO_ROOT/maps/$ENV_NAME/$MAP_KIND"
NOTES_TARGET="$REPO_ROOT/notes/$ENV_NAME"

if [ ! -d "$MAP_TARGET" ]; then
  echo "Map target does not exist: $MAP_TARGET"
  exit 1
fi

if [ ! -d "$NOTES_TARGET" ]; then
  echo "Notes target does not exist: $NOTES_TARGET"
  exit 1
fi

ln -sfn "$MAP_TARGET" "$REPO_ROOT/maps/current"
ln -sfn "$NOTES_TARGET" "$REPO_ROOT/notes/current"

echo "maps/current  -> $(readlink -f "$REPO_ROOT/maps/current")"
echo "notes/current -> $(readlink -f "$REPO_ROOT/notes/current")"
