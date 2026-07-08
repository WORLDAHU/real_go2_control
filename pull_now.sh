#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

branch="$(git branch --show-current)"

echo "== current branch =="
echo "$branch"

echo
echo "== local status =="
git status --short

if ! git diff --quiet || ! git diff --cached --quiet; then
  echo
  echo "Local changes detected."
  echo "Choose:"
  echo "  1) stash local changes, pull, then keep stash for later"
  echo "  2) cancel"
  read -p "Your choice [1/2]: " choice

  if [ "$choice" != "1" ]; then
    echo "Cancelled."
    exit 1
  fi

  git stash push -u -m "auto-stash before pull $(date '+%Y-%m-%d %H:%M:%S')"
fi

echo
echo "== pulling origin/$branch =="
git pull --ff-only origin "$branch"

echo
echo "Pulled latest origin/$branch"
echo
echo "If changes were stashed, see:"
echo "  git stash list"
echo "To restore later:"
echo "  git stash pop"
