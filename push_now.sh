#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

echo "== current branch =="
git branch --show-current

echo
echo "== status =="
git status --short

echo
read -p "Commit message: " msg

if [ -z "$msg" ]; then
  msg="Update real_go2_control"
fi

git add -A

if git diff --cached --quiet; then
  echo "No changes to commit."
else
  git commit -m "$msg"
fi

branch="$(git branch --show-current)"
git push origin "$branch"

echo
echo "Pushed to origin/$branch"


