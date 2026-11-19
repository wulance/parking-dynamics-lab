#!/usr/bin/env bash
# Run from the unzipped project folder AFTER creating an empty GitHub repository.
# Usage: bash publish-to-github.sh https://github.com/YOUR_USER/parking-dynamics-lab.git
set -euo pipefail
repo="${1:-}"
if [[ -z "$repo" ]]; then
  echo "Usage: bash publish-to-github.sh https://github.com/YOUR_USER/parking-dynamics-lab.git"
  exit 2
fi
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git init -b main
fi
git add .
if ! git diff --cached --quiet; then
  git commit -m "feat: add parking dynamics prototype and test suite"
fi
git branch -M main
if git remote get-url origin >/dev/null 2>&1; then
  git remote set-url origin "$repo"
else
  git remote add origin "$repo"
fi
git push -u origin main
printf '\nPublished to %s\n' "$repo"
