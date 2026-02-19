#!/usr/bin/env bash
set -euo pipefail

# create_master_repo.sh
# Convert git submodules into git-subtree directories inside the current repo.
# Usage:
#   ./create_master_repo.sh [--apply] [--squash]
# By default the script runs in dry-run mode and prints the planned actions.

DRY_RUN=1
SQUASH=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) DRY_RUN=0; shift ;;
    --squash) SQUASH=1; shift ;;
    -h|--help) echo "Usage: $0 [--apply] [--squash]"; exit 0 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: not a git repository (run from repository root)" >&2
  exit 2
fi

if [[ ! -f .gitmodules ]]; then
  echo ".gitmodules not found — nothing to convert."; exit 0
fi

echo "Scanning .gitmodules for submodules..."

mapfile -t SUBMODULE_PATHS < <(git config -f .gitmodules --get-regexp '^submodule\..*\.path' | awk '{print $2}')
mapfile -t SUBMODULE_NAMES < <(git config -f .gitmodules --get-regexp '^submodule\..*\.path' | sed -E 's/submodule\.([^.]*)\.path.*/\1/')

if [[ ${#SUBMODULE_PATHS[@]} -eq 0 ]]; then
  echo "No submodules found in .gitmodules"; exit 0
fi

for i in "${!SUBMODULE_PATHS[@]}"; do
  path="${SUBMODULE_PATHS[$i]}"
  name="${SUBMODULE_NAMES[$i]}"
  url=$(git config -f .gitmodules --get "submodule.$name.url")
  branch=$(git config -f .gitmodules --get "submodule.$name.branch" || true)
  if [[ -z "$branch" ]]; then
    # try to detect main branch on remote
    if git ls-remote --heads "$url" refs/heads/main | grep -q refs/heads/main 2>/dev/null; then
      branch=main
    else
      branch=master
    fi
  fi

  remote_name="subtree-$name"

  echo "\nFound submodule: name=$name path=$path url=$url branch=$branch"
  echo "  remote: $remote_name"

  if [[ $DRY_RUN -eq 1 ]]; then
    echo "  DRY-RUN: would run: git remote add $remote_name $url  (if missing)"
    echo "  DRY-RUN: would run: git fetch $remote_name --tags"
    if [[ $SQUASH -eq 1 ]]; then
      echo "  DRY-RUN: would run: git subtree add --prefix=$path $remote_name $branch --squash"
    else
      echo "  DRY-RUN: would run: git subtree add --prefix=$path $remote_name $branch"
    fi
    echo "  DRY-RUN: would remove submodule entries and commit"
    continue
  fi

  # Add or update remote
  if git remote get-url "$remote_name" >/dev/null 2>&1; then
    git remote set-url "$remote_name" "$url"
  else
    git remote add "$remote_name" "$url"
  fi

  git fetch "$remote_name" --tags

  # Choose branch that exists
  if ! git ls-remote --exit-code --heads "$remote_name" "$branch" >/dev/null 2>&1; then
    # try main/master fallbacks
    if git ls-remote --exit-code --heads "$remote_name" main >/dev/null 2>&1; then
      branch=main
    elif git ls-remote --exit-code --heads "$remote_name" master >/dev/null 2>&1; then
      branch=master
    else
      echo "  ERROR: remote $remote_name has no main/master/$branch branch"; exit 3
    fi
  fi

  echo "  Importing $remote_name/$branch into $path"
  if [[ $SQUASH -eq 1 ]]; then
    git subtree add --prefix="$path" "$remote_name" "$branch" --squash
  else
    git subtree add --prefix="$path" "$remote_name" "$branch"
  fi

  echo "  Removing submodule metadata for $path"
  git submodule deinit -f -- "$path" || true
  git rm -f --cached "$path" || true
  git config -f .gitmodules --remove-section "submodule.$name" || true
  git add .gitmodules || true
  if [[ ! -s .gitmodules ]]; then
    git rm --cached .gitmodules || true
    rm -f .gitmodules
  fi

  git add "$path"
  git commit -m "Import subtree: $path (from $url, branch $branch)" || true
done

echo "Import complete. Review the repository state and push to your remote when ready."
