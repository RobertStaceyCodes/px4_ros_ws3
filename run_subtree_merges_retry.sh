#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/rob/px4_ros_ws_broken_20260211_221222"
repos=(
  "$ROOT/PX4-Autopilot"
  "$ROOT/PX4-Autopilot/src/modules/mavlink/mavlink"
  "$ROOT/PX4-Autopilot/Tools/simulation/jsbsim/jsbsim_bridge"
  "$ROOT/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
  "$ROOT/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/external/OpticalFlow"
  "$ROOT/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim"
  "$ROOT/PX4-Autopilot/Tools/simulation/flightgear/flightgear_bridge"
)

for repo in "${repos[@]}"; do
  if [[ ! -d "$repo" ]]; then
    echo "Skipping missing repo: $repo"
    continue
  fi
  if [[ ! -f "$repo/.gitmodules" ]]; then
    echo "Skipping (no .gitmodules): $repo"
    continue
  fi

  echo "\n--- Repo: $repo ---"

  git -C "$repo" config -f .gitmodules --get-regexp '^submodule\..*\.path' | while read -r key val; do
    name=$(printf "%s" "$key" | sed -E 's/submodule\.([^.]*)\.path.*/\1/')
    path="$val"
    url=$(git -C "$repo" config -f .gitmodules --get "submodule.$name.url")
    branch=$(git -C "$repo" config -f .gitmodules --get "submodule.$name.branch" || echo master)
    remote_name="subtree-$name"

    echo "Submodule: name=$name path=$path url=$url branch=$branch remote=$remote_name"

    # ensure remote exists and is fetched
    if ! git -C "$repo" remote get-url "$remote_name" >/dev/null 2>&1; then
      git -C "$repo" remote add "$remote_name" "$url" || true
    else
      git -C "$repo" remote set-url "$remote_name" "$url" || true
    fi
    git -C "$repo" fetch "$remote_name" --tags || true

    # determine the exact commit for the remote branch
    ref="refs/remotes/$remote_name/$branch"
    if git -C "$repo" rev-parse --verify "$ref" >/dev/null 2>&1; then
      commit=$(git -C "$repo" rev-parse "$ref")
    else
      # try to find a matching remote ref
      commit=$(git -C "$repo" ls-remote "$remote_name" "$branch" | awk '{print $1}' | head -n1 || true)
    fi

    if [[ -z "$commit" ]]; then
      echo "Could not resolve a commit for $remote_name/$branch; skipping."
      continue
    fi

    echo "Merging commit $commit into prefix $path"
    set +e
    git -C "$repo" subtree merge --prefix="$path" "$commit"
    rc=$?
    set -e
    if [[ $rc -ne 0 ]]; then
      echo "Warning: subtree merge failed for $path (repo: $repo). Manual resolution may be required."
    else
      echo "Merged $commit into $path"
    fi
  done
done

echo "Retry pass complete. Review repos for conflicts and run 'git status' where needed."
