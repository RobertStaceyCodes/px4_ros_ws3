#!/bin/bash
# Run this once after cloning the repo to fix PX4-Autopilot build requirements.
# Usage: ./setup_after_clone.sh

set -e
cd "$(dirname "$0")"

echo "Setting up PX4-Autopilot .git pointer for CMake version detection..."
if [ ! -e PX4-Autopilot/.git ]; then
    echo "gitdir: ../.git" > PX4-Autopilot/.git
    echo "  Created PX4-Autopilot/.git pointer"
else
    echo "  PX4-Autopilot/.git already exists, skipping"
fi

echo "Done. You can now build PX4 SITL."
