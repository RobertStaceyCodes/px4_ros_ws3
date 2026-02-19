#!/bin/bash
# Replace PX4-Autopilot submodule with actual files so they push to GitHub.
# Run from repo root: ./fix_px4_submodule.sh

set -e
cd "$(dirname "$0")"

echo "Removing PX4-Autopilot submodule reference (files stay on disk)..."
git rm --cached PX4-Autopilot

echo "Adding PX4-Autopilot as normal files (this may take a minute)..."
git add PX4-Autopilot/

echo "Status:"
git status -s | head -20

echo ""
echo "Commit with:"
echo "  git commit -m 'Replace PX4-Autopilot submodule with in-repo files'"
echo "Then push:"
echo "  git push"
