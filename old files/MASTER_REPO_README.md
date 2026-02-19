Master repository conversion helper
=================================

Purpose
-------
This repository helper converts git submodules into in-repo directories using git-subtree so a single "master" repository contains the code and history from sub-repositories.

Files added
-----------
- `create_master_repo.sh`: Script to detect submodules from `.gitmodules` and import them as git-subtrees. Runs in dry-run mode by default.

Quick usage
-----------
1. Inspect the planned actions (dry-run):

```bash
cd /home/rob/px4_ros_ws3
./create_master_repo.sh
```

2. Perform the conversion (make sure you have a clean working tree and a backup remote):

```bash
./create_master_repo.sh --apply
```

3. If you want to squash subtree histories into single commits, add `--squash`:

```bash
./create_master_repo.sh --apply --squash
```

Notes & safety
--------------
- The script edits `.gitmodules`, removes submodule metadata and commits after importing each subtree. Keep a backup of your repository or push your current branch to a safe remote before running with `--apply`.
- The script attempts to detect the main branch for each submodule; verify the chosen branch before running.
- After conversion, verify file contents and history, then push the consolidated repository to your remote.

If you'd like, I can run the dry-run here or help run the conversion interactively—tell me which branch and whether you want `--squash`.
