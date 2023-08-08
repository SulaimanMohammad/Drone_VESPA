#!/bin/bash

REPO_URL="https://github.com/SulaimanMohammad/Drone_VESPA.git"
CLONE_PATH="/home/pi-drone1/Drone_VESPA"

# Navigate to the directory where your repository is located
cd /home/pi-drone1/Drone_VESPA

# Check if the repository is already cloned
if [ ! -d "$CLONE_PATH" ]; then
    git clone "$REPO_URL" "$CLONE_PATH"
fi

# Navigate to the directory
cd "$CLONE_PATH"


# Configure Git to use the merge strategy for pulling
git config --global pull.rebase false

# Fetch updates from the remote repository
git fetch

# Check if there are any updates
UPSTREAM=${1:-'@{u}'}
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse "$UPSTREAM")
BASE=$(git merge-base @ "$UPSTREAM")

if [ $LOCAL = $REMOTE ]; then
    echo "Up-to-date"
elif [ $LOCAL = $BASE ]; then
    echo "Need to pull"
    git pull
else
    echo "Diverged"
fi

cd /home/pi-drone1/Drone_VESPA/unit_tests
# Check if the "log" directory exists
if [ ! -d "log" ]; then
  mkdir log
  sudo chown -R $USER log
fi

