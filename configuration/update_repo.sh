#!/bin/bash

REPO_URL="https://github.com/SulaimanMohammad/Drone_VESPA.git"
# find the name of the RP 
PI_DRONE_DIR=$(ls /home | grep drone)
CLONE_PATH="/home/$PI_DRONE_DIR/Drone_VESPA"


cp /home/$PI_DRONE_DIR/Drone_VESPA/configuration/update_repo.sh /home/$PI_DRONE_DIR

cd  /home/$PI_DRONE_DIR/

# If the branch name is passed as an argument, use it; otherwise, default to "main".
BRANCH_NAME=${1:-"main"}

# Check if the repository is already cloned and remove it if it exists
if [ -d "$CLONE_PATH" ]; then
    echo "Repository exists. Removing..."
    rm -rf "$CLONE_PATH"
fi

# Clone the repository
git clone -b "$BRANCH_NAME" "$REPO_URL" "$CLONE_PATH"

# Adjust the ownership:
sudo chown -R $USER:$USER "$CLONE_PATH"


# Navigate to the directory
cd "$CLONE_PATH" || exit # prevent the script from continuing if it can't enter the desired directory


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

."$CLONE_PATH"/configuration/setup_drone_info.sh 