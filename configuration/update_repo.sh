#!/bin/bash

REPO_URL="https://github.com/SulaimanMohammad/Drone_VESPA.git"
# find the name of the RP 
PI_DRONE_DIR=$(ls /home | grep pi-drone)
CLONE_PATH="/home/$PI_DRONE_DIR/Drone_VESPA"


cp /home/$PI_DRONE_DIR/Drone_VESPA/configuration/update_repo.sh /home/$PI_DRONE_DIR

cd  /home/$PI_DRONE_DIR/

# If the branch name is passed as an argument, use it; otherwise, default to "main".
BRANCH_NAME=${1:-"main"}

# Navigate to the directory where your repository is located
cd /home/pi-$PI_DRONE_DIR/Drone_VESPA

# Check if the repository is already cloned
if [ ! -d "$CLONE_PATH" ]; then
    #git clone "$REPO_URL" "$CLONE_PATH"
    git clone -b "$BRANCH_NAME" "$REPO_URL" "$CLONE_PATH"

fi

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
