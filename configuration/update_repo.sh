#!/bin/bash
REPO_URL="https://github.com/SulaimanMohammad/Drone_VESPA.git"
# fin the name of the RP 
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

cd /home/$PI_DRONE_DIR/Drone_VESPA/
# Check if the "log" directory exists
if [ ! -d "log" ]; then
  mkdir mission_log
  sudo chown -R $USER log
fi

# Set the id of the drone 
# Get the hostname
hostname=$(hostname)
# Name of file where the id is set 
output_file="/home/$PI_DRONE_DIR/Drone_VESPA/src/drone/Operational_Data.txt"
# Get just the filename from the output_file path
output_filename=$(basename "$output_file")
# Use regular expression to extract the number after "pi-drone"
if [[ $hostname =~ pi-drone([0-9]+) ]]; then
  number=${BASH_REMATCH[1]}
else
  number=""
fi
# Check if a number was found
if [ -n "$number" ]; then
  # Check if the file contains an "id=" line
  if grep -q 'id=' "$output_file"; then
    # Update the "id=" line with the new number
    sed -i "s/id=[0-9]*/id=$number/" "$output_file"
    echo "Updated id=$number in $output_filename"
  else
    # Append a new "id=" line to the file
    echo "id=$number" >> "$output_file"
    echo "Appended id=$number to $output_filename"
  fi
else
  echo "No number found in the hostname."
fi