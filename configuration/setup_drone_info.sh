#!/bin/bash

#------------------------------------------
#----- Drone dim and object avoidance -----
#------------------------------------------

# Define the path of the file
FILE_PATH="../src/Operational_Data.txt"
INO_PATH="../src/Lidar/ESP/DBSCAN_lidar_3D.ino"
read_dimensions() {
    grep "$1=" $FILE_PATH | awk -F '=' '{print $2}'
}

# Read the current dimensions
drone_height=$(read_dimensions 'drone_hight')
drone_length=$(read_dimensions 'drone_length')
drone_width=$(read_dimensions 'drone_width')

# Function to add color
color_echo() {
    COLOR=$1
    TEXT=$2
    echo -ne "\033[${COLOR}m${TEXT}\033[0m"
}

# Ask the user to confirm the dimensions
echo -n "Are your drone dimensions as follows? "
color_echo "1;34" "Length: $drone_length, Width: $drone_width, Height: ${drone_height} cm\n"
color_echo "0;33" "Do you want to change dimensions? [yes/no] "

read answer

# Handle user response
case $answer in
    [Yy] | [Yy][Ee][Ss])
        echo "Please enter the new dimensions."

        # Update drone length
        echo "Enter drone length in cm:"
        read new_length
        sed -i "s/drone_length=$drone_length/drone_length=$new_length/" $FILE_PATH
        drone_length=$new_length

        # Update drone width
        echo "Enter drone width in cm:"
        read new_width
        sed -i "s/drone_width=$drone_width/drone_width=$new_width/" $FILE_PATH
        drone_width=$new_width

        # Update drone height
        echo "Enter drone height in cm:"
        read new_height
        sed -i "s/drone_hight=$drone_height/drone_hight=$new_height/" $FILE_PATH
        drone_height=$new_height

        echo "Dimensions updated."
        ;;
    *)
        echo "No changes made."
        ;;
esac


# Calculate x as the maximum of width and length
x=$(echo "$drone_width $drone_length" | awk '{print ($1>$2)?$1:$2}')

# Calculate argent_warning_distance
argent_warning_distance=$(echo "$x * 1.5" | bc | awk '{print int($1+0.5)}')

# Update the value in DBSCAN_lidar_3D.ino
sed -i "s/#define argent_warning_distance .*/#define argent_warning_distance $argent_warning_distance/" $INO_PATH

echo "Updated argent_warning_distance to $argent_warning_distance in DBSCAN_lidar_3D.ino"


#------------------------------------------
#------------ set id of drone -------------
#------------------------------------------

cd /home/$PI_DRONE_DIR/Drone_VESPA/
# Check if the "log" directory exists
if [ ! -d "mission_log" ]; then
  mkdir mission_log
  sudo chown -R $USER mission_log
fi

# Set the id of the drone 
id_username=$(whoami)
# Name of file where the id is set 
output_file="/home/$PI_DRONE_DIR/Drone_VESPA/src/Operational_Data.txt"
# Get just the filename from the output_file path
output_filename=$(basename "$output_file")
# Extract the number after "pi-drone"
if [[ $id_username =~ pi-drone([0-9]+) ]]; then
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
  echo "No id found"
fi