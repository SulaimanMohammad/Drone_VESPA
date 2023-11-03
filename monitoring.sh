#!/bin/bash

cd "monitoring_interface"

# Define the target directory name
target_dir="PapaParse"

# Check if the target directory exists
if [ ! -d "$target_dir" ]; then
    echo "$target_dir does not exist. Downloading and renaming PapaParse from GitHub..."
    git clone https://github.com/mholt/PapaParse.git
    # Rename the downloaded directory
    mv PapaParse "$target_dir"

    echo "Download and rename complete."
else
    echo "$target_dir already exists."
fi

# Function to find the next available port starting from a given port
find_available_port() {
    local port=$1
    while true; do
        if ! nc -z localhost $port &>/dev/null; then
            echo $port
            return
        fi
        ((port++))
    done
}

# Function to handle script termination
cleanup() {
    echo "Stopping server..."
    kill $SERVER_PID
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM

# Find an available port starting from 8000
PORT=$(find_available_port 8000)

PYTHON_CMD=$(command -v python3 || command -v python)

if [ -z "$PYTHON_CMD" ]; then
    echo "Neither python3 nor python could be found. Please install Python."
    exit 1
fi

$PYTHON_CMD -m http.server $PORT & SERVER_PID=$!

sleep 3

# Open the URL in the default browser
xdg-open "http://localhost:$PORT/VESPA.html"&

wait $SERVER_PID
