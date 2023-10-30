#!/bin/bash

cd "monitoring_interface"

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

python -m http.server $PORT & SERVER_PID=$!
sleep 3

# Open the URL in the default browser
xdg-open "http://localhost:$PORT/VESPA.html"&

wait $SERVER_PID
