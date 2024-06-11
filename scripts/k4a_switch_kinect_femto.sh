#!/usr/bin/env bash

# Function to display usage instructions
usage() {
    echo "Usage: $0 -m | -o"
    echo "-m : Use files for Microsoft Kinect Azure Kinect"
    echo "-o : Use files for ORBBEC Femto Cameras"
    exit 1
}

# Check if the script is run with superuser privileges
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or use sudo"
    exit 1
fi

# Check for the correct number of arguments
if [ "$#" -ne 1 ]; then
    usage
fi

# Set the source directory based on the argument
case "$1" in
    -m)
        SOURCE_DIR="/usr/lib/x86_64-linux-gnu/k4a_microsoft/"
        ;;
    -o)
        SOURCE_DIR="/usr/lib/x86_64-linux-gnu/k4a_orbbec/"
        ;;
    *)
        usage
        ;;
esac

# Set the destination directory
DEST_DIR="/usr/lib/x86_64-linux-gnu/"

# Copy files from the source to the destination, resolving any file duplication issues
for file in "$SOURCE_DIR"*; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        dest_file="$DEST_DIR$filename"
        
        if [ -f "$dest_file" ]; then
            # echo "Removing existing file: $dest_file"
            rm -f "$dest_file"
        fi
        
        # echo "Copying $file to $DEST_DIR"
        cp "$file" "$DEST_DIR"
    fi
done

case "$1" in
    -m)
        echo "Computer correctly set up for Microsoft Azure Kinect."
        ;;
    -o)
        echo "Computer correctly set up for ORBBEC Femto Cameras."
        ;;
    *)
        echo "Computer not correctly set up. Wrong argument!"
        ;;    
esac