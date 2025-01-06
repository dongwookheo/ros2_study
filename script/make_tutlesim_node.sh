#!/bin/bash

# Print the usage message
usage() {
    echo "Usage: $0 <domain_id> <namespace> <color>"
    echo "e.g.) $0 1 /mint r"
    echo "Available colors: r (red), g (green), b (blue), c (cyan), m (magenta), y (yellow)"
    exit 1
}

# Check the number of arguments
if [ "$#" -ne 3 ]; then
    usage
fi

# Check if the domain ID is a number
if ! [[ "$1" =~ ^[0-9]+$ ]]; then
    echo "Error: <domain_id> must be a number"
    usage
fi

# Check if the namespace starts with a slash
if ! [[ "$2" =~ ^/ ]]; then
    echo "Error: <namespace> must start with a slash"
    usage
fi

# Define color values
declare -A colors=(
    ["r"]="200 0 0"    # Red
    ["g"]="0 200 0"    # Green
    ["b"]="0 0 200"    # Blue
    ["c"]="0 200 200"  # Cyan
    ["m"]="200 0 200"  # Magenta
    ["y"]="200 200 0"  # Yellow
)

# Check if the color code is valid
if ! [[ -v colors[$3] ]]; then
    echo "Error: Invalid color code. Use r, g, b, c, m, or y"
    usage
fi

# Get RGB values for the selected color
read -r r g b <<< "${colors[$3]}"

# Set the domain ID and namespace
DOMAIN_ID=$1
NAMESPACE=$2
export ROS_DOMAIN_ID=$DOMAIN_ID

# Run turtlesim with background color parameters
ros2 run turtlesim turtlesim_node --ros-args \
    --remap __ns:=$NAMESPACE \
    -p background_r:=$r \
    -p background_g:=$g \
    -p background_b:=$b
