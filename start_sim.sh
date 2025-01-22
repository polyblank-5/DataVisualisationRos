#!/bin/bash

# Configuration: Define virtual environments, ROS setup, and programs
VENV_PATHS=(
    "~/ros2_ws/src/data_visualisation_pkg/.venv"
    "~/ros2_ws/src/data_collection_pkg/.venv"
    "~/ros2_ws/src/laser_control_pkg/.venv"
)

ROS_SETUP="/opt/ros/jazzy/setup.bash"

PROGRAMS_COLLECTION=(
    "python3 src/data_collection_pkg/data_collection_pkg/plant_position_publisher.py"  # Program for VENV1
    "python3 src/data_collection_pkg/data_collection_pkg/plant_velocity_publisher.py"  # Program for VENV2
    "python3 src/data_collection_pkg/data_collection_pkg/plant_data_subscriber.py"  # Program for VENV3
)
PROGRAM_VISUALIZER="python3 src/data_visualisation_pkg/data_visualisation_pkg/plant_data_visualizer.py"
PROGRAMS_CONTROL=(
    "python3 src/laser_control_pkg/laser_control_pkg/laser_position_publisher.py"
    "python3 src/laser_control_pkg/laser_control_pkg/laser_measured_publisher.py"
)

# Function to open a terminal, source venv, ROS, and start a program
start_terminal() {
    local venv_path=$1
    local program=$2
    gnome-terminal -- bash -c "
        echo 'Sourcing ROS: $ROS_SETUP';
        source $ROS_SETUP;
        echo 'Activating Virtual Environment: $venv_path';
        source $venv_path/bin/activate;
        echo 'Starting Program: $program';
        $program;
        exec bash"
}

# Change to the desired directory
cd ../.. || exit

# Commands to execute
echo "Executing commands in $(pwd)"

# Step 2: Open a new terminal, source VENV1, and start PROGRAM1 again
start_terminal "${VENV_PATHS[0]}" "${PROGRAM_VISUALIZER}"

# Step 1: Open 3 terminals, each with a different venv and program
for i in {0..2}; do
    start_terminal "${VENV_PATHS[1]}" "${PROGRAMS_COLLECTION[i]}"
done



# Step 3: Open 2 new terminals, each with a different venv and program
start_terminal "${VENV_PATHS[2]}" "${PROGRAMS_CONTROL[0]}"
start_terminal "${VENV_PATHS[2]}" "${PROGRAMS_CONTROL[1]}"

echo "All terminals started."
