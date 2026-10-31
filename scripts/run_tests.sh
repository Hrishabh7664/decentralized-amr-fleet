#!/bin/bash
# run_tests.sh - Run unit tests and colcon tests across the AMR fleet stack
set -e

echo "=========================================================="
echo " Running Decentralized AMR Fleet Test Suite"
echo "=========================================================="

echo "[1/2] Running Python unit tests via pytest..."
python3 -m pytest tests/ -v

if command -v colcon &> /dev/null; then
    echo "[2/2] Running colcon workspace tests..."
    colcon test --packages-select amr_fleet amr_description amr_worlds
    colcon test-result --verbose
else
    echo "[2/2] colcon not detected on host; running colcon test in simulation container if active..."
    if docker compose ps | grep -q "amr_fleet_simulation"; then
        docker compose exec simulation bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && colcon test --packages-select amr_fleet amr_description amr_worlds && colcon test-result --verbose"
    fi
fi

echo "All tests passed successfully!"
