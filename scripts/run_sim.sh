#!/bin/bash
# run_sim.sh - Start the decentralized AMR fleet simulation and web dashboard
set -e

SCENARIO="${1:-3}"

echo "=========================================================="
echo " Starting Decentralized AMR Fleet Simulation (${SCENARIO} AMRs)"
echo "=========================================================="

if [ "$SCENARIO" -eq 5 ]; then
    echo "Launching 5-robot high-density scenario..."
    docker compose up -d simulation rosbridge dashboard
    docker compose exec simulation bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch amr_fleet fleet_5.launch.py gui:=false"
else
    echo "Launching standard 3-robot chokepoint scenario..."
    docker compose up -d simulation rosbridge dashboard
fi

echo "Stack is running!"
echo "Dashboard UI: http://localhost:3000"
echo "Rosbridge WebSocket: ws://localhost:9090"
