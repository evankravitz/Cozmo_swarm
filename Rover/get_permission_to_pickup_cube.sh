#!/bin/bash

#bash get_permission_to_pickup_cube.sh [controller_ip_address] [robot id] [cube id]

echo "cd Cozmo_swarm/Rover; echo $2 $3 > fifo_controller_read" | sshpass -p "pi" ssh -o StrictHostKeyChecking=no pi@$1 
