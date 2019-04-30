#!/bin/bash

#bash get_permission_to_dropoff_cube.sh [controller_ip_address] [robot id] [column num]

echo "cd Cozmo_swarm/Rover; echo $2 1 $3 > fifo_controller_read" | sshpass -p "pi" ssh -o StrictHostKeyChecking=no pi@$1
