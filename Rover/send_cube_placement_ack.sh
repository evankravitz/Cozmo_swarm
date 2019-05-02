#!/bin/bash

#bash send_cube_placement_ack.sh [controller_ip_address] [robot id] [column num]

echo "cd Cozmo_swarm/Rover; echo $2 2 $3 > fifo_controller_read" | sshpass -p "pi" ssh -o StrictHostKeyChecking=no pi@$1 
