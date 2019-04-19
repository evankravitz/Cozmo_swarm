#!/bin/bash

#bash get_response_from_controller.sh [controller_ip_address] [robot id]

response=$(echo "cd Cozmo_swarm/Rover; cat fifo_$2_read" | sshpass -p "pi" ssh -o StrictHostKeyChecking=no pi@$1)
echo $response
