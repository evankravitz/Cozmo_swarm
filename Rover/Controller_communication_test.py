import subprocess

import os




if __name__ == "__main__":
	os.system('bash get_permission_to_pickup_cube.sh 10.148.11.163 0 1')
	output = subprocess.check_output(['bash', 'get_response_from_controller.sh', '10.148.11.163', '0']).decode('UTF-8')
	print(int(output[-2]))

