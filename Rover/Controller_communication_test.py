import subprocess

import os




if __name__ == "__main__":
	os.system('bash get_permission_to_pickup_cube.sh 10.148.12.120 0 0')
	output = subprocess.check_output(['bash', 'get_response_from_controller.sh', '10.148.12.120', '0'])
	print(int(output[-2]))

