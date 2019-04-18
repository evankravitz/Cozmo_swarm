import os
import subprocess


class Controller:
	
	def __init__(self, num_cubes):
		self.NUM_CUBES = num_cubes
		self.cubes = [False for _ in range(self.NUM_CUBES)]
		self.setup()
		self.run()
		
	def setup(self):
		os.system('mkfifo fifo_controller_read')
		
	def run(self):
		while True:
			output = subprocess.check_output(['cat', 'fifo_controller_read']).decode('UTF-8').split()
			print(int(output[0]))
			print(int(output[1]))
		#	robot_id = int(output[0])
	#		cube_num = int(output[1])
#			if not self.cubes[cube_num]:#
#				self.cubes[cube_num] = True
#				os.system('echo 1 > fifo_' + robot_id + '_read')
#			else:
#				os.system('echo 0 > fifo_' + robot_id + '_read')
	
		

if __name__ == "__main__":
	Controller(3)
		
