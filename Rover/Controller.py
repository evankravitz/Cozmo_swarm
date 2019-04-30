import os
import subprocess


class Controller:
	
	def __init__(self, num_cubes, block_placement_grid_width, num_cozmos):
		self.NUM_CUBES = num_cubes
		self.NUM_COZMOS = num_cozmos
		self.BLOCK_PLACEMENT_GRID_WIDTH = block_placement_grid_width
		self.cubes_for_pickup = [False for _ in range(self.NUM_CUBES)]
		self.cubes_for_placement = [False for _ in range(self.BLOCK_PLACEMENT_GRID_WIDTH)]
		self.setup()
		self.run()
		
	def setup(self):
		if not os.path.exists('fifo_controller_read'):
			os.system('mkfifo fifo_controller_read')

		for i in range(self.NUM_COZMOS):
			if not os.path.exists('fifo_' + str(i) + '_read'):
				os.system('mkfifo fifo_' + str(i) + '_read')



	def run(self):
		while True:
			output = subprocess.check_output(['cat', 'fifo_controller_read']).decode('UTF-8').split()
			robot_id = int(output[0])
			request_id = int(output[1])
			if request_id == 0: #cube pickup permission
				cube_num = int(output[2])
			elif request_id == 1 or request_id == 2: #cube placement permission or cube placement completion acknowledgement
				column_num = int(output[2])


			if request_id == 0: #cube pickup permission

				if not self.cubes_for_pickup[cube_num]:
					self.cubes_for_pickup[cube_num] = True
					os.system('echo 1 > fifo_' + str(robot_id) + '_read')
				else:
					os.system('echo 0 > fifo_' + str(robot_id) + '_read')

			elif request_id == 1:

				if not self.cubes_for_placement[column_num]:
					self.cubes_for_placement[column_num] = True
					os.system('echo 1 > fifo_' + str(robot_id) + '_read')
				else:
					os.system('echo 1 > fifo_' + str(robot_id) + '_read')

			elif request_id == 2:
				if self.cubes_for_placement[column_num]:
					self.cubes_for_placement[column_num] = False
				else:
					raise ValueError("Cozmo finished a column block placement that it did not get permission to do")


if __name__ == "__main__":
	Controller(3, 2, 2)
		
