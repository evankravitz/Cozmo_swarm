from enum import Enum
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes, ObservableElement, ObservableObject
from cozmo.util import Pose,distance_mm,degrees,speed_mmps,radians
import sys
import os
import subprocess
import math
import time



class Mission(Enum):
	FLOOR_CUBE_PLACEMENT = 1
	DO_NOTHING = 2
	
class CurrentAction(Enum):
	NOT_SET = 0
	GET_CUBE = 1
	PLACE_CUBE = 2
	GO_TO_STAGING_AREA = 3


class Rover:
	
	def __init__(self, controller_ip, robot_id, block_placement_grid_width, robot_starting_position):
		
		self.CONTROLLER_IP = controller_ip
		self.ROBOT_ID = robot_id

		#How far away initial Cozmo placement area is from start of cube placement area
		self.CUBE_PLACEMENT_DISTANCE_R = 150 #mm
		self.CUBE_PLACEMENT_DISTANCE_L = 110 #mm

		self.wall_displacement_distance = 300


		self.BLOCK_PLACEMENT_GRID_WIDTH = block_placement_grid_width

		self.mission = Mission.FLOOR_CUBE_PLACEMENT
		self.action = CurrentAction.NOT_SET
		self.active = True
		self.robot = None

		self.robot_starting_position = robot_starting_position
		self.custom_object_type_map = dict()



	def define_custom_boxes(self):
		custom_box_0 = self.robot.world.define_custom_wall(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType00, marker=cozmo.objects.CustomObjectMarkers.Triangles4, width_mm=60, height_mm=45, marker_width_mm=23, marker_height_mm=23, is_unique=True)

		custom_box_1 = self.robot.world.define_custom_wall(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType01, marker=cozmo.objects.CustomObjectMarkers.Circles2, width_mm=60, height_mm=45, marker_width_mm=23, marker_height_mm=23, is_unique=True)
				   
		wall_0 = self.robot.world.define_custom_wall(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType02, marker=cozmo.objects.CustomObjectMarkers.Diamonds5, width_mm=60, height_mm=45, marker_width_mm=23, marker_height_mm=23, is_unique=True)

												   
		wall_1 = self.robot.world.define_custom_wall(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType03, marker=cozmo.objects.CustomObjectMarkers.Circles5, width_mm=60, height_mm=45, marker_width_mm=23, marker_height_mm=23, is_unique=True)

												   

		self.custom_object_type_map[custom_box_0.object_type] = 0
		self.custom_object_type_map[custom_box_1.object_type] = 1

	def run(self, robot: cozmo.robot.Robot):
		self.robot = robot
		self.define_custom_boxes()

        
		self.run_fsm_impl()
	
	def can_fetch_cube(self, cube_id):
		os.system('bash get_permission_to_pickup_cube.sh %s %d %d' % (self.CONTROLLER_IP, self.ROBOT_ID, cube_id))
		output = subprocess.check_output(['bash', 'get_response_from_controller.sh', self.CONTROLLER_IP, str(self.ROBOT_ID)]).decode('UTF-8')
		return int(output[-2]) == 1

	def can_dropoff_cube(self, column_num):
		os.system('bash get_permission_to_dropoff_cube.sh %s %d %d' % (self.CONTROLLER_IP, self.ROBOT_ID, column_num))
		output = subprocess.check_output(['bash', 'get_response_from_controller.sh', self.CONTROLLER_IP, str(self.ROBOT_ID)]).decode('UTF-8')
		return int(output[-2]) == 1

	def send_cube_dropoff_ack(self, column_num):
		os.system('bash send_cube_placement_ack.sh %s %d %d' % (self.CONTROLLER_IP, self.ROBOT_ID, column_num))
	
	def run_fsm_impl(self):
		if self.mission == Mission.FLOOR_CUBE_PLACEMENT:
			if self.action == CurrentAction.NOT_SET:
				self.action = CurrentAction.GET_CUBE
				self.retrieve_cube()
				self.run_fsm_impl()
			elif self.action == CurrentAction.GET_CUBE:
				self.action = CurrentAction.PLACE_CUBE
				self.place_cube()
				self.run_fsm_impl()
			elif self.action == CurrentAction.PLACE_CUBE:
				self.mission = Mission.DO_NOTHING
				self.run_fsm_impl()
		elif self.mission == Mission.DO_NOTHING:
			self.go_to_original_position()
			while True:
				pass
			
	def go_to_original_position(self):
		self.robot.go_to_pose(Pose(0, 0, 0, angle_z=degrees(0)), relative_to_robot=False, num_retries=0,
								  in_parallel=False).wait_for_completed()
			
				
	def place_cube(self):



		

		if self.robot_starting_position == "R":

			time.sleep(5)
		
			self.robot.go_to_pose(Pose(self.wall_displacement_distance, 0, 0, angle_z=degrees(90)), relative_to_robot=False, num_retries=0,
								  in_parallel=False).wait_for_completed()

			self.robot.drive_straight(distance_mm(self.CUBE_PLACEMENT_DISTANCE_R + 30), speed_mmps(50)).wait_for_completed()


			for column_num in range(self.BLOCK_PLACEMENT_GRID_WIDTH):
				self.robot.turn_in_place(degrees(-90)).wait_for_completed()
				if self.can_dropoff_cube(column_num):
					cube_ids = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=2)
					if len(cube_ids) > 0:
						print("Found dropoff spot")
						curr_min = float('inf')
						curr_min_idx = None
						for i in range(len(cube_ids)):
							print(cube_ids[i].pose)
							if curr_min > abs(cube_ids[i].pose.position.y - self.robot.pose.position.y):
								curr_min = abs(cube_ids[i].pose.position.y - self.robot.pose.position.y)
								curr_min_idx = i
						print("printing best cube: " + str(cube_ids[curr_min_idx].pose))
						return self.dropoff_cube(cube_ids[curr_min_idx], column_num)
					else:
						raise ValueError('Couldnt Find Cube')
				self.robot.turn_in_place(degrees(90)).wait_for_completed()
				self.robot.drive_straight(distance_mm(60), speed_mmps(50)).wait_for_completed()

			raise ValueError("No space available to drop-off cube")

		elif self.robot_starting_position == "L":

			self.robot.go_to_pose(Pose(self.wall_displacement_distance, 0, 0, angle_z=degrees(-90)), relative_to_robot=False, num_retries=0,
								  in_parallel=False).wait_for_completed()

			self.robot.drive_straight(distance_mm(self.CUBE_PLACEMENT_DISTANCE_L + 30),
									  speed_mmps(50)).wait_for_completed()

			for column_num in range(self.BLOCK_PLACEMENT_GRID_WIDTH - 1, -1, -1):
				self.robot.turn_in_place(degrees(90)).wait_for_completed()
				if self.can_dropoff_cube(column_num):
					cube_ids = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject,
																			   timeout=2)
					if len(cube_ids) > 0:
						print("Found dropoff spot")
						curr_min = float('inf')
						curr_min_idx = None
						for i in range(len(cube_ids)):
							print(cube_ids[i].pose)
							if curr_min > abs(cube_ids[i].pose.position.y - self.robot.pose.position.y):
								curr_min = abs(cube_ids[i].pose.position.y - self.robot.pose.position.y)
								curr_min_idx = i
						print("printing best cube: " + str(cube_ids[curr_min_idx].pose))
						return self.dropoff_cube(cube_ids[curr_min_idx], column_num)
					else:
						raise ValueError('Couldnt Find Cube')
				self.robot.turn_in_place(degrees(-90)).wait_for_completed()
				self.robot.drive_straight(distance_mm(60), speed_mmps(50)).wait_for_completed()

			raise ValueError("No space available to drop-off cube")







	def retrieve_cube(self):
		self.robot.set_lift_height(height = 0, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()
		
		step_size = 30

		#Move parallel to where cubes are initially placed, scan for cubes

		if self.robot_starting_position == "R":

			for i in range(30):

				self.robot.drive_straight(distance_mm(step_size), speed_mmps(50)).wait_for_completed()

				self.robot.turn_in_place(degrees(-90)).wait_for_completed()

				cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=0.5)

				if len(cube) == 0:
					self.robot.turn_in_place(degrees(-30)).wait_for_completed()
					cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=0.5)
					if len(cube) == 0:
						self.robot.turn_in_place(degrees(60)).wait_for_completed()
						cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=0.5)
						if len(cube) == 1:
							if self.can_fetch_cube(self.custom_object_type_map[cube[0].object_type]):
								self.pickup_cube(cube[0])
								return
							else:
								self.robot.turn_in_place(degrees(60)).wait_for_completed()
						else:
							self.robot.turn_in_place(degrees(60)).wait_for_completed()
					else:
						if self.can_fetch_cube(self.custom_object_type_map[cube[0].object_type]):
							self.pickup_cube(cube[0])
							return
						else:
							self.robot.turn_in_place(degrees(120)).wait_for_completed()

				else:
					if self.can_fetch_cube(self.custom_object_type_map[cube[0].object_type]):
						self.pickup_cube(cube[0])
						return
					else:
						self.robot.turn_in_place(degrees(90)).wait_for_completed()


				step_size *= 1.5

		elif self.robot_starting_position == "L":

			for i in range(30):

				self.robot.drive_straight(distance_mm(step_size), speed_mmps(50)).wait_for_completed()

				self.robot.turn_in_place(degrees(90)).wait_for_completed()

				cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=0.5)

				if len(cube) == 0:
					self.robot.turn_in_place(degrees(30)).wait_for_completed()
					cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=0.5)
					if len(cube) == 0:
						self.robot.turn_in_place(degrees(-60)).wait_for_completed()
						cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject,
																			   timeout=0.5)
						if len(cube) == 1:
							if self.can_fetch_cube(self.custom_object_type_map[cube[0].object_type]):
								self.pickup_cube(cube[0])
								return
							else:
								self.robot.turn_in_place(degrees(-60)).wait_for_completed()
						else:
							self.robot.turn_in_place(degrees(-60)).wait_for_completed()
					else:
						if self.can_fetch_cube(self.custom_object_type_map[cube[0].object_type]):
							self.pickup_cube(cube[0])
							return
						else:
							self.robot.turn_in_place(degrees(-120)).wait_for_completed()

				else:
					if self.can_fetch_cube(self.custom_object_type_map[cube[0].object_type]):
						self.pickup_cube(cube[0])
						return
					else:
						self.robot.turn_in_place(degrees(-90)).wait_for_completed()

				step_size *= 1.5




	def pickup_cube(self, cube):
		
		print('found cube')
		
		dist_tolerance = 110
		dist_to_move_into_cube = 50
    
		self.robot.set_lift_height(height = 0, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()

		cube_x = cube.pose.position.x
		cube_y = cube.pose.position.y
		cube_z_angle = cube.pose.rotation.angle_z.radians

		dx = self.robot.pose.position.x - cube_x
		dy = self.robot.pose.position.y - cube_y
		dist = (dx**2 + dy**2)**0.5


		while dist > dist_tolerance:

			a = math.tan(cube_z_angle)
			b = cube_y - cube_x*a
			# the second line y=dx is the line between pose and origine, perpendicular to

			d = -1/a
			x = b/(d - a)
			y = d*x

			self.robot.go_to_pose(Pose(x, y, 0, angle_z = radians(cube_z_angle)), relative_to_robot = False, num_retries = 0, in_parallel = False).wait_for_completed()

			#differece between cube and cozmo, to aligne them


			dx = self.robot.pose.position.x - cube_x
			dy = self.robot.pose.position.y - cube_y
			dist = (dx**2 + dy**2)**0.5

			dist_per_iteration = dist*0.65

			self.robot.drive_straight(distance_mm(dist_per_iteration), speed_mmps(50)).wait_for_completed()


			dx = self.robot.pose.position.x - cube_x
			dy = self.robot.pose.position.y - cube_y
			dist = (dx**2 + dy**2)**0.5


			if dist > dist_tolerance:
				look_around = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
				cubes = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout = 5)
				look_around.stop()


				if len(cubes) == 0:
					print("Lost cube.")
					return
				else:
					cube_x = cubes[0].pose.position.x
					cube_y = cubes[0].pose.position.y
					cube_z_angle = cubes[0].pose.rotation.angle_z.radians


		self.robot.drive_straight(distance_mm(dist_to_move_into_cube), speed_mmps(50)).wait_for_completed()
		self.robot.set_lift_height(height = 1, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()
		
		self.robot.turn_in_place(degrees(-180)).wait_for_completed()
		self.robot.drive_straight(distance_mm(50), speed_mmps(50)).wait_for_completed()



		
		
	def dropoff_cube(self, cube_id, column_num):
		dist_tolerance = 200
		dist_to_move_into_spot = 0


		cube_x = cube_id.pose.position.x
		cube_y = cube_id.pose.position.y
		cube_z_angle = cube_id.pose.rotation.angle_z.radians

		dx = self.robot.pose.position.x - cube_x
		dy = self.robot.pose.position.y - cube_y
		dist = (dx ** 2 + dy ** 2) ** 0.5

		while dist > dist_tolerance:

			a = math.tan(cube_z_angle)
			b = cube_y - cube_x * a
			# the second line y=dx is the line between pose and origine, perpendicular to

			d = -1 / a
			x = b / (d - a)
			y = d * x

			self.robot.go_to_pose(Pose(x, y, 0, angle_z=radians(cube_z_angle)), relative_to_robot=False, num_retries=0,
								  in_parallel=False).wait_for_completed()

			dx = self.robot.pose.position.x - cube_x
			dy = self.robot.pose.position.y - cube_y
			dist = (dx ** 2 + dy ** 2) ** 0.5

			dist_per_iteration = (dist - 120) * 1

			self.robot.drive_straight(distance_mm(dist_per_iteration), speed_mmps(50)).wait_for_completed()

			dx = self.robot.pose.position.x - cube_x
			dy = self.robot.pose.position.y - cube_y
			dist = (dx ** 2 + dy ** 2) ** 0.5

			if dist > dist_tolerance:
				look_around = self.robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
				cubes = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=5)
				look_around.stop()

				if len(cubes) == 0:
					print("Lost cube.")
					return
				else:
					cube_x = cubes[0].pose.position.x
					cube_y = cubes[0].pose.position.y
					cube_z_angle = cubes[0].pose.rotation.angle_z.radians

		self.robot.drive_straight(distance_mm(dist_to_move_into_spot), speed_mmps(50)).wait_for_completed()
		self.robot.set_lift_height(height=0, accel=6, max_speed=500, duration=1, in_parallel=False,
								   num_retries=3).wait_for_completed()
		self.send_cube_dropoff_ack(column_num)


if __name__ == "__main__":
	rover = Rover(controller_ip = "10.148.2.133", robot_id = 0, block_placement_grid_width = 2, robot_starting_position = 'R')
	cozmo.run_program(rover.run)
