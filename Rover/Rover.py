from enum import Enum
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes, ObservableElement, ObservableObject
from cozmo.util import Pose,distance_mm,degrees,speed_mmps,radians
import sys
import os
import subprocess
import math



class Mission(Enum):
	FLOOR_CUBE_PLACEMENT = 1
	DO_NOTHING = 2
	
class CurrentAction(Enum):
	NOT_SET = 0
	GET_CUBE = 1
	PLACE_CUBE = 2
	GO_TO_STAGING_AREA = 3


class Rover:
	
	def __init__(self, controller_ip, robot_id, block_placement_grid_width):
		
		self.CONTROLLER_IP = controller_ip
		self.ROBOT_ID = robot_id

		#How far away initial Cozmo placement area is from start of cube placement area
		self.CUBE_PLACEMENT_DISTANCE = 200 #mm

		self.BLOCK_PLACEMENT_GRID_WIDTH = block_placement_grid_width

		self.custom_boxes = []
		self.mission = Mission.FLOOR_CUBE_PLACEMENT
		self.action = CurrentAction.NOT_SET
		self.active = True
		self.robot = None
		self.custom_object_type_map = dict()



	def define_custom_boxes(self):
		custom_box_0 = self.robot.world.define_custom_box(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType00, \
												   marker_front=cozmo.objects.CustomObjectMarkers.Circles2, \
												   marker_back=cozmo.objects.CustomObjectMarkers.Circles3, \
												   marker_top=cozmo.objects.CustomObjectMarkers.Circles4, \
												   marker_bottom=cozmo.objects.CustomObjectMarkers.Circles5, \
												   marker_left=cozmo.objects.CustomObjectMarkers.Diamonds2, \
												   marker_right=cozmo.objects.CustomObjectMarkers.Diamonds3, \
												   depth_mm=60, \
												   width_mm=60, \
												   height_mm=45, \
												   marker_width_mm=17.86, \
												   marker_height_mm=17.86, \
												   is_unique=True)

		self.custom_boxes.append(custom_box_0)
		self.custom_object_type_map[custom_box_0.object_type] = 0

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
			while True:
				pass
				
	def place_cube(self):

		#move Cozmo to initial pose
		self.robot.go_to_pose(Pose(0, 0, 0, angle_z=degrees(0)), relative_to_robot=False, num_retries=0, in_parallel=False).wait_for_completed()


		self.robot.turn_in_place(degrees(90)).wait_for_completed()

		self.robot.drive_straight(distance_mm(self.CUBE_PLACEMENT_DISTANCE + 30), speed_mmps(50)).wait_for_completed()


		for column_num in range(self.BLOCK_PLACEMENT_GRID_WIDTH):
			self.robot.turn_in_place(degrees(-90)).wait_for_completed()
			cube_ids = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=2)
			if len(cube_ids) > 0:
				curr_min = float('inf')
				curr_min_idx = None
				for i in range(len(cube_ids)):
					if curr_min > abs(cube_ids[i].pose.position.x - self.robot.pose.position.x):
						curr_mind = abs(cube_ids[i].pose.position.x - self.robot.pose.position.x)
						curr_min_idx = i
				if self.can_dropoff_cube(column_num):
					self.dropoff_cube(cube_ids[curr_min_idx])
					return
				else:
					pass
			else:
				raise ValueError("Cannot find cube dropoff spot where there should be one")
			self.robot.turn_in_place(degrees(-90)).wait_for_completed()
			self.robot.drive_straight(distance_mm(60), speed_mmps(50)).wait_for_completed()


		raise ValueError("No space available to drop-off cube")





	def retrieve_cube(self):
		self.robot.set_lift_height(height = 0, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()


		#Move parallel to where cubes are initially placed, scan for cubes

		for i in range(30):

			self.robot.drive_straight(distance_mm(30), speed_mmps(50)).wait_for_completed()

			self.robot.turn_in_place(degrees(-90)).wait_for_completed()

			cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=2)

			if len(cube) == 0:
				self.robot.turn_in_place(degrees(-30)).wait_for_completed()
				cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=2)
				if len(cube) == 0:
					self.robot.turn_in_place(degrees(60)).wait_for_completed()
					cube = self.robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=2)
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

				






	def pickup_cube(self, cube):
		
		dist_tolerance = 110
		dist_to_move_into_cube = 30
    
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

			self.robot.go_to_pose(Pose(x, y, 0, angle_z = degrees(0)), relative_to_robot = False, num_retries = 0, in_parallel = False).wait_for_completed()

			#differece between cube and cozmo, to aligne them



			angle_to_turn = cube_z_angle - self.robot.pose.rotation.angle_z.radians # want -(difference angle)
			self.robot.turn_in_place(radians(angle_to_turn)).wait_for_completed()


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



		
		
	def dropoff_cube(self, cube_id):
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

			self.robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(0)), relative_to_robot=False, num_retries=0,
								  in_parallel=False).wait_for_completed()

			# differece between cube and cozmo, to aligne them



			angle_to_turn = cube_z_angle - self.robot.pose.rotation.angle_z.radians  # want -(difference angle)
			self.robot.turn_in_place(radians(angle_to_turn)).wait_for_completed()

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


if __name__ == "__main__":
	rover = Rover(controller_ip = "10.148.2.170", robot_id = 0, block_placement_grid_width = 2)
	cozmo.run_program(rover.run)
