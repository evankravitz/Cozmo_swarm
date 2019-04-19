from enum import Enum
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes, ObservableElement, ObservableObject
from cozmo.util import Pose
import sys
import os
import subprocess




class Mission(Enum):
	PLACE_CUBE_FLOOR = 1
	DO_NOTHING = 2
	
class CurrentAction(Enum):
	NOT_SET = 0
	GET_CUBE_FLOOR = 1
	PLACE_CUBE_FLOOR = 2
	PLACE_CUBE_CEILING = 3


class Rover:
	
	def __init__(self):
		
		self.CONTROLLER_IP = ""
		self.ROBOT_ID = 0
		
		
		self.mission = Mission.PLACE_CUBE_FLOOR
		self.active = True
		self.robot = None
		

	
	def run(self, robot: cozmo.robot.Robot):
		self.robot = robot
		
		custom_box_1 = self.robot.world.define_custom_box(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType00, \
                                                       marker_front=cozmo.objects.CustomObjectMarkers.Circles2, \
                                                       marker_back=cozmo.objects.CustomObjectMarkers.Circles3, \
                                                       marker_top=cozmo.objects.CustomObjectMarkers.Circles4, \
                                                       marker_bottom=cozmo.objects.CustomObjectMarkers.Circles5, \
                                                       marker_left=cozmo.objects.CustomObjectMarkers.Diamonds2, \
                                                       marker_right=cozmo.objects.CustomObjectMarkers.Diamonds3, \
                                                       depth_mm=60, \
                                                       width_mm=60, \
                                                       height_mm=45, \
                                                       marker_width_mm=24.892, \
                                                       marker_height_mm=24.892, \
                                                       is_unique=True)
                                                       
        
		self.run_fsm_impl()
	
	def can_fetch_cube(cube_id):
		os.system('bash get_permission_to_pickup_cube.sh %s %d %d' % (self.CONTROLLER_IP, self.ROBOT_ID, cube_id))
		output = subprocess.check_output(['bash', 'get_response_from_controller.sh', self.CONTROLLER_IP, str(self.ROBOT_ID)])
		if int(output[-2]) == 1:
			return True
		return False
		
	
	def run_fsm_impl(self):
		if self.mission == Mission.PLACE_CUBE_FLOOR:
			if self.action == CurrentAction.NOT_SET:
				self.action = CurrentAction.GET_CUBE_FLOOR
				self.pickup_cube()
				self.run_fsm_impl()
			elif self.action == CurrentAction.GET_CUBE_FLOOR:
				self.action = CurrentAction.PLACE_CUBE_FLOOR
				self.place_cube_floor()
				self.run_fsm_impl()
			elif self.action == CurrentAction.PLACE_CUBE_FLOOR:
				self.mission = Mission.DO_NOTHING
			
		elif self.mission == Mission.DO_NOTHING:
			pass
				
			
	
	def pickup_cube(self):
		
		dist_tolerance = 110
		dist_to_move_into_cube = 90
    
		robot.set_lift_height(height = 0, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()
		look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
		cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=30)
		look_around.stop()
    
		if len(cubes) > 0:
        
			dx = robot.pose.position.x - cubes[0].pose.position.x
			dy = robot.pose.position.y - cubes[0].pose.position.y
			dist = (dx**2 + dy**2)**0.5
        
        
			while dist > dist_tolerance:
				cubex = cubes[0].pose.position.x
				cubey = cubes[0].pose.position.y
				dis =  sqrt(cubex*cubex + cubey*cubey)
				cube_z = cubes[0].pose.rotation.angle_z.radians #pos if pos gradient, neg if neg gradient in x/y plane 
				#look for y=ax+b atan(cube_z)=gradient; b is intersection on y
            
				a = tan(cube_z)
				b = cubey-cubex*a
				# the second line y=dx is the line between pose and origine, perpendicular to 
            
				d=-1/(a)
				x=b/(d-a)
				y=d*x
            
				robot.go_to_pose(Pose(x,y,0,angle_z=degrees(0)),relative_to_robot=False,num_retries=0,in_parallel=False).wait_for_completed() 
            
				#differece between cube and cozmo, to aligne them 
            
            

				angle1=cubes[0].pose.rotation.angle_z.radians-robot.pose.rotation.angle_z.radians # want -(difference angle)
				robot.turn_in_place(radians(angle1)).wait_for_completed()
            
            
				dx = (robot.pose.position.x - cubes[0].pose.position.x)
				dy = (robot.pose.position.y - cubes[0].pose.position.y)
				dist = (dx**2 + dy**2)**0.5
            
				dist_per_iteration = dist*0.75
                        
				robot.drive_straight(distance_mm(dist_per_iteration),speed_mmps(50)).wait_for_completed()
            
            
				dx = (robot.pose.position.x - cubes[0].pose.position.x)
				dy = (robot.pose.position.y - cubes[0].pose.position.y)
				dist = (dx**2 + dy**2)**0.5
                      
            
				if dist > dist_tolerance:
					look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
					cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=10)
					look_around.stop()


					if len(cubes) == 0:
						print("Lost cube.")
						return
						
						
			robot.drive_straight(distance_mm(dist_to_move_into_cube),speed_mmps(50)).wait_for_completed()
			robot.set_lift_height(height = 1, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()

        
    else:
        print("Cannot locate custom box")
		
		
	def place_cube_floor(self):
		pass


if __name__ == "__main__":
	rover = Rover(cozmo)
	cozmo.run_program(rover.run)
