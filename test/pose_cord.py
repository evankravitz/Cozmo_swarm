#!/usr/bin/env python3

# Copyright (c) 2017 Anki, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''This example demonstrates how you can define custom objects.

The example defines several custom objects (2 cubes, a wall and a box). When
Cozmo sees the markers for those objects he will report that he observed an
object of that size and shape there.

You can adjust the markers, marker sizes, and object sizes to fit whatever
object you have and the exact size of the markers that you print out.
'''

import time,math,cozmo
from math import sqrt,atan,cos,sin,tan
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes, ObservableElement, ObservableObject
from cozmo.util import Pose,distance_mm,degrees,speed_mmps,radians
import sys
print(sys.path)


def pickupCube(robot):
    
    dist_tolerance = 40
    dist_to_move_into_cube = 90
    
    robot.set_lift_height(height = 0, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=20)
    look_around.stop()
    find=False
    if len(cubes) > 0:
        
        dx = robot.pose.position.x - cubes[0].pose.position.x
        dy = robot.pose.position.y - cubes[0].pose.position.y
        dist = (dx**2 + dy**2)**0.5

        while find==False:
            cubex=cubes[0].pose.position.x
            cubey=cubes[0].pose.position.y
            dis=sqrt(cubex*cubex+cubey*cubey)
            cube_z=cubes[0].pose.rotation.angle_z.radians #pos if pos gradient, neg if neg gradient in x/y plane 
            #look for x=ay+b  a=cubex/cubey=1/tan(cubez) tan(cubez)=y/x
            a=1/tan(cube_z)
            b=cubex-cubey*a
            # the second line x=dy is the line between pose and origine, perpendicular to 
            
            d=-1/(a)
            y=b/(d-a)
            x=d*y
            #print('cubes[0].pose :')
            #print(cubes[0].pose)
            #print('robot.pose1 :')
            #print(robot.pose)
            #print('x:',x, ' y:',y,' cube_z:',cube_z,' a:',a, ' b:',b,' d:',d)
            
            
            robot.go_to_pose(Pose(x,y,0,angle_z=degrees(0)),relative_to_robot=False,num_retries=0,in_parallel=False).wait_for_completed() 

            
            #differece between cube and cozmo, to aligne them 
            
            print('here')

            angle1=cubes[0].pose.rotation.angle_z.radians-robot.pose.rotation.angle_z.radians # want -(difference angle)
            robot.turn_in_place(radians(angle1)).wait_for_completed()
            #print('different rad: ',angle1, ' radians: ',radians(angle1))
            #print('robot.pose : after')
            #print(robot.pose)

            dx = (x - robot.pose.position.x)
            dy = (y - robot.pose.position.y)
            diff=math.sqrt(dx**2+dy**2)
            
            print('diff: ',diff)
            #print('going to smaller')
            ang=cubes[0].pose.rotation.angle_z.radians-robot.pose.rotation.angle_z.radians
            print('diff angle: ',ang)

            if (diff<10 and ang<0.08):
                print('smaller than tolerance')
                dx = (robot.pose.position.x - cubes[0].pose.position.x)
                dy = (robot.pose.position.y - cubes[0].pose.position.y)
                dist = (dx**2 + dy**2)**0.5
                robot.drive_straight(distance_mm(dist),speed_mmps(50)).wait_for_completed()
                find=True
            else:
                print('bigger than tolerance')
                cubes=[]
                while(len(cubes)==0):
                    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
                    cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=CustomObject, timeout=5)
                    look_around.stop()
                    if(len(cubes)==0):
                        robot.drive_straight(distance_mm(-50),speed_mmps(50)).wait_for_completed()
                        print('not find')
                robot.drive_straight(distance_mm(dist*0.5),speed_mmps(50)).wait_for_completed()

            
            
           
            
        robot.drive_straight(distance_mm(dist_to_move_into_cube),speed_mmps(50)).wait_for_completed()
        robot.set_lift_height(height = 1, accel = 6, max_speed = 500, duration = 1, in_parallel = False, num_retries = 3).wait_for_completed()

        
    else:
        print("Cannot locate custom box")
    

    
    

        

    
    


def handle_object_appeared(evt, **kw):
    # This will be called whenever an EvtObjectAppeared is dispatched -
    # whenever an Object comes into view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo started seeing a %s" % str(evt.obj.object_type))


def handle_object_disappeared(evt, **kw):
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))


        

def custom_objects(robot: cozmo.robot.Robot):
    # Add event handlers for whenever Cozmo sees a new object
    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)





    custom_box = robot.world.define_custom_box(custom_object_type=cozmo.objects.CustomObjectTypes.CustomType00, \
                                                       marker_back=cozmo.objects.CustomObjectMarkers.Circles3, \
                                                       marker_front=cozmo.objects.CustomObjectMarkers.Circles2, \
                                                       marker_top=cozmo.objects.CustomObjectMarkers.Circles4, \
                                                       marker_bottom=cozmo.objects.CustomObjectMarkers.Circles5, \
                                                       marker_left=cozmo.objects.CustomObjectMarkers.Diamonds2, \
                                                       marker_right=cozmo.objects.CustomObjectMarkers.Diamonds3, \
                                                       depth_mm=60, \
                                                       width_mm=60, \
                                                       height_mm=45, \
                                                       marker_width_mm=17, \
                                                       marker_height_mm=17, \
                                                       is_unique=True)
    
    if (custom_box is not None):
        print("All objects defined successfully!")
    else:
        print("One or more object definitions failed!")
        return

    #print("Show the above markers to Cozmo and you will see the related objects "
    #      "annotated in Cozmo's view window, you will also see print messages "
    #      "everytime a custom object enters or exits Cozmo's view.")

    #print("Press CTRL-C to quit")

    #print('robot_initial: ',robot.pose.rotation.angle_z.degrees)
    #cubes = lookForCubes(robot, 45, 0)
    
    #print('robot.pose.position_beforelook: \n')
    #print(robot.pose)
    print('robot.pose1 :')
    print(robot.pose)
    pickupCube(robot)
    


    while True:
        time.sleep(0.1)


cozmo.run_program(custom_objects)
