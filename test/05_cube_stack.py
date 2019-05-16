#!/usr/bin/env python3

# Copyright (c) 2016-2017 Anki, Inc.
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

'''Make Cozmo stack Cubes.

This script is meant to show off how easy it is to do high level robot actions.
Cozmo will wait until he sees two Cubes, and then will pick up one and place it on the other.
He will pick up the first one he sees, and place it on the second one.
'''

import cozmo
from cozmo import objects
from cozmo import world


# new_cube = objects.CustomObject
# new_cube.descriptive_name = "custom cube"
# new_cube.is_visible = True
# new_cube.is_unique = False
# new_cube.marker_height_mm = 24.892
# new_cube.marker_width_mm = 24.892
# new_cube.pickupable = True
# new_cube.place_objects_on_this = True
# new_cube.object_id = "CustomObject"
# new_cube.x_size_mm = 100
# new_cube.y_size_mm = 100
# new_cube.z_size_mm = 100



def cozmo_program(robot: cozmo.robot.Robot):
    custom_box = cozmo.robot.world.World.define_custom_box(cozmo.world.World, custom_object_type = cozmo.objects.CustomObjectTypes.CustomType00, \
                                marker_front = cozmo.objects.CustomObjectMarkers.Circles2, \
                                marker_back = cozmo.objects.CustomObjectMarkers.Circles3, \
                                marker_top = cozmo.objects.CustomObjectMarkers.Circles4, \
                                marker_bottom = cozmo.objects.CustomObjectMarkers.Circles5, \
                                marker_left = cozmo.objects.CustomObjectMarkers.Diamonds2, \
                                marker_right = cozmo.objects.CustomObjectMarkers.Diamonds3, \
                                depth_mm = 100, \
                                width_mm = 100, \
                                height_mm = 100, \
                                marker_width_mm = 24.892, \
                                marker_height_mm = 24.892, \
                                is_unique = True)
    custom_box.pickupable = True
    lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    custom_cube = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.CustomObjectTypes.CustomType00, timeout=60)
    lookaround.stop()
    if len(custom_cube) > 0:
        current_action = robot.pickup_object(custom_cube[0], num_retries=3)

    # Attempt to stack 2 cubes

    # Lookaround until Cozmo knows where at least 2 cubes are:
    lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
#     cubes = robot.world.wait_until_observe_num_objects(num=3, object_type=cozmo.objects.LightCube, timeout=60)
#     lookaround.stop()
#
#     if len(cubes) < 2:
#         print("Error: need 2 Cubes but only found", len(cubes), "Cube(s)")
#     else:
#         # Try and pickup the 1st cube
#         current_action = robot.pickup_object(cubes[0], num_retries=3)
#         current_action.wait_for_completed()
#         if current_action.has_failed:
#             code, reason = current_action.failure_reason
#             result = current_action.result
#             print("Pickup Cube failed: code=%s reason='%s' result=%s" % (code, reason, result))
#             return
#
#         # Now try to place that cube on the 2nd one
#         current_action = robot.place_on_object(cubes[1], num_retries=3)
#         current_action.wait_for_completed()
#         if current_action.has_failed:
#             code, reason = current_action.failure_reason
#             result = current_action.result
#             print("Place On Cube failed: code=%s reason='%s' result=%s" % (code, reason, result))
#             return
#
#         print("Cozmo successfully stacked 2 blocks!")
#
cozmo.run_program(cozmo_program)
