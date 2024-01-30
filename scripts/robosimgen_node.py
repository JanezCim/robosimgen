#!/usr/bin/env python3

import rospy
from em import expand
from math import pi
import os
from rospkg import RosPack


# ros imports
import tf_conversions
import geometry_msgs
from dynamic_reconfigure.server import Server
from robosimgen.cfg import RobosimConfig
from visualization_msgs.msg import Marker

package_name='robosim'  # Can also be considered as robot name
output_path=f"{os.path.dirname(os.path.abspath(__file__))}/../.."
GLOBAL_MARKER_FRAME = 'base_link'


def dynamic_param_cb(config, level):
    body_mpub.publish(Marker(action=Marker.DELETE))
    body_mpub.publish(create_marker(Marker.CUBE,
                                          GLOBAL_MARKER_FRAME,
                                          px=config.body_pos_x, 
                                          py=config.body_pos_y,
                                          pz=config.body_pos_z,
                                          ox=0, oy=0, oz=0,
                                          sx=config.body_size_x, sy=config.body_size_y, sz=config.body_size_z,
                                          cr=1.0, cg=0, cb=0))
    # Wheels
    left_wheel_mpub.publish(Marker(action=Marker.DELETE))
    left_wheel_mpub.publish(create_marker(Marker.CYLINDER,
                                          GLOBAL_MARKER_FRAME,
                                          px=0, py=config.body_size_y/2+config.wheel_size_d/2, pz=0,  
                                          ox=pi/2, oy=0, oz=0, # Orient wheel upright automatically
                                          sx=config.wheel_size_r*2, sy=config.wheel_size_r*2, sz=config.wheel_size_d,
                                          cr=0, cg=1.0, cb=0))
    right_wheel_mpub.publish(Marker(action=Marker.DELETE))
    right_wheel_mpub.publish(create_marker(Marker.CYLINDER,
                                          GLOBAL_MARKER_FRAME,
                                          px=0, py=-config.body_size_y/2-config.wheel_size_d/2, pz=0, 
                                          ox=pi/2, oy=0, oz=0,  # Orient wheel upright automatically
                                          sx=config.wheel_size_r*2, sy=config.wheel_size_r*2, sz=config.wheel_size_d,
                                          cr=0, cg=1.0, cb=0))
    caster_wheel_mpub.publish(Marker(action=Marker.DELETE))
    caster_wheel_mpub.publish(create_marker(Marker.SPHERE,
                                            GLOBAL_MARKER_FRAME,
                                            px=config.caster_pos_x, py=config.caster_pos_y, pz=config.caster_pos_z, 
                                            ox=0, oy=0, oz=0,
                                            sx=config.caster_size_r*2, sy=config.caster_size_r*2, sz=config.caster_size_r*2,
                                            cr=0, cg=1.0, cb=0))
    if os.path.exists(output_path):
        rospy.logwarn(f"Path {output_path} already exist. Be sure no files from before are left there!")
    create_package(config)
    
    return config

def create_package(config):
    rp = RosPack()
    abs_in_templ_path = f"{os.path.dirname(os.path.abspath(__file__))}/../templates"  # Get path of templates
    for root, dirs, files in os.walk(abs_in_templ_path):  # Walk through all files in tempalte package
        for file in files:
            with open(os.path.join(root, file), 'r') as f:  # Open all subdir files in template package
                temp = f.read()
                try:
                    out = expand(temp, {'config': config})  # Import file content and emplace specified params
                    # TODO figure out how to also change package and file names to not be hardcoded
                    abs_out_pkg_path = f"{output_path}/{package_name}"  # Output package path
                    abs_out_file_path = f"{abs_out_pkg_path}/{os.path.relpath(root, abs_in_templ_path)}/{file}"
                    os.makedirs(os.path.dirname(abs_out_file_path), exist_ok=True)  # Create dirs to output file
                    with open(abs_out_file_path, 'w') as fi:
                        print(f"Creating {abs_out_file_path}")
                        fi.write(out)  # Create file and populate with template which has params emplaced
                except Exception as e:
                    rospy.loginfo(f"When trying to emplace to {os.path.join(root, file)} error: {e}.")

def create_marker(type, frame, px, py , pz, ox, oy, oz, sx, sy, sz, cr, cg, cb):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = type
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz

    # Set the color
    marker.color.r = cr
    marker.color.g = cg
    marker.color.b = cb
    marker.color.a = 0.8

    # Set the pose of the marker
    marker.pose.position.x = px
    marker.pose.position.y = py
    marker.pose.position.z = pz
    marker.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(ox, 
                                                                                                                 oy, 
                                                                                                                 oz))
    marker.lifetime = rospy.Duration(0)  # Display marker indefenetly
    return marker

if __name__ == "__main__":
    rospy.init_node("robosimgen", anonymous = False)
    body_mpub = rospy.Publisher("/robot_body_marker", Marker, queue_size = 2)
    left_wheel_mpub = rospy.Publisher("/robot_lwheel_marker", Marker, queue_size = 2)
    right_wheel_mpub = rospy.Publisher("/robot_rwheel_marker", Marker, queue_size = 2)
    caster_wheel_mpub = rospy.Publisher("/robot_cwheel_marker", Marker, queue_size = 2)
    srv = Server(RobosimConfig, dynamic_param_cb)
    rospy.spin()