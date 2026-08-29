#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET
from gazebo_msgs.srv import GetModelState
from urdf_parser_py.urdf import URDF

def get_model_pose(model_name):
    """Get the pose of the model from Gazebo"""
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state(model_name, 'world')

        # Extract position
        x, y, z = response.pose.position.x, response.pose.position.y, response.pose.position.z

        # Extract orientation (quaternion)
        qx, qy, qz, qw = (response.pose.orientation.x, response.pose.orientation.y,
                           response.pose.orientation.z, response.pose.orientation.w)

        print(f"📍 Model: {model_name}")
        print(f"🌍 Position (w.r.t world): x={x}, y={y}, z={z}")
        print(f"🔄 Orientation (quaternion): x={qx}, y={qy}, z={qz}, w={qw}")

        return x, y, z, qx, qy, qz, qw

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def get_model_dimensions(urdf_path, model_name):
    """Extracts the dimensions of the model from its URDF file"""
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Search for the first <box> geometry inside the <link> matching the model name
        for link in root.findall('link'):
            if model_name in link.attrib['name']:
                for visual in link.findall('visual'):
                    geometry = visual.find('geometry')
                    if geometry is not None:
                        box = geometry.find('box')
                        if box is not None:
                            size = box.attrib['size'].split()
                            length, breadth, height = float(size[0]), float(size[1]), float(size[2])
                            print(f"📏 Dimensions: Length={length}m, Breadth={breadth}m, Height={height}m")
                            return length, breadth, height

        print("⚠️ Could not find model dimensions in URDF.")
    except Exception as e:
        print(f"Error reading URDF: {e}")

if __name__ == "__main__":
    rospy.init_node('gazebo_model_info')

    # 🔹 Change the model_name and URDF file path accordingly
    model_name = "table"
    urdf_path = "/home/iitgn-robotics-1/Debojit_WS/Bi-Manual_Redundancy_Work/src/gazebo_assets/table.urdf"

    # Get pose and dimensions
    get_model_pose(model_name)
    get_model_dimensions(urdf_path, model_name)
