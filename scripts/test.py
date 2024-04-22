from rbs_gripper import RbsGrippperBuilder

robot_name = "arm0"

robot = RbsGrippperBuilder(robot_name, "world")
robot.base()
robot.ros2_control("gazebo")
robot.moveit()
urdf = robot.robot.urdf()
srdf = robot.robot.srdf()
print(srdf)

with open(f"rbs_{robot_name}.urdf", 'w') as xfile:
    xfile.write(urdf.urdf())
    xfile.close()
