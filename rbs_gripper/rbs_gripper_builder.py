from robot_builder.components import JointNode
from robot_builder.robot_builder import RobotBuilderABC
from robot_builder.robot import LinkNode, RobotConfig
from odio_urdf import *

class RbsGrippperBuilder(RobotBuilderABC):
    def __init__(self,
                 robot_name:str, 
                 parent:str) -> None:
        self.robot_name = robot_name
        self.robot_type = "rbs_gripper"
        self.reset(robot_name, parent)

    def reset(self, robot_name, parent) -> None:
        self._rc = RobotConfig(robot_name=robot_name, robot_type=self.robot_type, parent=parent)

    @property
    def robot(self) -> RobotConfig:
        return self._rc

    def base(self) -> None:
        link0 = LinkNode(0, self.robot_name, 
                         self._rc.robot_config["base"], "base", self._rc.robot_package_abs_path)
        joint0 = JointNode(0, self._rc.parent, link0.name, joint_config={"type": "fixed"})
        self._rc.add(joint0)
        self._rc.add(link0)

        link1 = LinkNode(1, self.robot_name, self._rc.robot_config["rot"]["link"],
                         "rot", self._rc.robot_package_abs_path)
        self._rc.add(JointNode(1, link0.name, link1.name,
                               joint_config=self._rc.robot_config["rot"]["joint"]))
        self._rc.add(link1)
        link2 = LinkNode(2, self.robot_name, self._rc.robot_config["l_finger"]["link"],
                         "l_finger", self._rc.robot_package_abs_path)
        self._rc.add(link2)
        self._rc.add(JointNode(2, link1.name, link2.name, 
                               joint_config=self._rc.robot_config["l_finger"]["joint"]))
        link3 = LinkNode(3, self.robot_name, self._rc.robot_config["r_finger"]["link"],
                         "r_finger", self._rc.robot_package_abs_path)
        self._rc.add(link3)
        self._rc.add(JointNode(3, link1.name, link3.name,
                               joint_config=self._rc.robot_config["r_finger"]["joint"]))

        link_ee = LinkNode(None, self.robot_name,
                           link_config={},
                           link_name="grasp_point",
                           package_path=self._rc.robot_package_abs_path)
        self._rc.add(JointNode(None, link1.name, link_ee.name, joint_config={
            "type": "fixed",
            "origin": [0,0, 0.09139, 0, 0, 0]
        }))
        self._rc.add(link_ee)

        

    def ros2_control(self, hardware: str) -> None:
        self._rc.add_interface(hardware)
        self._rc.add_controller_manager(self.robot_name, config={
            'gripper_controller': True
        })
        self._rc.add_part("ros2_control")

    def gripper(self) -> None:
        raise NotImplementedError()

    def moveit(self) -> None:
        self._rc.add_srdf()
        self._rc.add_part("MoveIt2")
