import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class BiRobot(ManipulatorModel):
    """
    BiRobot is a hunky bimanual robot designed by Rethink Robotics.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):
        self.select = 0
        # self.select = 1
        if self.select == 1:
            super().__init__(xml_path_completion("robots/gr1/gr1_upperbody.xml"), idn=idn)
        else:
            # get robot path from ENV export
            import os
            mjcf_path = os.environ.get("BIROBOT_MJCF_PATH")
            print("BIROBOT_MJCF_PATH: ", mjcf_path)
            assert mjcf_path is not None, "Please export BIROBOT_MJCF_PATH to the path of the BiRobot MJCF file"
            super().__init__(mjcf_path, idn=idn)

    @property
    def default_mount(self):
        return "RethinkMinimalMount"

    @property
    def default_gripper(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific gripper names
        """
        return {"right": "RethinkGripper", "left": "RethinkGripper"}
        # return {"right": "BiRobotRightHand", "left": "BiRobotLeftHand"}

    @property
    def default_controller_config(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific default controller config names (*filename)
        """
        return {"right": "default_birobot", "left": "default_birobot"}
    

    @property
    def init_qpos(self):
        """
        Since this is bimanual robot, returns [right, left] array corresponding to respective values

        Note that this is a pose such that the arms are half extended

        Returns:
            np.array: default initial qpos for the right, left arms
        """
        # [right, left]
        # Arms half extended
        # 1st is the head, next 7 are right arm, next 7 are left arm
        qpos = np.zeros(14)
        qpos[0] = 0.4
        qpos[7] = 0.4
        # # left arm
        # qpos[1] = -1.57
        # qpos[2] = -1.57

        # # right arm
        # qpos[7+1] = 1.57
        # qpos[7+2] = -0.57
        return qpos

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.29, 0, 0),
            "table": lambda table_length: (-0.26 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "bimanual"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        if self.select == 1:
            return {"right": "right_mount", "left": "left_mount"}
        else:
            return {"right": "r_gripper_base", "left": "l_gripper_base"}
