"""
Dexterous hands for GR1 robot.
"""
import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class BiRobotLeftHand(GripperModel):
    """
    Dexterous left hand of GR1 robot

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        import os
        mjcf_path = os.environ.get("BIROBOT_HANDS_MJCF_PATH")
        print("BIROBOT_HANDS_MJCF_PATH: ", mjcf_path)
        assert mjcf_path is not None, "Please export BIROBOT_HANDS_MJCF_PATH to the path of the BiRobot MJCF file"
        super().__init__(xml_path_completion(mjcf_path), idn=idn)

    def format_action(self, action):
        action[0] = np.pi / 2
        # return action[[0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5]]  # *0.5
        return np.array([np.pi / 2] * 7) # TODO

    @property
    def init_qpos(self):
        return np.array([0.0] * 7)

    @property
    def speed(self):
        return 0.15

    @property
    def dof(self):
        return 6 #12
    
    # @property
    # def _important_geoms(self):
    #     return {}
        # return {
        #     "left_finger": ["l_thumb_proximal_col", 
        #                     "l_thumb_proximal_2_col", 
        #                     "l_thumb_middle_col", 
        #                     "l_thumb_distal_col"],
        #     "right_finger": ["l_index_proximal_col", "l_index_distal_col",
        #                      "l_middle_proximal_col", "l_middle_distal_col",
        #                      "l_ring_proximal_col", "l_ring_distal_col",
        #                      "l_pinky_proximal_col", "l_pinky_distal_col"],
        #     "left_fingerpad": ["l_thumb_proximal_col", 
        #                     "l_thumb_proximal_2_col", 
        #                     "l_thumb_middle_col", 
        #                     "l_thumb_distal_col"],
        #     "right_fingerpad": ["l_index_proximal_col", "l_index_distal_col",
        #                      "l_middle_proximal_col", "l_middle_distal_col",
        #                      "l_ring_proximal_col", "l_ring_distal_col",
        #                      "l_pinky_proximal_col", "l_pinky_distal_col"],
        # }


class BiRobotRightHand(GripperModel):
    """
    Dexterous right hand of GR1 robot

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/birobot/right_floating_gr2.xml"), idn=idn)

    def format_action(self, action):
        action[0] = np.pi / 2
        # return action[[0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5]]  # *0.5
        return np.array([np.pi / 2] * 7) # TODO

    @property
    def init_qpos(self):
        return np.array([0.0] * 7)

    @property
    def speed(self):
        return 0.15

    @property
    def dof(self):
        return 6 #12
    
    # @property
    # def _important_geoms(self):
    #     return {}
