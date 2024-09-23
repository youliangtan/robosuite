#!/usr/bin/env python
"""
This uses env in https://github.com/ARISE-Initiative/robosuite
"""

import robosuite as suite
from robosuite.wrappers.gym_wrapper import GymWrapper
import cv2
import numpy as np
from robosuite.controllers import load_controller_config



if __name__ == '__main__':
    # mimicgen abs pose (ik)
    # mink
    # mimicgen OSC     


    controller_config = load_controller_config(default_controller="JOINT_TORQUE")
    # controller_config = load_controller_config(default_controller="JOINT_POSITION")
    # controller_config = load_controller_config(default_controller="OSC_POSE")
    #kp 
    # controller_config["kp"] = 5
    print("Controller Configuration:", controller_config)
    # controller_config["control_delta"] = False
    # create environment instance
    env = suite.make(
        # env_name="Lift",  # try with other tasks like "Stack" and "Door"
        # robots="Panda",   # try with other robots like "Sawyer" and "Jaco"
        env_name="TwoArmHandover",
        # robots="Baxter",
        robots="BiRobot",
        has_renderer=True,
        has_offscreen_renderer=True,
        use_camera_obs=True,
        control_freq=20,
        controller_configs=controller_config,
    )

    print("Environment Configuration:", env.observation_spec().keys())

    # convert to gym environment
    env = GymWrapper(
        env,
        keys=[
            "robot0_proprio-state", "object-state", "agentview_image",
              "robot0_left_eef_pos", "robot0_right_eef_pos", "robot0_joint_pos"
        ],
        flatten_obs=False, # default is True, we will not flatten the observation
    )
    print(" -> action space:", env.action_space)
    print(" -> observation space:", env.observation_space)
    # exit(0)
    obs, info = env.reset()

    action_dim = env.action_space.shape[0]
    action = np.zeros(action_dim)
    # action[:3] = np.array([1., 1., 1.])



    import time

    for i in range(1000):
        # action = np.random.randn(env.robots[0].dof)  # sample random action
        # action = env.action_space.sample()
        print("action:", action, action.shape)
        print("right eef pos:", obs["robot0_right_eef_pos"])
        print("left eef pos:", obs["robot0_left_eef_pos"])

        obs, reward, done, trunc, info = env.step(action)  # take action in the environment
        print(obs["robot0_proprio-state"].shape)

        # Optional camera view viz
        cv2.imshow("Agent View", cv2.cvtColor(obs["agentview_image"], cv2.COLOR_RGB2BGR))
        if cv2.waitKey(1) & 0xFF == ord('q'): break

        # time.sleep(0.5)
        env.render()  # render on display

    cv2.destroyAllWindows()
