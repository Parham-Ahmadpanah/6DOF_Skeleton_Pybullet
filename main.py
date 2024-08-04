import pybullet_data
import pybullet as p
import time
import numpy as np
from gym import spaces, Env
import os
import inspect

from gait import *
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))


class EskeletonEnv(Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, renders=False):
        super(EskeletonEnv, self).__init__()

        self.max_x = 8
        self.max_y = 2
        self.max_z = 3

        self.renders = renders
        self.connected = False  # Track connection status
        self.goal_x = 5

        # Connect to PyBullet
        if self.renders and not self.connected:
            p.connect(p.GUI)
            self.connected = True
            print("Connected to PyBullet in GUI mode.")
        elif not self.renders and not self.connected:
            p.connect(p.DIRECT)
            self.connected = True
            print("Connected to PyBullet in DIRECT mode.")

        # Load plane and robot URDF only once
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = p.loadURDF("plane.urdf", [0, 0, 0])
        self.body = p.loadURDF("Eskeleton4/urdf/Eskeleton4.urdf", basePosition=[0, 0, 2])
        self.num_joints = p.getNumJoints(self.body)

        # Dynamically set the action and observation space
        self.action_space = self.get_action_space()
        self.observation_space = self.get_observation_space()

        p.setGravity(0, 0, -9.8)

        print("Environment initialized.")

    def get_action_space(self):
        action_low = []
        action_high = []
        
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.body, i)
            
            if joint_info[2] == p.JOINT_REVOLUTE:
                # Retrieve joint limits
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                
                # Append joint limits to action space
                action_low.append(lower_limit)
                action_high.append(upper_limit)
        
        action_low = np.array(action_low, dtype=np.float32)
        action_high = np.array(action_high, dtype=np.float32)
        return spaces.Box(low=action_low, high=action_high, dtype=np.float32)

    def get_observation_space(self):
        pi = np.pi 
        observation_low = [-self.max_x, -self.max_y, 0, -pi, -pi, -pi, -0.785, -1.5, 0, -1.75, 0, 0]
        observation_high = [self.max_x, self.max_y, self.max_z, pi, pi, pi, 1.75, 0, 1.4, 0.785, 1.5, 1.4]
        
        observation_low = np.array(observation_low)
        observation_high = np.array(observation_high)
        
        return spaces.Box(low=observation_low, high=observation_high, dtype=np.float32)

    def reset(self):
        # Reset the simulation state
        p.resetBasePositionAndOrientation(self.body, [0, 0, 2], [0, 0, 0, 1])
        self.state = self.compute_state()
        self.step_count = 0
        return self.state

    def compute_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.body)
        rpy = p.getEulerFromQuaternion(orn)

        joint_angles = [p.getJointState(self.body, i)[0] for i in range(self.num_joints)]
        state = np.array([*pos, *rpy, *joint_angles])
        return state

    def step(self, action):
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.body, i)
            
            if joint_info[2] == p.JOINT_REVOLUTE:
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                target_position = np.clip(action[i], lower_limit, upper_limit)
                p.setJointMotorControl2(
                    bodyUniqueId=self.body,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_position,
                    force=joint_info[10]
                )
            else:
                p.setJointMotorControl2(
                    bodyUniqueId=self.body,
                    jointIndex=i,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=action[i]
                )

        p.stepSimulation()
        self.state = self.compute_state()
        self.step_count += 1
        
        reward = self.get_reward()
        done = self.is_done()

        return self.state, reward, done, {}

    def render(self, mode='human'):
        if self.renders:
            width, height = 320, 240
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0, 2],
                distance=1.5,
                yaw=0,
                pitch=-30,
                roll=0,
                upAxisIndex=2
            )
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=60,
                aspect=float(width) / height,
                nearVal=0.1,
                farVal=100.0
            )
            p.getCameraImage(
                width, height, view_matrix, proj_matrix,
                renderer=p.ER_TINY_RENDERER
            )

    def close(self):
        if self.connected:
            p.disconnect()
            self.connected = False
            print("Disconnected from PyBullet.")

    def get_reward(self):
        height_reward = self.state[2]  # Reward for maintaining a certain height
        ex = abs(self.goal_x - self.state[0])
        reward = height_reward*2 - ex
        if ex < 0.1:
            reward = reward + 20
            if -0.4<self.state[1]<0.4:
                reward = reward +100
        return reward

    def is_done(self):
        ex = abs(self.goal_x - self.state[0])
        if self.state[2] < 0.6:  # If the robot falls below a certain height
            return True
        if ex <0.1 and -0.4<self.state[1]<0.4:
            return True
        return False


def gait(x_r):
    x_l=abs(50-x_r)
    j1r = angle_hip(x_r)
    j2r = -angle_knee(x_r)
    j3r = angle_ankle(x_r)
    j1l = -angle_hip(x_l)
    j2l = angle_knee(x_l)
    j3l = -angle_ankle(x_l)
    return [j1r, j2r, j3r, j1l, j2l, j3l]

# Simulation
env = EskeletonEnv(renders=True)
state = env.reset()
done = False
i=0
actionrr = [[0,0,90,-90,0,-90]]
actionrr = np.array(actionrr) * np.pi / 180

# Simulate for a few seconds
for _ in range(3240):  # 240 steps, assuming 60Hz simulation frequency, will simulate for 4 seconds
    # action = [0,0,0,0,0,0]  # Example action
    
    action = np.array(gait(i)) * np.pi / 180.0
    # action = actionrr[0]
    # if _  % 20 == 0:
    i=i+0.2
    if i == 100:
        i=0
    if i>100:
        i=0
    next_state, reward, done, _ = env.step(action)
    print(reward)
    if done:
        break

    time.sleep(1./250.)  # Sleep to match real-time simulation

env.close()

