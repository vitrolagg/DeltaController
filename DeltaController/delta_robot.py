from .kinematics import InverseKinematics
import numpy as np

class DeltaRobot:
    def __init__(self, base_radius, gripper_radius, active_arm, passive_arm):
        self.base_radius = base_radius
        self.gripper_radius = gripper_radius
        self.active_arm = active_arm
        self.passive_arm = passive_arm
        self.theta = [0, 120, -120]
        self.inverse_k = InverseKinematics(
            self.base_radius, self.gripper_radius, self.active_arm, self.passive_arm, theta=self.theta
        )

    def update_pose(self, xyz):
        if isinstance(xyz, list):
            xyz = np.array(xyz)
        phi = self.inverse_k.inverse(xyz)
        if phi is not None:
            return phi