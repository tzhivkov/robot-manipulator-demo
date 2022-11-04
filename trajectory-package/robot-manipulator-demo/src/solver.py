#!"C:\Users\Tsvetan\anaconda3\envs\workenv\python.exe"

import numpy as np

# This function will be turned into a class (it needs to be as fast as possible, consider using quaternions)
def forward_kinematics(link_lengths:list, joint_angles:list):
    """ FK function that takes link lengths and joint angles and returns all joints' (x,y,phi) cartesian coordinates and
        end-effectors' (x,y, phi) coordinates.

        Args:
            link_lengths:
            joint_angles:

        Returns:
            list: 
    """
    pass


def inverse_kinematics(link_lengths:list, target_coordinates:list, starting_joint_angles:list=None):
    """ IK function (based on FABRIK) that iteratively performs IK. 
        It is a best effort approach and therefore it might not be the most accurate method.

        Args:

        Returns:
    """
    pass