import numpy as np
import matplotlib.pyplot as plt
import sys

# This function will be turned into a class (it needs to be as fast as possible, it will use dual quaternion calculations)
def fk_dual_quat(link_lengths:list, joint_angles:list):
    """ FK function that takes link lengths and joint angles and returns all joints' (x,y,phi) cartesian coordinates and
        end-effectors' (x,y, phi) coordinates. The FK function will perform dual quaternion operation to calculate coordinates.

        Args:
            link_lengths:
            joint_angles:

        Returns:
            list: 
    """
    pass

def fk_matrices(link_lengths:list, joint_angles:list):
    """ FK function that calculates forward kinematics for a 5 joint planar arm using Denavit Hartenberg (DH) matrices. The fuction 
        'can' return (x,y,phi) of a particular joint node, however it will only return a list of the final joint angles. This 
        function will be much slower than the 'fk_dual_quat' method. It will mainly be used for testing and debugging this project 
        and also as a legacy method of calculating forward kinematics.

        Args:
            link_lengths:
            joint_angles:

        Returns:
            list:
    """
    # if the list length of links and joints isn't 5, then exit function
    if not (len(link_lengths) == 5) or not (len(joint_angles) == 5):
        print('The number of links or joints isn\'t correct. Please input 5 links and joints.')
        return []
    # make sure all link lengths are of type float 
    for link in link_lengths:
        if isinstance(link, int):
            link = float(link)
        if not isinstance(link, float):
            return []
    # check link lengths are reasonable sizes
    max_link_length, min_link_length = max(link_lengths), min(link_lengths)
    if (max_link_length > 29) or (min_link_length < 0.1):
        print(f'A link is considered too long if it is over 29m long or too short if it is less than 0.1m. \nPlease try again.')
        return []
    # populating the DH table (format is [alpha, a, offset dist, theta], where <alpha> is twist angle, <a> link length, <offset dist> is distance, 
    # <theta> is joint angle) 
    dh_table = np.array([[0.0,link_lengths[0],0.0,joint_angles[0]],
                        [0.0,link_lengths[1],0.0,joint_angles[1]],
                        [0.0,link_lengths[2],0.0,joint_angles[2]],
                        [0.0,link_lengths[3],0.0,joint_angles[3]],
                        [0.0,link_lengths[4],0.0,joint_angles[4]]])
    array_list = []
    # create a list of arrays of homogeneous transformation matrices
    for row in dh_table:
        array_list.append(np.array([[np.cos(row[3]),-np.sin(row[3]),0,row[1]*np.cos(row[3])],
                                    [np.sin(row[3]),np.cos(row[3]),0,row[1]*np.sin(row[3])],
                                    [0.0,0.0,1.0,0.0],
                                    [0.0,0.0,0.0,1.0]]))
    final_coordinates = []
    # get each coordinate from frame 0 to frame 'i'
    htransform = previous_htransform = None
    for i in range(0,len(array_list)):
        if i == 0:
            htransform = array_list[i]
        else:
            htransform = np.matmul(previous_htransform, array_list[i])
        final_coordinates.append([float(htransform[[0],[3]]),float(htransform[[1],[3]])])
        previous_htransform = htransform
        
    # final homogeneous transform matrix (used @ operator out of convenience instead of numpy's matmul)
    # final_htransform = array_list[0] @ array_list[1] @ array_list[2] @ array_list[3] @ array_list[4]
    # final_pose = [float(final_htransform[[0],[3]]),float(final_htransform[[1],[3]]),np.arccos(float(final_htransform[[0],[0]]))]
    return final_coordinates


def inverse_kinematics(link_lengths:list, target_coordinates:list, starting_joint_angles:list=None):
    """ IK function (based on FABRIK) that iteratively performs IK. 
        It is a best effort approach and therefore it might not be the most accurate method.

        Args:
            link_lengths:


        Returns:
    """
    pass

def plot_arm(coordinates):
    plt.plot(coordinates[0], coordinates[1], color='b')
    plt.scatter(coordinates[0], coordinates[1], alpha=1, color='r')
    plt.tight_layout
    plt.show()


def main():
    link_lengths = [0.5,0.4,0.3,0.2,0.1]
    joint_angles = [0.261,0.261,0.261,0.261,0.261]
    # fk_coordinates = (fk_matrices([0.5,0.4,0.3,0.2,0.1],[0.261,0.261,0.261,0.261,0.261]))
    fk_final = (fk_matrices([0.5,0.4,0.3,0.2,0.1],[0.261,0.261,0.561,1.261,0.261]))
    print(fk_final[-1])
    x_coords = [0.0]
    x_coords.extend([x[0] for x in fk_final])
    y_coords = [0.0]
    y_coords.extend([y[1] for y in fk_final])
    print(x_coords)
    print(y_coords)
    plot_arm([x_coords,y_coords])

if __name__ == '__main__':
    main()
