import rosbag
import tf
import numpy as np
import sys
import os

def quaternion_to_transformation_matrix(orientation, position):
    """Convert quaternion and position to a full 4x4 transformation matrix."""
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    translation = [position.x, position.y, position.z]
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
    transformation_matrix = np.zeros((4, 4))  # Now we use a 4x4 matrix
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    transformation_matrix[3, 3] = 1  # Add the extra row for homogeneous coordinates
    return transformation_matrix

def create_laser_to_camera_transform():
    """Create the 4x4 transformation matrix from laser to camera frame."""
    R = np.array([
        [7.027555e-03, -9.999753e-01, 2.599616e-05],
        [-2.254837e-03, -4.184312e-05, -9.999975e-01],
        [9.999728e-01, 7.027479e-03, -2.255075e-03]
    ])
    T = np.array([-7.137748e-03, -7.482656e-02, -3.336324e-01])
    transform = np.eye(4)
    transform[:3, :3] = R
    transform[:3, 3] = T
    return transform
def count_lines(filename):
    """Count the number of lines in a text file."""
    with open(filename, 'r') as file:
        line_count = 0
        for line in file:
            line_count += 1
    return line_count


def process_bag_file(seq,algorithm="kiss_icp"):
    bag_file = f"/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences/{str(seq).zfill(2)}/{algorithm}_path.bag"
    base_path = f"/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences/{str(seq).zfill(2)}/compare_ros"
    subdirs = [x[0] for x in os.walk(base_path)]
    subfolder = subdirs[1]  # Assuming there is only one subfolder
    bag_file = f"{subfolder}/{algorithm}_path.bag"
    out_path=f'/home/dronomy/TFM_ws/kitti_data/odom/dataset/results/{algorithm}/data/{str(seq).zfill(2)}.txt'
    gt_path=f'/home/dronomy/TFM_ws/kitti_data/odom/dataset/poses/{str(seq).zfill(2)}.txt'
    number_of_lines = count_lines(gt_path)
    """Process a ROS bag file to extract path transformations and save in KITTI format."""
    laser_to_camera_transform = create_laser_to_camera_transform()
    laser_to_camera_transform_inv = np.linalg.inv(laser_to_camera_transform)
    bag = rosbag.Bag(bag_file, 'r')
    i=0
    # Check if the directory of out_path exists, if not, create it
    out_dir = os.path.dirname(out_path)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    with open(out_path, 'w') as f:
        for _, msg, _ in bag.read_messages():
            poses = msg.poses
            
            
        for pose in poses:
            pose_stamped = pose
            pose_matrix = quaternion_to_transformation_matrix(
                    pose_stamped.pose.orientation, pose_stamped.pose.position)
            # Transform to camera coordinate system and re-reference
            

            # print(pose_matrix)
            # print("---------------")
            # Transform the pose to camera coordinate system
            pose_in_camera_frame = np.dot(laser_to_camera_transform,
                                            np.dot(pose_matrix,
                                                    laser_to_camera_transform_inv))            # Only use the top 3 rows and first 4 columns for KITTI format
            
            # print("Rounded Pose in Camera Frame:\n", np.array2string(np.round(pose_in_camera_frame, decimals=4), formatter={'float_kind':lambda x: "%.4f" % x}))
            # print("******************")
            formatted_matrix = ' '.join(f'{num:.9f}' for num in pose_in_camera_frame[:3].flatten())
            f.write(formatted_matrix + '\n')
            i+=1
            if (i>number_of_lines-1): 
                print("break")
                break
        while i< number_of_lines:
            f.write(formatted_matrix + '\n')
            print('while i< number_of_lines:',i)
            i+=1
    bag.close()

# Check if the correct number of arguments is provided
if len(sys.argv) < 2:
    print("Usage: python bag2kittiformat2.py  <algorithm>")
    print("Usage: python bag2kittiformat2.py  <algorithm> <sequence_number>")
    sys.exit(1)
elif len(sys.argv) > 3:
    print("Usage: python bag2kittiformat2.py  <algorithm>")
    print("Usage: python bag2kittiformat2.py  <algorithm> <sequence_number>")
    sys.exit(1)
    
    
    

# Get the sequence number and algorithm from the command line arguments
algorithm = sys.argv[1]

# Use the function
if len(sys.argv) == 2:
    for i in range(1,11):
        if i==3:
            continue
        process_bag_file(i, algorithm)
else:
    process_bag_file(int(sys.argv[2]), algorithm)
            

