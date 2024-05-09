import rosbag
import tf.transformations
import numpy as np

def quaternion_to_matrix(quaternion):
    """Convert a quaternion to a full 4x4 transformation matrix."""
    return tf.transformations.quaternion_matrix(quaternion)

def read_rosbag(bag_file, topic):
    """Reads a ROS bag file and extracts pose data from the specified topic."""
    bag = rosbag.Bag(bag_file, 'r')
    poses = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        for pose_stamped in msg.poses:
            position = pose_stamped.pose.position
            orientation = pose_stamped.pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            matrix = quaternion_to_matrix(quaternion)
            matrix[0:3, 3] = [position.x, position.y, position.z]  # Set the translation part
            poses.append(matrix)
    bag.close()
    return poses

def compute_transforms(poses):
    """Compute transformations relative to the first frame."""
    base_pose_inv = np.linalg.inv(poses[0])
    transformations = [base_pose_inv @ pose for pose in poses]
    return transformations

def save_to_kitti_format(transformations, output_file):
    """Save the transformation matrices to a file in KITTI format."""
    with open(output_file, 'w') as f:
        for trans in transformations:
            # Only use the top 3 rows and first 4 columns for KITTI format
            for row in trans[:3]:
                f.write(' '.join(f'{v:.9f}' for v in row) + '\n')

def main(bag_file, topic, output_file):
    poses = read_rosbag(bag_file, topic)
    transformations = compute_transforms(poses)
    save_to_kitti_format(transformations, output_file)

if __name__ == '__main__':
    # Set the path to your ROS bag file, the relevant topic, and the desired output file.
    bag_file = '/home/dronomy/Downloads/kiss_icp_path.bag'
    topic = 'kiss_icp_path'
    output_file = 'output_kitti_format.txt'
    main(bag_file, topic, output_file)
