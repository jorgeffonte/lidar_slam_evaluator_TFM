import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import rosbag
import heapq
import sys

def find_pairs_and_publish(odom_msgs, pose_pub, path, time_tolerance=0.1):
    tolerance = rospy.Duration.from_sec(time_tolerance)
    bag = rosbag.Bag(path + 'utbm_gt_odom.bag', 'w')

    odom_heap = []

    gt_path = Path()
    gt_path.header.frame_id = '/map'
    origin = None

    for msg, t in odom_msgs:
        heapq.heappush(odom_heap, (t, msg))

        odom_time, odom_msg = heapq.heappop(odom_heap)

        if origin is None:
            origin = odom_msg.pose.pose.position
            origin_x = origin.x
            origin_y = origin.y
            origin_z = origin.z

        # Calculate relative position from origin
        dx = odom_msg.pose.pose.position.x - origin_x
        dy = odom_msg.pose.pose.position.y - origin_y
        dz = odom_msg.pose.pose.position.z - origin_z

        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom_time
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = dx
        pose_msg.pose.position.y = dy
        pose_msg.pose.position.z = dz
        pose_msg.pose.orientation = odom_msg.pose.pose.orientation
        gt_path.header.stamp = pose_msg.header.stamp
        gt_path.poses.append(pose_msg)

        bag.write('ground_truth', gt_path, t=gt_path.header.stamp)
    bag.close()

def process_rosbag(path, name):
    bag = rosbag.Bag(path + name)
    pose_pub = rospy.Publisher('path', PoseStamped, queue_size=10)
    odom_msgs = []
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if topic == '/odom':
            odom_msgs.append((msg, msg.header.stamp))
    find_pairs_and_publish(odom_msgs, pose_pub, path)
    bag.close()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: script.py <path_to_bag> <bag_filename>")
        sys.exit(1)
    process_rosbag(sys.argv[1], sys.argv[2])
