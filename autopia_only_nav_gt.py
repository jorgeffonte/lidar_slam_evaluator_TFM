from geodesy import utm
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rosbag
import heapq
import sys
import math

def find_pairs_and_publish(navsatfix_msgs, navrelposned_msgs,pose_pub, path, time_tolerance=0.1):
    tolerance = rospy.Duration.from_sec(time_tolerance)
    bag = rosbag.Bag(path + 'autopia_gt.bag', 'w')

    fix_heap = []
    ned_heap = []

    gt_path = Path()
    gt_path.header.frame_id = '/map'
    origin = None

    for msg, t in navsatfix_msgs:
        heapq.heappush(fix_heap, (t, msg))

        fix_time, fix_msg = heapq.heappop(fix_heap)

        if origin is None:
            origin = utm.fromLatLong(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude)
            origin_easting = origin.easting
            origin_northing = origin.northing
            origin_altitude = fix_msg.altitude

        # Convert NavSatFix to UTM
        utm_point = utm.fromLatLong(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude)

        # Calculate relative position from origin
        dx = utm_point.easting - origin_easting
        dy = utm_point.northing - origin_northing
        dz = fix_msg.altitude - origin_altitude

        pose_msg = PoseStamped()
        pose_msg.header.stamp = fix_time
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = dx
        pose_msg.pose.position.y = dy
        pose_msg.pose.position.z = dz
        pose_msg.pose.orientation.w = 1  # Assuming no orientation information
        gt_path.header.stamp = pose_msg.header.stamp
        gt_path.poses.append(pose_msg)

        bag.write('ground_truth', gt_path, t=gt_path.header.stamp)
    bag.close()

def process_rosbag(path, name):
    bag = rosbag.Bag(path + name)
    pose_pub = rospy.Publisher('path', PoseStamped, queue_size=10)
    navsatfix_msgs = []
    navrelposned9_msgs = []
    for topic, msg, t in bag.read_messages(topics=['/ublox/fix','/ublox/navrelposned']):
        if topic == '/ublox/fix':
            navsatfix_msgs.append((msg, msg.header.stamp))
        else:
            navrelposned9_msgs.append((msg, msg.header.stamp))  
    find_pairs_and_publish(navsatfix_msgs,navrelposned9_msgs, pose_pub, path)
    bag.close()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: script.py <path_to_bag> <bag_filename>")
        sys.exit(1)
    process_rosbag(sys.argv[1], sys.argv[2])
