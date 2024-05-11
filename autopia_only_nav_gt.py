import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from geopy.distance import geodesic
import heapq
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import rosbag
from tf.transformations import quaternion_from_euler
import math
import sys
from geographiclib.geodesic import Geodesic
import heapq
from geometry_msgs.msg import PoseStamped
geod = Geodesic.WGS84
def find_pairs_and_publish(navsatfix_msgs, pose_pub, path, time_tolerance=0.1):
    tolerance = rospy.Duration.from_sec(time_tolerance)
    bag = rosbag.Bag(path + 'autopia_gt.bag', 'w')  # Open the bag file in write mode

    fix_heap = []
    gt_path = Path()
    gt_path.header.frame_id = '/map'
    
    # Variable para almacenar la posición inicial
    origin = None

    for msg, t in navsatfix_msgs:
        heapq.heappush(fix_heap, (t, msg))

    while fix_heap:
        fix_time, fix_msg = heapq.heappop(fix_heap)

        if origin is None:
            # Set the first received position as the origin
            origin = (fix_msg.latitude, fix_msg.longitude)
            origin_altitude = fix_msg.altitude

        # Calculate the relative position using geographiclib
        current_position = (fix_msg.latitude, fix_msg.longitude)
        inv = geod.Inverse(origin[0], origin[1], current_position[0], current_position[1])
        distance = inv['s12']
        bearing = inv['azi1']  # Initial bearing from origin to current_position

        dx = distance * math.sin(math.radians(bearing))
        dy = distance * math.cos(math.radians(bearing))
        dz = fix_msg.altitude - origin_altitude

        pose_msg = PoseStamped()
        pose_msg.header.stamp = fix_time
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = dx
        pose_msg.pose.position.y = dy
        pose_msg.pose.position.z = dz
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 1
        gt_path.header.stamp = pose_msg.header.stamp
        gt_path.poses.append(pose_msg)
        
        bag.write('pose_gt', pose_msg, t=pose_msg.header.stamp)
        bag.write('path_gt', gt_path,t=gt_path.header.stamp)
    bag.close()

def process_rosbag(path,name):
    bag = rosbag.Bag(path+name)
    pose_pub = rospy.Publisher('path', PoseStamped, queue_size=10)

    navsatfix_msgs = []

    # Leer y almacenar mensajes por tipo
    for topic, msg, t in bag.read_messages(topics=['/simpleRTK2B/ublox_gps_node/fix']):
        navsatfix_msgs.append((msg, msg.header.stamp))

    # Sincronizar y procesar los mensajes
    find_pairs_and_publish(navsatfix_msgs, pose_pub,path)

    bag.close()
    
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Please provide the path to the bag file as an argument.")
        sys.exit(1)
    
    if len(sys.argv) < 3:
        print("Please provide the path and name as arguments.")
        sys.exit(1)
    
    path = sys.argv[1]
    name = sys.argv[2]
    bag_file_path = path + name
    process_rosbag(path,name)
