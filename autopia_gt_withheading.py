from geodesy import utm
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rosbag
import heapq
import sys
import math
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavRELPOSNED9
from tf.transformations import quaternion_from_euler


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
    for msg, t in navrelposned_msgs:
        heapq.heappush(ned_heap, (t, msg))
    while fix_heap and ned_heap:

        fix_time, fix_msg = heapq.heappop(fix_heap)
        ned_time, ned_msg = heapq.heappop(ned_heap)
        if abs(fix_time - ned_time) <= tolerance:

            if origin is None:
                origin = utm.fromLatLong(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude)
                origin_easting = origin.easting
                origin_northing = origin.northing
                origin_altitude = fix_msg.altitude
                origin_heading = ned_msg.relPosHeading * 1e-5  # Convertir de 1e-5 grados a grados
                
                

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
            
            # Orientación del heading
            heading = ned_msg.relPosHeading * 1e-5  # Convertir de 1e-5 grados a grados
            heading_rad = (heading - origin_heading) / 180.0 * math.pi  # Convertir a radianes
            pose_msg.pose.orientation = quaternion_from_euler(0, 0, heading_rad)
            
            gt_path.header.stamp = pose_msg.header.stamp
            gt_path.poses.append(pose_msg)

            bag.write('ground_truth', gt_path, t=gt_path.header.stamp)
        elif fix_time < ned_time:
            heapq.heappush(ned_heap, (ned_time, ned_msg))   
        else:   
            heapq.heappush(fix_heap, (fix_time, fix_msg))
    bag.close()
def estimate_orientation(navrelposned9_msgs, path):
        buffer = []
        for msg, t in navrelposned9_msgs:
            buffer.append(msg.relPosHeading * 1e-5)  # Convertir de 1e-5 grados a grados
            if t.to_sec() >= 3:
                break
            if len(buffer) > 5:
                mean_heading = sum(buffer) / len(buffer)
                std_dev = math.sqrt(sum((heading - mean_heading) ** 2 for heading in buffer) / len(buffer))
                if std_dev < 0.01:  # Adjust the threshold as needed
                    with open(path + 'orientation.txt', 'w') as file:
                        file.write(str(mean_heading))
                        print("Orientation estimation saved to orientation.txt")
                else:
                    buffer.pop(0)
                    print("Not enough reliable data for orientation estimation")
            else:
                print("No data available for orientation estimation")
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
