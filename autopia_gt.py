import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from geopy.distance import geodesic
import heapq
import rospy
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavRELPOSNED9
from geometry_msgs.msg import PoseStamped
import rosbag
from tf.transformations import quaternion_from_euler
import math
def find_pairs_and_publish(navsatfix_msgs, navrelposned9_msgs, pose_pub, time_tolerance=0.1):
    tolerance = rospy.Duration.from_sec(time_tolerance)

    fix_heap = []
    ned_heap = []

    # Variable para almacenar la posición inicial
    origin = None

    for msg, t in navsatfix_msgs:
        heapq.heappush(fix_heap, (t, msg))
    for msg, t in navrelposned9_msgs:
        heapq.heappush(ned_heap, (t, msg))

    while fix_heap and ned_heap:
        fix_time, fix_msg = heapq.heappop(fix_heap)
        ned_time, ned_msg = heapq.heappop(ned_heap)

        time_diff = fix_time - ned_time

        if abs(time_diff) <= tolerance:
            if origin is None:
                # Establecer la primera posición recibida como el origen
                origin = (fix_msg.latitude, fix_msg.longitude)
                origin_altitude = fix_msg.altitude

            # Calcular la posición relativa usando geodesic
            current_position = (fix_msg.latitude, fix_msg.longitude)
            distance = geodesic(origin, current_position).meters
            bearing = geodesic(origin, current_position).initial
            dx = distance * math.sin(bearing)
            dy = distance * math.cos(bearing)
            dz = fix_msg.altitude - origin_altitude

            pose_msg = PoseStamped()
            pose_msg.header.stamp = fix_time
            pose_msg.header.frame_id = "world"

            # Posiciones relativas
            pose_msg.pose.position.x = dx
            pose_msg.pose.position.y = dy
            pose_msg.pose.position.z = dz

            # Orientación del heading
            heading = ned_msg.heading * 1e-5  # Convertir de 1e-5 grados a grados
            heading_rad = heading / 180.0 * math.pi  # Convertir a radianes
            pose_msg.pose.orientation = quaternion_from_euler(0, 0, heading_rad)

            pose_pub.publish(pose_msg)
        elif time_diff < -tolerance:
            heapq.heappush(ned_heap, (ned_time, ned_msg))
        else:
            heapq.heappush(fix_heap, (fix_time, fix_msg))


def process_rosbag(bag_file_path):
    rospy.init_node('pose_converter_from_bag')
    bag = rosbag.Bag(bag_file_path)
    pose_pub = rospy.Publisher('path', PoseStamped, queue_size=10)

    navsatfix_msgs = []
    navrelposned9_msgs = []

    # Leer y almacenar mensajes por tipo
    for topic, msg, t in bag.read_messages(topics=['navsatfix_topic', 'navrelposned9_topic']):
        if topic == 'navsatfix_topic':
            navsatfix_msgs.append((msg, t))
        elif topic == 'navrelposned9_topic':
            navrelposned9_msgs.append((msg, t))

    # Aquí se implementaría lógica para sincronizar manualmente estos mensajes
    # Por ejemplo, buscar pares de mensajes que estén dentro de un umbral de tiempo aceptable

    bag.close()
    
if __name__ == '__main__':
    process_rosbag('/path/to/your/bagfile.bag')