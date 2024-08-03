import rosbag
import rospy
from sensor_msgs.msg import Imu, PointCloud2
import numpy as np
import copy
import os

# Configuración inicial
imu_topic = "/imu_raw"
lidar_topic = "/points_raw"
imu_frequency = 100.0  # Hz
lidar_frequency = 10.0  # Hz
seconds_to_add = 10.0

# Función para crear un mensaje de IMU estático
def create_static_imu_message(timestamp):
    imu_msg = Imu()
    imu_msg.header.stamp = timestamp
    imu_msg.linear_acceleration.x = 0.0
    imu_msg.linear_acceleration.y = 0.0
    imu_msg.linear_acceleration.z = 9.81
    imu_msg.angular_velocity.x = 0.0
    imu_msg.angular_velocity.y = 0.0
    imu_msg.angular_velocity.z = 0.0
    imu_msg.orientation_covariance[0] = -1  # Indicates no orientation data
    return imu_msg

# Función para procesar un archivo .bag
def process_bag(input_bag_path, output_bag_path):
    input_bag = rosbag.Bag(input_bag_path, 'r')
    output_bag = rosbag.Bag(output_bag_path, 'w')
    
    try:
        # Encuentra los primeros mensajes de cada tópico
        first_imu_msg = None
        first_lidar_msg = None
        for topic, msg, t in input_bag.read_messages(topics=[imu_topic, lidar_topic]):
            if topic == imu_topic and first_imu_msg is None:
                first_imu_msg = msg
                first_imu_time = msg.header.stamp
                imu_framde_id = msg.header.frame_id
            if topic == lidar_topic and first_lidar_msg is None:
                first_lidar_msg = msg
                first_lidar_time = msg.header.stamp
            if first_imu_msg and first_lidar_msg:
                break

        # Calcula el tiempo inicial para añadir mensajes
        start_time = min(first_imu_time, first_lidar_time) - rospy.Duration(seconds_to_add)

        # Añade mensajes de IMU estáticos
        for i in range(int(seconds_to_add * imu_frequency)):
            timestamp = start_time + rospy.Duration(i / imu_frequency)
            imu_msg = create_static_imu_message(timestamp)
            imu_msg.header.frame_id = imu_framde_id
            output_bag.write(imu_topic, imu_msg, timestamp)

        # Añade mensajes de LIDAR replicando el primer mensaje
        for i in range(int(seconds_to_add * lidar_frequency)):
            timestamp = start_time + rospy.Duration(i / lidar_frequency)
            lidar_msg = copy.deepcopy(first_lidar_msg)
            lidar_msg.header.stamp = timestamp
            output_bag.write(lidar_topic, lidar_msg, timestamp)

        # Copia los mensajes originales al nuevo bag
        for topic, msg, t in input_bag.read_messages():
            output_bag.write(topic, msg, t)
    finally:
        input_bag.close()
        output_bag.close()

# Directorio base
base_dir = "/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences"

# Itera sobre cada secuencia del 00 al 10
for seq_num in [4]:
    seq_dir = os.path.join(base_dir, f"{seq_num:02d}", "compare_ros")
    
    # Verifica si el directorio de la secuencia existe
    if not os.path.isdir(seq_dir):
        print(f"Directorio {seq_dir} no encontrado, saltando...")
        continue
    
    # Encuentra la subcarpeta dentro de compare_ros
    sub_dirs = [d for d in os.listdir(seq_dir) if os.path.isdir(os.path.join(seq_dir, d))]
    if len(sub_dirs) != 1:
        print(f"No se encontró una única subcarpeta en {seq_dir}, saltando...")
        continue
    
    bag_dir = os.path.join(seq_dir, sub_dirs[0])
    
    # Busca archivos .bag en el directorio
    bag_file= "kitti.bag"
    input_bag_path = os.path.join(bag_dir, bag_file)
    output_bag_path = os.path.join(bag_dir, f"output_{bag_file}")
    
    print(f"Procesando {input_bag_path}...")
    process_bag(input_bag_path, output_bag_path)

    # Borrar el archivo original y renombrar el nuevo archivo
    # os.remove(input_bag_path)
    # os.rename(output_bag_path, input_bag_path)

    print(f"Archivo original {input_bag_path} borrado y nuevo archivo renombrado.")