import rosbag
import rospy
import subprocess
import os
import signal

import copy
def create_static_bag(original_bag_path, static_bag_path, topic_info, static_duration):
    # Abrir el rosbag original
    with rosbag.Bag(original_bag_path, 'r') as bag:
        # Diccionario para almacenar el primer mensaje de cada tópico
        first_msgs = {}
        first_times = {}
        
        # Buscar el primer mensaje de cada tópico especificado
        for topic, msg, t in bag.read_messages(topics=topic_info.keys()):
            if topic not in first_msgs:
                first_msgs[topic] = msg
                first_times[topic] = t
            if len(first_msgs) == len(topic_info):  # Todos los tópicos han sido encontrados
                break

    # Comprobar si se encontraron mensajes para todos los tópicos
    if len(first_msgs) != len(topic_info):
        missing_topics = set(topic_info.keys()) - set(first_msgs.keys())
        print(f"No se encontraron mensajes para los tópicos: {missing_topics}")
        return

    # Crear un rosbag con los mensajes estáticos
    with rosbag.Bag(static_bag_path, 'w') as static_bag:
        for topic, info in topic_info.items():
            frequency = info['frequency']
            num_messages = int(static_duration * frequency)
            time_shift = rospy.Duration(1.0 / frequency)
            for i in range(num_messages):
                new_time = first_times[topic] - time_shift * (num_messages - i)
                new_msg = copy.deepcopy(first_msgs[topic])
                
                # Ajustar el campo del tiempo interno del mensaje, si existe
                if hasattr(new_msg, 'header') and hasattr(new_msg.header, 'stamp'):
                    new_msg.header.stamp = new_time
                
                static_bag.write(topic, new_msg, new_time)

    print(f"Rosbag estático creado exitosamente en {static_bag_path}")


def merge_bags(static_bag_path, original_bag_path, merged_bag_path):
    # Iniciar el comando para grabar el rosbag en segundo plano
    record_command = ["rosbag", "record", "-a", "-O", merged_bag_path]
    record_process = subprocess.Popen(record_command)

    try:
        # Ejecutar el comando para reproducir los rosbags
        play_command = ["rosbag", "play", "--clock", static_bag_path, original_bag_path]
        subprocess.run(play_command, check=True)

    finally:
        # Asegurarse de terminar el proceso de grabación una vez finalizado el play
        record_process.send_signal(signal.SIGINT)  # Enviar señal de interrupción para terminar record
        record_process.wait()  # Esperar a que el proceso termine

    print(f"Rosbags fusionados exitosamente en {merged_bag_path}")

if __name__ == '__main__':
    # Configuración
    original_bag_path = '/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences/01/compare_ros/kitti_2011_10_03_drive_0042_synced/kitti.bag'
    static_bag_path = '/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences/01/compare_ros/kitti_2011_10_03_drive_0042_synced/static_part.bag'
    merged_bag_path = '/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences/01/compare_ros/kitti_2011_10_03_drive_0042_synced/combined.bag'
    topic_info = {
        '/kitti/oxts/imu': {'frequency': 100},  # 100 Hz para kitti/oxts/imu
        '/points_raw': {'frequency': 10}  # 10 Hz para points_raw
    }
    static_duration = 10  # Duración en segundos de la parte estática

    # Crear la parte estática
    create_static_bag(original_bag_path, static_bag_path, topic_info, static_duration)

    # Fusionar el rosbag estático con el original
    merge_bags(static_bag_path, original_bag_path, merged_bag_path)
