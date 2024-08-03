import os
import subprocess

# Directorio base
base_dir = "/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences"

# Función para ejecutar el comando compare.py
def run_compare_script(seq_num):
    compare_ros_dir = os.path.join(base_dir, f"{seq_num:02d}", "compare_ros")
    
    # Verifica si el directorio de compare_ros existe
    if not os.path.isdir(compare_ros_dir):
        print(f"Directorio {compare_ros_dir} no encontrado, saltando...")
        return
    
    # Encuentra la subcarpeta dentro de compare_ros
    sub_dirs = [d for d in os.listdir(compare_ros_dir) if os.path.isdir(os.path.join(compare_ros_dir, d))]
    if len(sub_dirs) != 1:
        print(f"No se encontró una única subcarpeta en {compare_ros_dir}, saltando...")
        return
    
    bag_dir = os.path.join(compare_ros_dir, sub_dirs[0])
    
    # Construye la ruta del archivo bag
    bag_path = bag_dir
    
    # Construye el comando
    cmd = [
        "python", "compare.py",
        "--slam", "all",
        "--bag_path", bag_path,
        "--plot", "all","--no_play"
        
    ]
    
    print(f"Ejecutando comando: {' '.join(cmd)}")
    
    # Ejecuta el comando
    subprocess.run(cmd)

# Itera sobre cada secuencia del 00 al 10
for seq_num in range(11):
    run_compare_script(seq_num)
