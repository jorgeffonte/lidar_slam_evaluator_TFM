import os
import subprocess

def run_compare_on_bag(bag_path):
    print(f"Ejecutando compare.py para {bag_path}")
    
    # Build the command
    cmd = [
        "python", "compare.py",
        "--dataset", "utbm",
        "--slam","faster_lio", "fast_lio", "dlo","dlio", "kiss_icp","f_loam",
        "--bag_path", bag_path,
        "--plot", "all", "--no_play"
    ]
    
    print(f"Executing command: {' '.join(cmd)}")
    # run rosclean purge -y
    subprocess.run("rosclean purge -y", shell=True, check=False)
    try:
        # Run the command and wait for it to complete
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        # Print error message but continue with the mv command
        print(f"Command failed with error: {e}. Proceeding to move files.")
if __name__ == "__main__":
    # Ruta del directorio raíz donde se encuentran las carpetas con los archivos .bag
    root_directory = os.path.expanduser("~/TFM_ws/utbm_dataset")
    
    # Recorrer todos los subdirectorios y archivos
    for root, dirs, files in os.walk(root_directory):
        for file in files:
            if file == 'utbm.bag':
                bag_path = os.path.join(root, file)
                run_compare_on_bag(root)
