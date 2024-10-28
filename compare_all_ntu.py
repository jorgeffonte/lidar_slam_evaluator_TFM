import os
import subprocess

# Base directory
base_dir = "/home/dronomy/TFM_ws/NTU_data/ntu_sequences"
# "/home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences"

# Function to run the compare.py script
def run_compare_script(seq_name):
    compare_ros_dir = os.path.join(base_dir, seq_name)
    
    # Check if the compare_ros directory exists
    if not os.path.isdir(compare_ros_dir):
        print(f"Directory {compare_ros_dir} not found, skipping...")
        return
    
    bag_dir = compare_ros_dir
    
    # Build the path to the bag file
    bag_path = bag_dir
    
    # Build the command
    cmd = [
        "python", "compare.py",
        "--dataset", "ntu",
        "--slam", "dlio",# "dlo", "faster_lio", "fast_lio",
        "--bag_path", bag_path,
        "--plot", "all"
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
    
    
    # Move all files from bag_path/result/* to base_dir/viral_eval_TFM/result_{seq_name}
    result_dir = os.path.join(base_dir, f"../viral_eval_TFM/result_{seq_name}")
    os.makedirs(result_dir, exist_ok=True)
    
    # Using `shutil.move` to handle the file movement properly
    cp_cmd = f"cp {os.path.join(bag_path, 'result', '*')} {result_dir}/"
    
    print(f"Moving results to {result_dir}")
    subprocess.run(cp_cmd, shell=True, check=False)

# Iterate over each sequence
names = ["sbs_01", "eee_02", "eee_03", "spms_02", "rtp_03"]
names = [ "spms_02", "rtp_03"]

# run_compare_script("eee_03")

def remove_names(original_list, names_to_remove):
    return [name for name in original_list if name not in names_to_remove]

# Example usage
names = ["eee_01", "eee_02", "eee_03", "nya_01", "nya_03", "rtp_01", "rtp_02", "rtp_03", "sbs_01", "spms_01", "spms_02", "tnp_02"]
names_to_remove = ["sbs_01", "eee_02", "eee_03", "spms_02", "rtp_03"]
updated_names = remove_names(names, names_to_remove)
print(updated_names)

for seq_name in names:
    run_compare_script(seq_name)