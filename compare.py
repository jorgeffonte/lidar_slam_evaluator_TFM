
# python compare.py --slam lego_loam lio_sam aloam kiss_icp --bag_path /home/dronomy/TFM_ws/kitti_data/odom/dataset/sequences/07/compare_ros/kitti_2011_09_30_drive_0027_synced --plot all --no_play

import argparse
import time
import yaml
from subprocess import *
from rosbag.bag import Bag
import matplotlib.pyplot as plt

import src.trajectory as tj
import src.error as error

packages_list = {
    'aloam': ['record_aloam.launch', 'record_aloam_nclt.launch'],
    'lego_loam': ['record_lego_loam.launch', 'record_lego_loam_nclt.launch'],
    'lio_sam': ['record_lio_sam.launch', 'record_lio_sam_nclt.launch'],
    'kiss_icp': ['record_kiss_icp.launch', 'record_kiss_icp_nclt.launch'],
    'dlo': ['record_dlo.launch', 'record_dlo_nclt.launch'],
    'f_loam': ['record_f_loam.launch', 'record_f_loam_nclt.launch'],
    'faster_lio': ['record_faster_lio.launch', 'record_faster_lio_nclt.launch'],
    'fast_lio': ['record_fast_lio.launch', 'record_fast_lio_nclt.launch']
}

parser = argparse.ArgumentParser(description="Lidar SLAM Evaluator")
parser.add_argument('--dataset', nargs="+", dest='dataset', help='SLAM algorithms that want to compare: kitti, nclt')
parser.add_argument('--slam', nargs="+", dest='slam_packages', help='SLAM algorithms that want to compare: aloam, lego_loam, lio_sam')
parser.add_argument('--bag_path', dest='bagfile_path', help='Directory path where KITTI dataset bag file "kitti.bag" exists')
parser.add_argument('--plot', dest='plot_arg', choices=['all', 'traj', 'error', 'stat'], default='all', help='Plot options: all, traj, error, stat')
parser.add_argument('--no_play', dest='play_flag', action='store_false', help='If you already have recorded result bag file, add --no_play')
args = parser.parse_args()

if args.dataset is None:
    print("No dataset specified. Using KITTI dataset by default.")
    args.dataset = ["kitti"]
if args.slam_packages is None:
	print("No specified SLAM Lists. Run default algorithms: ALOAM, LEGO_LOAM, LIO_SAM")
	args.slam_packages = ["aloam", "lego_loam", "lio_sam", "kiss_icp"]

if 'all' in args.slam_packages:
    args.slam_packages = ["aloam", "lego_loam", "lio_sam", "kiss_icp"]

for slam in args.slam_packages:
    if slam not in packages_list:
        raise ValueError("%s algorithm is not available!" % slam)

if args.bagfile_path is None:
	raise parser.error("Please set bag file directory with --bag_path parser")

class CompareSLAM():
    def __init__(self, slam_packages, bag_path, plot_arg):
        self.slam_packages = slam_packages
        self.bag_path = bag_path
        self.bag_file = bag_path + '/'+args.dataset[0]+'.bag'
        bag_info_dict = yaml.safe_load(Bag(self.bag_file, 'r')._get_yaml_info())
        self.bag_duration = bag_info_dict['duration']
        self.plot_arg = plot_arg
        self.file_list = []
        print(args.dataset)
        self.file_list.append(bag_path + '/'+args.dataset[0]+'_gt.bag')
        for slam in self.slam_packages:
            self.file_list.append(self.bag_path +'/'+ slam + '_path.bag')
        
    def play_algorithm(self):     
        for slam in self.slam_packages:            
            print("%s algorithm is running ..." % slam)
            print("bag file duration: %i" % self.bag_duration)
            print(self.bag_file)
            p1 = Popen(["roslaunch", "path_recorder", 'record_'+slam+'_'+args.dataset[0]+'.launch', "bag_path:=" + self.bag_path])
            
            start_time = time.time()
            currnet_time = start_time
            while True:
                time.sleep(1)
                current_time = time.time()
                if current_time - start_time >= self.bag_duration:
                    print("finishing %s ..."% slam)
                    p1.terminate()
                    p1.wait()
                    break
            
        print("Finished all algorithm")
        self.plot()

    def plot(self):
        plot_arg = self.plot_arg
        file_list = self.file_list
        gt, tj_list = self.traj_process(file_list)
        if plot_arg == 'traj' : return self.plot_traj(gt, tj_list)
        else:
            error_list = self.error_process(gt, tj_list)
            return self.plot_error(plot_arg, gt, tj_list, error_list)

    def traj_process(self, data_files):
        tj_list = []
        gt = None
        for file in data_files:
            if (file.endswith('.bag') or file.endswith('.txt')):
                trajectory = tj.Trajectory(file)
                if(not trajectory.is_gt):
                    tj_list.append(trajectory)
                else: gt = trajectory
            else: print("Unsupported .{} file type".format(file.split('.')[-1]))
        return gt, tj_list

    def error_process(self, gt, tj_list):
        error_list = []
        if(gt == None):
            print('Need ground truth for error calculation.')
            return
        for tj in tj_list:
            error_list.append(error.Error(gt, tj))
        return error_list

    def plot_traj(self, gt, tj_list):
        print("plotting...")
        tj.plotXYZ(gt, tj_list)
        tj.plot2D('xy', gt, tj_list)
        tj.plot3D(gt, tj_list)
        return plt.show()

    def plot_error(self, plot_arg, gt, tj_list, error_list):
        print("plotting...")
        if (plot_arg == 'all'):
            tj.plotXYZ(gt, tj_list)
            tj.plot2D('xy', gt, tj_list)
            tj.plot3D(gt, tj_list)
        
            error.plotAPE(error_list)
            error.plotAPEStats(error_list)
            error.plotRPE(error_list)
            error.plotRPEStats(error_list)
            
        if (plot_arg == 'error'):    
            error.plotAPE(error_list)
            error.plotAPEStats(error_list)
            error.plotRPE(error_list)
            error.plotRPEStats(error_list)
            
        if (plot_arg == 'stat'):
            error.plotAPEStats(error_list)
            error.plotRPEStats(error_list)
        return self.print_result(error_list), plt.show()
    
    def print_result(self, error_list):
        for error in error_list:
            print(error.name)
            print("APE[m]   : {}".format(error.ape_tans_stat[5]))
            print("RPE[m]   : {}".format(error.rpe_tans_stat[5]))
            print("RPE[rad] : {}".format(error.rpe_rot_stat[0]))


if __name__ == '__main__':
    compare = CompareSLAM(args.slam_packages, args.bagfile_path, args.plot_arg)
    if args.play_flag:
        compare.play_algorithm()
    else:
        compare.plot()
            
