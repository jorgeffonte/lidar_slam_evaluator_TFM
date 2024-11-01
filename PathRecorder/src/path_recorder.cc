#include "path_recorder.h"

void PathRecorder::Record()
{
    boost::filesystem::create_directory(save_to_);

    rosbag::Bag bag,bag_time;
    ros::Subscriber sub_path,sub_comp;
    if (algorithm_type_ == path_recorder::Algorithm::A_LOAM)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::AloamHandler, this,
                                                 ros::TransportHints().tcpNoDelay());
    else if (algorithm_type_ == path_recorder::Algorithm::LeGO_LOAM)
        sub_path = nh_.subscribe<nav_msgs::Odometry>(path_topic_,
                                                     100,
                                                     &PathRecorder::LegoloamHandler, this,
                                                     ros::TransportHints().tcpNoDelay());
    else if(algorithm_type_ == path_recorder::Algorithm::LIO_SAM)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::LiosamHandler, this,
                                                 ros::TransportHints().tcpNoDelay());
                                                 
    else if(algorithm_type_ == path_recorder::Algorithm::KISS_ICP)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::KissHandler, this,
                                                 ros::TransportHints().tcpNoDelay());

    else if(algorithm_type_ == path_recorder::Algorithm::DLO)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::DloHandler, this,
                                                 ros::TransportHints().tcpNoDelay());
                                              
                                                 
    
    else if(algorithm_type_ == path_recorder::Algorithm::F_LOAM)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::FloamHandler, this,
                                                 ros::TransportHints().tcpNoDelay());

    else if(algorithm_type_ == path_recorder::Algorithm::FASTER_LIO)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::FasterLioHandler, this,
                                                 ros::TransportHints().tcpNoDelay());                                                                                          
    else if(algorithm_type_ == path_recorder::Algorithm::FAST_LIO)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::FastLioHandler, this,
                                                 ros::TransportHints().tcpNoDelay());                                                                                          
    sub_comp = nh_.subscribe<std_msgs::Float64>(comp_topic_,
                                                 100,
                                                 &PathRecorder::CompHandler, this,
                                                 ros::TransportHints().tcpNoDelay());   
    std::string bag_fn = save_to_ + "/" + algorithm_name_ + "_path.bag";
    std::string bag_time_fn = save_to_ + "/" + algorithm_name_ + "_time.bag";

    bag.open(bag_fn, rosbag::bagmode::Write);
    bag_time.open(bag_time_fn, rosbag::bagmode::Write);

    while (ros::ok())
    {
        if (updated_)
        {
            std::unique_lock<std::mutex> lock(path_mtx_);
            bag_time.write(comp_topic_, ros::Time::now(), time_comp_rcvd_);
            bag.write(algorithm_name_ + "_path", ros::Time::now(), path_rcvd_);
            updated_ = false;
        }
        ros::spinOnce();
    }

    bag.close();
    std::cout << "Recorded path bag file is saved in: " << bag_fn  << std::endl;
}

void PathRecorder::CompHandler(const std_msgs::Float64 msg)
{
    time_comp_rcvd_.data=msg.data;
}
void PathRecorder::AloamHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}
void PathRecorder::LiosamHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}
void PathRecorder::KissHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}
void PathRecorder::FasterLioHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}
void PathRecorder::FastLioHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}
void PathRecorder::FloamHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}

void PathRecorder::DloHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses = path->poses;
    updated_ = true;
}
void PathRecorder::LegoloamHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = odom->header.seq;
    path_rcvd_->header.frame_id = "/map";

	tf::Quaternion q_rot = tf::createQuaternionFromRPY(0, -M_PI / 2, M_PI / 2);

    geometry_msgs::PoseStamped pose_lego_loam;
    pose_lego_loam.header = odom->header;
    pose_lego_loam.pose.position.x = odom->pose.pose.position.z;
    pose_lego_loam.pose.position.y = odom->pose.pose.position.x;
    pose_lego_loam.pose.position.z = odom->pose.pose.position.y;
    
    tf::Quaternion q_orientation, q_orientation_rot;	
	tf::quaternionMsgToTF(odom->pose.pose.orientation, q_orientation);
    q_orientation_rot = q_rot * q_orientation;
    q_orientation_rot.normalize();
	tf::quaternionTFToMsg(q_orientation_rot, pose_lego_loam.pose.orientation);
    path_rcvd_->poses.push_back(pose_lego_loam);
    updated_ = true;
}


