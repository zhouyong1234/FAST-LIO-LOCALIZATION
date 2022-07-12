//
// Created by bruce on 2022/3/29.
//

#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include "pclomp/ndt_omp.h"

using namespace std;

typedef pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> NDT;
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;

class Config
{
public:
    string odomFrame = "camera_init";
    struct
    {
        bool debug = false;
        int numThreads = 4;
        int maximumIterations = 20;
        float voxelLeafSize = 0.1;
        float resolution = 1.0;
        double transformationEpsilon = 0.01;
        double stepSize = 0.1;
        double threshShift = 2;
        double threshRot = M_PI / 12;
        double minScanRange = 1.0;
        double maxScanRange = 100;
    } ndt;

    explicit Config(ros::NodeHandle &nh) : _nh(nh)
    {
        _nh.getParam("odom_frame", odomFrame);

        _nh.getParam("ndt/debug", ndt.debug);
        _nh.getParam("ndt/num_threads", ndt.numThreads);
        _nh.getParam("ndt/maximum_iterations", ndt.maximumIterations);
        _nh.getParam("ndt/voxel_leaf_size", ndt.voxelLeafSize);
        _nh.getParam("ndt/transformation_epsilon", ndt.transformationEpsilon);
        _nh.getParam("ndt/step_size", ndt.stepSize);
        _nh.getParam("ndt/resolution", ndt.resolution);
        _nh.getParam("ndt/thresh_shift", ndt.threshShift);
        _nh.getParam("ndt/thresh_rot", ndt.threshRot);
        _nh.getParam("ndt/min_scan_range", ndt.minScanRange);
        _nh.getParam("ndt/max_scan_range", ndt.maxScanRange);
    }

private:
    ros::NodeHandle &_nh;
};

class Localizer
{
public:
    explicit Localizer(ros::NodeHandle &nh) :
            _nh(nh), _cfg(nh), _mapPtr(new Cloud), _mapFilteredPtr(new Cloud)
    {
        _mapSub = _nh.subscribe("/map_cloud", 10, &Localizer::mapCallback, this);
        _initPoseSub = _nh.subscribe("/initialpose", 10, &Localizer::initPoseWithNDTCallback, this);

        // _points_sub = _nh.subscribe("/velodyne_points", 10, &Localizer::pointsCallback, this);

        _ndtPosePub = nh.advertise<nav_msgs::Odometry>("/ndt/odometry", 100000);
        _ndtPathPub = nh.advertise<nav_msgs::Path>("/ndt/path", 100000);

        _pcSubPtr = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/velodyne_points", 1);
        _odomSubPtr = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/Odometry", 1);
        _syncPtr = new message_filters::Synchronizer<syncPolicy>(
                syncPolicy(10), *_pcSubPtr, *_odomSubPtr
        );
        _syncPtr->registerCallback(boost::bind(&Localizer::syncCallback, this, _1, _2));

        _voxelGridFilter.setLeafSize(_cfg.ndt.voxelLeafSize, _cfg.ndt.voxelLeafSize, _cfg.ndt.voxelLeafSize);
        _ndt.setNumThreads(_cfg.ndt.numThreads);
        _ndt.setTransformationEpsilon(_cfg.ndt.transformationEpsilon);
        _ndt.setStepSize(_cfg.ndt.stepSize);
        _ndt.setResolution(_cfg.ndt.resolution);
        _ndt.setMaximumIterations(_cfg.ndt.maximumIterations);

        _odomMap.setIdentity();
    }

private:
    ros::NodeHandle &_nh;
    ros::Subscriber _mapSub;
    ros::Subscriber _initPoseSub;
    ros::Subscriber _points_sub;
    ros::Publisher _ndtPosePub;
    ros::Publisher _ndtPathPub;
    tf2_ros::TransformBroadcaster _br;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *_pcSubPtr;
    message_filters::Subscriber<nav_msgs::Odometry> *_odomSubPtr;
//    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;
    message_filters::Synchronizer<syncPolicy> *_syncPtr;

    nav_msgs::Path _ndtPath;

    NDT _ndt;
    pcl::VoxelGrid<pcl::PointXYZI> _voxelGridFilter;
    Config _cfg;
    Cloud::Ptr _mapPtr, _mapFilteredPtr;
    tf::Pose _baseOdom, _odomMap;
    tf::Transform _baseMap;
    sensor_msgs::PointCloud2::ConstPtr _pcPtr = nullptr;

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Get map");
        pcl::fromROSMsg<pcl::PointXYZI>(*msg, *_mapPtr);
        _ndt.setInputTarget(_mapPtr);
    }

    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;
        tf::Pose baseMap(tf::Quaternion(q.x, q.y, q.z, q.w), tf::Vector3(p.x, p.y, p.z));
        _odomMap = baseMap * _baseOdom.inverse();
        ROS_INFO("Initial pose set");
    }

    void initPoseWithNDTCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        if (_pcPtr == nullptr)
        {
            ROS_WARN("No point cloud");
            return;
        }
        ROS_INFO("Initial pose set");
        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;
        tf::Pose baseMap(tf::Quaternion(q.x, q.y, q.z, q.w), tf::Vector3(p.x, p.y, p.z));

        match(_pcPtr, baseMap);
        publishTF();
    }

    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr &points_msg)
    {
        Cloud::Ptr tmpCloudPtr(new Cloud);
        pcl::fromROSMsg(*points_msg, *tmpCloudPtr);
        Cloud::Ptr filteredCloudPtr(new Cloud);
        _voxelGridFilter.setInputCloud(tmpCloudPtr);
        _voxelGridFilter.filter(*filteredCloudPtr);
        Cloud::Ptr scanCloudPtr(new Cloud);
        for(const auto &p: *filteredCloudPtr)
        {
            auto r = hypot(p.x, p.y);
            if(r > _cfg.ndt.minScanRange and r < _cfg.ndt.maxScanRange)
            {
                scanCloudPtr->push_back(p);
            }
        }

        _ndt.setInputSource(scanCloudPtr);

        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        Cloud::Ptr aligned(new Cloud);
        _ndt.align(*aligned, init_guess);

        Eigen::Matrix4f trans = _ndt.getFinalTransformation();
        Eigen::Vector3f p = trans.block<3, 1>(0,3);
        Eigen::Quaternionf q(trans.block<3, 3>(0,0));

        publishPoseTF(p, q);
    }

    void syncCallback(const sensor_msgs::PointCloud2::ConstPtr &pcMsg, const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        // std::cout << "syncCallback" << std::endl;
        _pcPtr = pcMsg;
        static chrono::steady_clock::time_point t0, t1;
        tf::poseMsgToTF(odomMsg->pose.pose, _baseOdom);
        static tf::Pose lastNDTPose = _baseOdom;

        auto T = lastNDTPose.inverseTimes(_baseOdom);

        if (hypot(T.getOrigin().x(), T.getOrigin().y()) > _cfg.ndt.threshShift or
            tf::getYaw(T.getRotation()) > _cfg.ndt.threshRot)
        {
            std::cout << _odomMap.getOrigin().x() << ", " << _odomMap.getOrigin().y() << std::endl;
            std::cout << _odomMap.getOrigin().getX() <<  ", " << _odomMap.getOrigin().getY() << std::endl;
            match(pcMsg, _odomMap * _baseOdom);
            lastNDTPose = _baseOdom;
        }
        // publishTF();
        // std::cout << "publish pose" << std::endl;
        // publishPose();
    }

    /**
     * Matching the point cloud with map to calculate `_odomMap`.
     *
     * @param pcPtr The point cloud for matching.
     * @param baseMap The guess matrix.
     * */
    void match(const sensor_msgs::PointCloud2::ConstPtr &pcPtr, const tf::Transform &baseMap)
    {
        static chrono::steady_clock::time_point t0, t1;

        Cloud::Ptr tmpCloudPtr(new Cloud);
        pcl::fromROSMsg(*pcPtr, *tmpCloudPtr);
        Cloud::Ptr filteredCloudPtr(new Cloud);
        _voxelGridFilter.setInputCloud(tmpCloudPtr);
        _voxelGridFilter.filter(*filteredCloudPtr);
        Cloud::Ptr scanCloudPtr(new Cloud);
        for (const auto &p: *filteredCloudPtr)
        {
            auto r = hypot(p.x, p.y);
            if (r > _cfg.ndt.minScanRange and r < _cfg.ndt.maxScanRange)
                scanCloudPtr->push_back(p);
        }

        _ndt.setInputSource(scanCloudPtr);
        Eigen::Affine3d baseMapMat;
        tf::poseTFToEigen(baseMap, baseMapMat);
        Cloud::Ptr outputCloudPtr(new Cloud);
        if (_cfg.ndt.debug) t0 = chrono::steady_clock::now();
        _ndt.align(*outputCloudPtr, baseMapMat.matrix().cast<float>());
        if (_cfg.ndt.debug) t1 = chrono::steady_clock::now();

        auto tNDT = _ndt.getFinalTransformation();
        tf::Transform baseMapNDT;
        tf::poseEigenToTF(Eigen::Affine3d(tNDT.cast<double>()), baseMapNDT);
        _odomMap = baseMapNDT * _baseOdom.inverse();
        _baseMap = baseMapNDT;

        Eigen::Vector3f pose_map;
        Eigen::Quaternionf rotation_map;
        pose_map[0] = _baseMap.getOrigin().getX();
        pose_map[1] = _baseMap.getOrigin().getY();
        pose_map[2] = _baseMap.getOrigin().getZ();

        rotation_map.x() = _baseMap.getRotation().getX();
        rotation_map.y() = _baseMap.getRotation().getY();
        rotation_map.z() = _baseMap.getRotation().getZ();
        rotation_map.w() = _baseMap.getRotation().getW();

        publishPoseTF(pose_map, rotation_map);

        // std::cout << _baseMap.getOrigin().getX() << ", " << _baseMap.getOrigin().getY() << std::endl;

        if (_cfg.ndt.debug) ROS_INFO("NDT: %ldms", chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());
        ROS_INFO("NDT Relocated");
    }

    void publishTF()
    {
        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.stamp = ros::Time::now();
        tfMsg.header.frame_id = "map";
        tfMsg.child_frame_id = _cfg.odomFrame;
        tfMsg.transform.translation.x = _odomMap.getOrigin().x();
        tfMsg.transform.translation.y = _odomMap.getOrigin().y();
        tfMsg.transform.translation.z = _odomMap.getOrigin().z();
        tfMsg.transform.rotation.x = _odomMap.getRotation().x();
        tfMsg.transform.rotation.y = _odomMap.getRotation().y();
        tfMsg.transform.rotation.z = _odomMap.getRotation().z();
        tfMsg.transform.rotation.w = _odomMap.getRotation().w();

        _br.sendTransform(tfMsg);
    }

    void publishPoseTF(Eigen::Vector3f &p, Eigen::Quaternionf &q)
    {
        nav_msgs::Odometry pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.child_frame_id = "velodyne";
        pose.pose.pose.position.x = p[0];
        pose.pose.pose.position.y = p[1];
        pose.pose.pose.position.z = p[2];
        pose.pose.pose.orientation.x = q.x();
        pose.pose.pose.orientation.y = q.y();
        pose.pose.pose.orientation.z = q.z();
        pose.pose.pose.orientation.w = q.w();
        _ndtPosePub.publish(pose);

        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.header.stamp = pose.header.stamp;
        this_pose_stamped.header.frame_id = "map";
        this_pose_stamped.pose.position.x = p[0];
        this_pose_stamped.pose.position.y = p[1];
        this_pose_stamped.pose.position.z = p[2];
        this_pose_stamped.pose.orientation.x = q.x();
        this_pose_stamped.pose.orientation.y = q.y();
        this_pose_stamped.pose.orientation.z = q.z();
        this_pose_stamped.pose.orientation.w = q.w();

        _ndtPath.header.stamp = pose.header.stamp;
        _ndtPath.header.frame_id = "map";
        _ndtPath.poses.push_back(this_pose_stamped);
        _ndtPathPub.publish(_ndtPath);


        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion rotation;
        transform.setOrigin(tf::Vector3(p[0], p[1], p[2]));
        rotation.setW(q.w());
        rotation.setX(q.x());
        rotation.setY(q.y());
        rotation.setZ(q.z());
        transform.setRotation(rotation);
        br.sendTransform(tf::StampedTransform(transform, pose.header.stamp, "map", "velodyne"));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_lio_localization");
    ros::NodeHandle nh("~");

    Localizer localizer(nh);

    ros::spin();

    return 0;
}