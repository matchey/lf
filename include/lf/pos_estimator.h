
#ifndef LOCALIZATION_POS_ESTIMATER_H
#define LOCALIZATION_POS_ESTIMATER_H

#include <ros/ros.h>
#include <random>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Geometry> // for affine
#include "ceres_msgs/AMU_data.h"
#include "ceres_msgs/GPSFix.h"

class NodeEdgeManager;
class NeGridMapCreator;
class Particle;
struct Point2D;

class PosEstimator
{
	ros::NodeHandle n;
	ros::Subscriber odom_sub;
	ros::Subscriber gps_sub;
	ros::Subscriber ff_sub;
	ros::Publisher pos_pub;
	ros::Publisher corrected_pub;
	ros::Publisher lcl_pub; // for sawahashi
	ros::Publisher flag_diff_pub; // for sawahashi
	ros::Publisher edge_pub; // for sawahashi
	ros::Publisher odom_pub;
	tf::TransformBroadcaster pos_broadcaster;
	tf::TransformBroadcaster tiny_broadcaster;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time, last_time;
	double x;
	double y;
	double yaw; // [rad]
	double lat;
	double lng;
	double yaw_ave; // [rad]
	std::vector< std::vector<Point2D> > lines;
	std::vector<int> ignore_edge; // [rad]
	int cnt_yaw;
	double x_init; // [rad]
	double y_init; // [rad]
	double yaw_init; // [rad]
	double yaw_correction; // [rad]
	int flag_correction;
	bool flag_clear;
	bool flag_turn;
	bool flag_ignore;
	Eigen::Affine2d odom_correction; // [-Wreorder]
	NodeEdgeManager* nem;
	NeGridMapCreator* ngmc;
	bool flag_finish;

	double forward_noise;
	double mu;
	static std::random_device rnd;
	std::mt19937_64 mt;
	std::normal_distribution<> error;

	void setGauss();
	void setIgnoreEdge();
	double localPCA();
	void fit_odom(int, const Eigen::Affine2d&);
	void calcCorrection(const int&, double&, double&, Eigen::Affine2d&);//.., ratio, yaw_diff, ..
	void corrector();
	void corrector_tentatively();
	void ignoreEdge(int);
	void odomPublisher(double, double);

	public:
	PosEstimator();
	~PosEstimator();
	void odomCallback(const nav_msgs::Odometry::ConstPtr&);
	void gpsCallback(const ceres_msgs::GPSFix::ConstPtr&);
	void ffCallback(const std_msgs::Bool::ConstPtr&);
	void setNoise(double);
	void setMu(double);
	void setParticlePose(int, int, double, Particle&); //node1, node2, 0.0 ~ 1.0
	void setRobotPose(int, int, double); //node1, node2, 0.0 ~ 1.0
	void setParticle2nearEdge(int, Particle&); //uniq, 
	void particleManager(Particle&);
	void globalPCA();
	void onEdgeManager(int);
	void getAve();
	void posPublisher(Particle);
	void tinyPublisher();
	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

