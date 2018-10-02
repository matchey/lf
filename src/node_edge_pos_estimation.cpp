//
// src: node_edge_pos_estimation.cpp
//
// last update: '17.10.29
// author: matchey
//
// memo:
//   Line Fitting Localization (LFL) を実行するためのメインプログラム
//

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include "pf/particle.h"
#include "lf/pos_estimator.h"

using namespace std;

bool is_run = false;
void isRunCallback(const std_msgs::Bool::ConstPtr& msg)
{
	is_run = msg->data;
}

void set_init_pose(Particle p[], int num, PosEstimator &pe, XmlRpc::XmlRpcValue in)
{
	pe.setRobotPose(in["begin"], in["end"], in["div"]);
	for(int i=0; i<num; i++){
		pe.setParticlePose(in["begin"], in["end"], in["div"], p[i]);
	}
}

void particle_filter(Particle p[], int num, PosEstimator &pe)
{
	static int cnt = 0;
	double max = 0;
	double w_i = 0;
	// int max_i = 0;
	int uniq = 1;
// #pragma omp parallel private(w_i)
	// p[1].setFast(true);
	{
		// #pragma omp for
		// #pragma omp parallel for
		for(int i=0; i<num; i++){
			pe.particleManager(p[i]);
		}
		// #pragma omp for
		// #pragma omp parallel for
		for(int i=0; i<num; i++){
			w_i = p[i].getWeight();
			if(max < w_i ){
				max = w_i;
				// max_i = i;
			}
			if(uniq && (p[i].getEdge() != p[0].getEdge() || p[i].flag)){
				uniq = 0;
			}
		}
		if(max){
		// #pragma omp for
		// #pragma omp parallel for
			for(int i=0; i<num; i++){
				w_i = p[i].getWeight() / max;
				p[i].setWeight(w_i);
			}
		}
		if(uniq){
			uniq = p[0].getEdge();
		}else{
			uniq = -1;
		}
	}
	pe.tinyPublisher();
	if(!(cnt%3)){
		// p[1].setWeight(1.0);
		if(is_run) resampling(p, num);
		// #pragma omp parallel for
		for(int i=0; i<num; i++){
			p[i].setWeight(1.0);
		}
		cnt=0;
	}
	pe.globalPCA();
	pe.onEdgeManager(uniq);
	pe.setParticle2nearEdge(uniq, p[0]);
	cnt++;
}

void PublishParticles(Particle p[], int num, ros::Publisher particle_pub)
{
	geometry_msgs::PoseArray particles_msg;
	particles_msg.header.stamp = ros::Time::now();
	particles_msg.header.frame_id = "map";

	for(int i=0; i<num; i++)
	{
		geometry_msgs::Pose pose;

		geometry_msgs::Quaternion robot_quat=tf::createQuaternionMsgFromYaw(p[i].getYaw());

		pose.position.x = p[i].getX();
		pose.position.y = p[i].getY();
		pose.orientation.x = robot_quat.x;
		pose.orientation.y = robot_quat.y;
		pose.orientation.z = robot_quat.z;
		pose.orientation.w = robot_quat.w;

		particles_msg.poses.insert(particles_msg.poses.begin(), pose);
	}
	particle_pub.publish(particles_msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "node_edge_pos_estimation");
	cout << "node-edge graph based localization" << endl;

	ros::NodeHandle n;
	ros::Publisher particle_pub = n.advertise<geometry_msgs::PoseArray>("/particles/odom", 50);
	ros::Subscriber flag_is_run_sub = n.subscribe<std_msgs::Bool>("/flag/is_run", 1, isRunCallback);

	ros::Time::init();
	ros::Rate loop_rate(25);

	int num = 1000;
	n.getParam("/particle/size", num);

	if(argc == 2){
		istringstream ss(argv[1]);
		if (!(ss >> num)) cerr << "Invalid number " << argv[1] << '\n';
	}
	cout << "particle size: " << num << endl;

	XmlRpc::XmlRpcValue init_node;
	n.getParam("/init/node", init_node);
	cout << "init_node: " << init_node << endl;

	Particle* p;
	p = new Particle[num];
	p[0].setWeight(p[0].getWeight()*2);

	PosEstimator pe;

	set_init_pose(p, num, pe, init_node);

	while(ros::ok()){
		particle_filter(p, num, pe);
		PublishParticles(p, num, particle_pub);

		ros::spinOnce();
		loop_rate.sleep();
	}

	delete [] p;
	
	return 0;
}

