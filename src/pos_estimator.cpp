
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense> // for EigenSolver
#include <fstream>

#include "ceres_msgs/AMU_data.h"
#include "ceres_msgs/GPSFix.h"
#include "pf/particle.h"
#include "lf/node_edge_manager.h"
#include "lf/pos_estimator.h"
#include "voronoi/NeGridMapCreator.h"
#include "mmath/mmath.h"
#include "mstring/mstring.h"

using namespace std;

random_device PosEstimator::rnd;

PosEstimator::PosEstimator()
	:x(0.0), y(0.0), cnt_yaw(0),
	flag_correction(0), flag_clear(false), flag_turn(false), flag_ignore(false),
	odom_correction(Eigen::Affine2d::Identity()), flag_finish(false),
	forward_noise(1.45), mu(0.255), mt(rnd()), error(mu, forward_noise)
{
	string filename;
	string topic_odom;
	int init_edge = 0;
	n.getParam("/node_edge", filename);
	n.param<string>("/topic_name/odom_complement", topic_odom, "/odom/complement");
	nem = new NodeEdgeManager(filename);
	ngmc = new NeGridMapCreator(filename);
	x_init = 0.0;
	y_init = 0.0;
	yaw_init = nem->yawGetter(init_edge);
	yaw_correction = 0; //nem->yawGetter(init_edge);
	yaw = yaw_init;
	setIgnoreEdge();

	odom_sub = n.subscribe<nav_msgs::Odometry>(topic_odom, 1, &PosEstimator::odomCallback, this);
	gps_sub = n.subscribe<ceres_msgs::GPSFix>("/ceres/gps", 1, &PosEstimator::gpsCallback, this);
	ff_sub = n.subscribe<std_msgs::Bool>("/flag/moment", 1, &PosEstimator::ffCallback, this);
	// ff_sub = n.subscribe<std_msgs::Bool>("/flag/finish", 1, &PosEstimator::ffCallback, this);
	
	pos_pub = n.advertise<nav_msgs::Odometry>("/odom/particle", 50);
	corrected_pub = n.advertise<geometry_msgs::PoseArray>("/odom/corrected", 1);
	odom_pub = n.advertise<nav_msgs::Odometry>("/odom/lfl", 1);

	lcl_pub = n.advertise<nav_msgs::Odometry>("/lcl", 1); // for sawahashi
	flag_diff_pub = n.advertise<std_msgs::Float64>("/flag/diff", 1); // for sawahashi
	edge_pub = n.advertise<std_msgs::Int16MultiArray>("/edge/certain", 1); // for sawahashi
}

PosEstimator::~PosEstimator()
{
	// delete nem;
}

void PosEstimator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double __x = msg->pose.pose.position.x;
	double __y = msg->pose.pose.position.y;
	Eigen::Vector3d v(1, 1, 1);

	v(0) = __x*cos(yaw_init) - __y*sin(yaw_init) + x_init;
	v(1) = __x*sin(yaw_init) + __y*cos(yaw_init) + y_init;

	v = odom_correction * v;

	x = v(0);
	y = v(1);

	double roll, pitch;
	tf::Quaternion q(msg->pose.pose.orientation.x,
					 msg->pose.pose.orientation.y,
					 msg->pose.pose.orientation.z,
					 msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	yaw += yaw_correction + yaw_init;
	if(yaw> M_PI) yaw = yaw - 2*M_PI;
	if(yaw<-M_PI) yaw = yaw + 2*M_PI;
}

void PosEstimator::gpsCallback(const ceres_msgs::GPSFix::ConstPtr& msg)
{
	lat = msg->lat;
	lng = msg->lon;
	const double r = 6378137;
	const double lat0 = 36.080109 * M_PI/180; //36.080196, 140.115588
	const double lng0 = 140.115647 * M_PI/180;
	lat*=M_PI/180;
	lng*=M_PI/180;
	static double last_x =0.0;
	static double last_y =0.0;
	double gps_x = r * (lng-lng0) * cos( (lat0+lat)/2 );
	double gps_y = r * (lat-lat0);
	if(fabs(last_x - gps_x) > 1e-5 || fabs(last_y - gps_y) > 1e-5){
		// cout << "(x, y) = (" << gps_x;
		// cout << ", " << gps_y << ")" << endl;
		// x = gps_x;
		// y = gps_y;
	}
	last_x = gps_x;
	last_y = gps_y;
}

void PosEstimator::ffCallback(const std_msgs::Bool::ConstPtr& msg)
{
	corrector_tentatively();
	for(auto line = lines.begin(); line != lines.end(); ++line){
		for(auto itr = line->begin(); itr != line->end(); ++itr){
			odomPublisher(itr->x, itr->y);
		}
	}
	flag_finish = msg->data;
	odomPublisher(0, 0);
	flag_finish = false;
}

void PosEstimator::setNoise(double noise){forward_noise = noise; setGauss();}

void PosEstimator::setMu(double _mu){mu = _mu; setGauss();}

void PosEstimator::setGauss(){error.param(normal_distribution<>::param_type(mu, forward_noise));}

void PosEstimator::setIgnoreEdge()
{
	string filename = "ignore_edge_filename";
	n.getParam("/ignore_edge", filename);
	ifstream ifs(filename);
	string str;
	vector<string> v;

	if(ifs.fail()){
		cerr << "can't read " << filename << endl;
		return;
	}

	while(getline(ifs, str)){
		if(!str.empty()){
			v = split(str, ' ');
			ignore_edge.push_back(nem->edgeGetter(atoi(v[1].c_str()), atoi(v[2].c_str())));
		}
	}
}

void PosEstimator::setParticlePose(int begin, int end, double div, Particle &p)
{
	int edge_num = nem->edgeGetter(begin, end);
	double dist_p = nem->distGetter(edge_num) * div * div;

	p.x_new = nem->nodeGetter(begin).x;
	p.y_new = nem->nodeGetter(begin).y;
	p.setInit(p.x_new, p.y_new);
	p.setXY(p.x_new, p.y_new);
	p.dist_r_last = p.distSquared(x, y);
	p.edge_last = -1;
	p.setEdge(edge_num);
	p.move(sqrt(dist_p), nem->yawGetter(edge_num));
	p.flag = false;
}

void PosEstimator::setRobotPose(int begin, int end, double div)
{
	int edge_num = nem->edgeGetter(begin, end);
	double dist = sqrt(nem->distGetter(edge_num)) * div;

	yaw_init = nem->edgeGetter(edge_num).orientation;
	x_init = nem->nodeGetter(begin).x + dist * cos(yaw_init) - x;
	y_init = nem->nodeGetter(begin).y + dist * sin(yaw_init) - y;
	yaw_correction = 0.0;

	 x  = x_init;
	 y  = y_init;
	yaw = yaw_init;

	nem->yawsReset(begin, end);
}

void PosEstimator::setParticle2nearEdge(int uniq, Particle &p) //p[0]
{
	static int cnt_no = 0;
	static int cnt_node = 0;

	if(cnt_node > 300) uniq = 0; //20[s] nodeに居続けたら
	if(uniq + 1){
		vector<int> v = ngmc->getNearEdge(x, y, yaw);
		if(v.size()){
			auto itr = find(v.begin(), v.end(), uniq);
			if( itr == v.end() ) {
				double dist_min = nem->distGetter(*v.begin(), x, y);
				int edge_num = *v.begin();
				double dist = 0.0;
				for(auto it = v.begin() + 1; it != v.end(); ++it){
					dist = nem->distGetter(*it, x, y);
					if(dist < dist_min){
						dist_min = dist;
						edge_num = *it;
					}
				}
				int begin = nem->edgeGetter(edge_num).begin;
				int end = nem->edgeGetter(edge_num).end;
				double div = dist_min / nem->distGetter(edge_num);
				setParticlePose(begin, end, div, p);
				cout << "---" << endl;
				cout << begin << ", " << end << ", " << div << endl;
				cout << "---\n" << endl;
			}
			cnt_no = 0;
		}else{
			cnt_no++;
			printf("\033[%dA" , 1);
			cout << "MAP DOESN'T HAVE THIS ROAD (" << cnt_no << ")"<< endl;
		}
		cnt_node = 0;
	}else{
		cnt_node++;
	}
}

void PosEstimator::particleManager(Particle &p) // for in p[]
{
	double p_x, p_y;
	int edge_num = p.getEdge();
	double dist_p = p.getDist();
	double dist_r = p.distSquared(x, y);
	double orientation_p = nem->yawGetter(edge_num);
	double dist_edge = nem->distGetter(edge_num);

	// double linear = (sqrt(dist_r) - sqrt(p.dist_r_last))*( 1 + error(mt));
	double linear = (sqrt(dist_r) - sqrt(p.dist_r_last));
	if(fabs(linear) < 0.5) linear *=  (1 + error(mt));

	p.move( linear, orientation_p );
	p_x = p.getX();
	p_y = p.getY();
	nem->setOnEdge(p_x, p_y, edge_num);
	p.setXY(p_x, p_y);
	p.dist_r_last = dist_r;

	if(p.flag){ // node付近にいるとき
		if(dist_p > 0.4){ //nodeから少し進んだら
			p.edge_last = edge_num;
			edge_num = nem->edgeGetter(edge_num, yaw);
			p.setEdge(edge_num);
			p.setXY(p.x_new, p.y_new);
			p.move(sqrt(dist_p)*( 1 + error(mt)), nem->yawGetter(edge_num));
			p.flag = false;
		}
	}else{
		p.measurement_prob(yaw);
		if(dist_p < 0.1){ //前のnodeに戻ってきたとき
			if(p.edge_last+1){ //start判定
				p.flag = true;
				p.setEdge(p.edge_last);
				p.setXY(p.x_new, p.y_new);
				dist_p = 0.0;
				p.dist_r_last = p.distSquared(x, y);
			}
		}
		if( dist_edge < dist_p ){ //次のnodeに到着
			p.flag = true;
			p.x_new = nem->xGetter(edge_num);
			p.y_new = nem->yGetter(edge_num);
			p.setInit(p.x_new, p.y_new);
			p.setXY(p.x_new, p.y_new);
			dist_p = 0.0;
			p.dist_r_last = p.distSquared(x, y);
		}
	}
	if(flag_correction){
		p.measurement_prob(x, y);
	}
}

void PosEstimator::globalPCA()
{
	static vector<Point2D> line;
	static int line_cnt = 0;
	static bool flag = false;
	Point2D point;
	const double thr = 1e-2; // threshold of curvature
	Eigen::Vector2d slope;
	static Eigen::Vector2d last(cos(yaw), sin(yaw));
	static double yaw_last;
	static bool flag_reset = false;
	if(!flag_ignore && flag_reset){ flag = false; }
	flag_reset = flag_ignore;

	if(flag_clear || flag_turn){ flag = false; flag_turn = false; }
	if( localPCA() > thr || (line_cnt > 390)){
		if(line_cnt > 80){
			if(flag){
				slope = getLine(line);
				double diff = acos(slope.dot(last));
				if(diff > M_PI/2) diff = M_PI - diff;
				if(diff > M_PI/5.5){ // const |vec| = 1;
					if(getDist(line) > 3.0){
						if(diff > M_PI / 3.5 || getDist(line) > 8.6){
							lines.push_back(line);
							last = slope;
						}else{
							copy(line.begin(), line.end(), back_inserter(lines.back()));
							last = getLine(lines.back());
						}
					}
				}else{
					if(isTurn(yaw_last, yaw)){ yaw_last = yaw; flag_clear = true;  flag_turn = true; return; }
					copy(line.begin(), line.end(), back_inserter(lines.back()));
					last = getLine(lines.back());
					if(!flag_correction && getDist(lines.back()) > 100){
						corrector_tentatively();
					}
					yaw_last = yaw;
				}
			}else{ // first (edge:0) only
				if(getDist(line) > 8.6){
					last = getLine(line);
					lines.push_back(line);
					yaw_last = yaw;
					flag = true;
				}
			}
			cout << lines.size() << endl;
		}
		line.clear();
		line_cnt = 0;
	}else{
		point.x = x; point.y = y;
		line.push_back(point);
		line_cnt++;
	}
}

void PosEstimator::onEdgeManager(int uniq)
{
	if(flag_correction) flag_correction--;

	// vector<int> v = ngmc->getNearEdge(x, y, yaw);

	if(uniq + 1){
		cnt_yaw = nem->yawsSetter(uniq);
		// if(v.size()){
		// 	auto itr = find(v.begin(), v.end(), uniq);
		// 	if( itr == v.end() ) {
		// 		cout << "NO EDGE!!!!" << endl;
		// 		// cout << "(x, y) = (" << x << ", " << y << ")" << endl;
		// 		cout << "---" << endl;
		// 		for(auto it = v.begin(); it != v.end(); ++it){
		// 			cout << *it << ": " <<
		// 				nem->edgeGetter(*it).begin << "->" << nem->edgeGetter(*it).end << endl;
		// 		}
		// 		cout << "---\n" << endl;
		// 	}
		// }
		Edge edge_certain = nem->edgeGetter(uniq);
		double dist_r = nem->distGetter(uniq, x, y);
		double div = sqrt(dist_r / nem->distGetter(uniq));
		if(div > 1.0) div = 1.0;
		std_msgs::Int16MultiArray edge_now;
		edge_now.data.push_back(edge_certain.begin);
		edge_now.data.push_back(edge_certain.end);
		edge_now.data.push_back(int(100.0*div));
		edge_pub.publish(edge_now);

	}

	nem->showEdge(uniq);
	// if(v.size()){
	// 	cout << "---" << endl;
	// 	for(auto it = v.begin(); it != v.end(); ++it){
	// 		cout << *it << ": " <<
	// 			nem->edgeGetter(*it).begin << "->" << nem->edgeGetter(*it).end << endl;
	// 	}
	// 	cout << "---\n" << endl;
	// }
	corrector();
	// ignoreEdge(uniq);

	if(uniq + 1){
		if(flag_clear){
			cout << "\n\n\nclear..." << endl;
			int begin = nem->edgeGetter(uniq).begin;
			int end = nem->edgeGetter(uniq).end;
			nem->yawsReset(begin, end);
			cnt_yaw = 0;
			lines.clear();
			flag_turn = true;
			flag_clear = false;
		}
	}
}

double PosEstimator::localPCA() // solverよりSVDのほうが安定する? 変わらん?
{
	// const int N = 20; //for d_in
	const int N = 37; // pcaに使うposの個数
	static Point2D pos[N];
	static int cnt = 0;
	static double x_sum = 0.0;
	static double y_sum = 0.0;
	static double prodsum_xy = 0.0;
	static double prodsum_x  = 0.0;
	static double prodsum_y  = 0.0;
	double x_ave = 0.0;
	double y_ave = 0.0;
	double cov = 0.0;
	Eigen::Matrix2d vcov;
	double curv = 100;

	x_sum -= pos[cnt].x;
	y_sum -= pos[cnt].y;
	prodsum_xy -= pos[cnt].x * pos[cnt].y;
	prodsum_x  -= pos[cnt].x * pos[cnt].x;
	prodsum_y  -= pos[cnt].y * pos[cnt].y;


	pos[cnt].x = x;
	pos[cnt].y = y;

	x_sum += pos[cnt].x;
	y_sum += pos[cnt].y;
	prodsum_xy += pos[cnt].x * pos[cnt].y;
	prodsum_x  += pos[cnt].x * pos[cnt].x;
	prodsum_y  += pos[cnt].y * pos[cnt].y;

	x_ave = x_sum / N;
	y_ave = y_sum / N;

	cov = prodsum_xy / N - x_ave * y_ave;

	vcov << prodsum_x / N - x_ave * x_ave, cov,
		    cov, prodsum_y / N - y_ave * y_ave;

	Eigen::EigenSolver<Eigen::Matrix2d> es(vcov);
	
	Eigen::Vector2d values = es.eigenvalues().real();
	
	double lambda_min = values(0) < values(1) ? values(0): values(1);
	if(values(0) + values(1)) curv = 2.0 * lambda_min / ( values(0) + values(1) );

	cnt = (cnt+1)%N;

	return curv;
}

void PosEstimator::fit_odom(int start, const Eigen::Affine2d &correction)
{
	// odomPublisher(0, 0);
	Eigen::Vector3d vec;

	for(auto line = lines.begin() + start; line != lines.end(); ++line){
		for(auto itr = line->begin(); itr != line->end(); ++itr){
			vec << itr->x, itr->y, 1;
			vec = correction * vec;
			itr->x = vec(0);
			itr->y = vec(1);
			// odomPublisher(vec(0), vec(1));
		}
	}
}

void PosEstimator::calcCorrection(const int &count, double &ratio, double &yaw_diff, Eigen::Affine2d &diff_correction)
{
	// Eigen::Vector2d slope = getLine(lines[count]);
	Eigen::Vector2d slope;
	double curv;
	getLine(lines[count], slope, curv);
	cout << "curv: " << curv << endl;
	double yaw_odom = atan2(slope(1), slope(0));
	double yaw_edge = nem->yawsGetter(count);

	yaw_diff =  yaw_edge - yaw_odom;
	thetaAround(yaw_diff); theta2slope(yaw_diff);
	cout << "odom :  " << yaw_odom * 180 / M_PI << endl;
	cout << "edge :  " << yaw_edge * 180 / M_PI << endl;
	cout << "diff :  " << yaw_diff * 180 / M_PI << endl;

	Eigen::Vector2d intersection = getIntersection(lines[count], lines[count+1]);
	Node node_corner = nem->intersectionGetter(count);

	cout << "odom coner: (" << intersection(0) << ", " << intersection(1) << ")" << endl;
	cout << "edge coner: (" <<  node_corner.x  << ", " <<  node_corner.y  << ")" << endl;

	double dist_odom = getDist(lines[count]);
	double dist_edge = sqrt(nem->distGetter(nem->intersectionGetter(count-1), node_corner));
	if(count){
		ratio = dist_odom / dist_edge;
	}else{
		ratio = 1.0;
	}

	const double w0 = 0.005;
	const double r  = 0.35;
	double weight = 1 / (1 + ((1-w0)/w0)*exp(-r*dist_odom));

	cout << "dist_odom: " << dist_odom << endl;
	cout << "dist_edge: " << dist_edge << endl;
	cout << "  ratio  : " << ratio << ", dist_diff = "<< dist_edge - dist_odom << endl;
	cout << "  count  : " << count << endl;
	cout << "  weight : " << weight<< endl;


	Eigen::Rotation2D<double> rot(weight * yaw_diff);
	Eigen::Translation<double, 2> trans1, trans2;

	double x_diff = weight * (node_corner.x - intersection(0));
	double y_diff = weight * (node_corner.y - intersection(1));
	trans1 = Eigen::Translation<double, 2>(-intersection(0), -intersection(1));
	trans2 = Eigen::Translation<double, 2>(intersection(0) + x_diff, intersection(1) + y_diff);
	// trans2 = Eigen::Translation<double, 2>(node_corner.x, node_corner.y);
	// yaw_correction += yaw_diff;
	diff_correction = trans2 * rot * trans1;
	if( fabs(ratio - 1.0) < 0.3 && x_diff*x_diff + y_diff*y_diff > pow(25, 2) ){
		cout << "diff over 25 [m]" << endl;
		yaw_diff = M_PI;
	}
}

void PosEstimator::corrector()
{
	static int count = 0;
	// static int erased = 0;

	if(flag_ignore || flag_clear){ count = 0; return; }

	if( cnt_yaw > count && (int)lines.size() > count+1){
		cout << "\n\n===========correct=========== " << count << endl;
		double ratio;
		double diff;
		Eigen::Affine2d diff_correction;

		calcCorrection(count, ratio, diff, diff_correction);

		// cout << "matrix :     \n" << diff_correction.matrix() << endl;
		// cout << "linear :     \n" << diff_correction.linear() << endl;
		// cout << "\ntranslation :\n" << diff_correction.translation() << endl;

		if(ratio > 1.10){
			nem->eraseIntersection(count);
			cnt_yaw--;
			cout << "removed short edge\n\n" << endl;
		}else if(ratio < 0.70){
			lines.erase(lines.begin() + count);
			cout << "erased " << count << "'s line\n\n" << endl;
		}else{
			if(fabs(diff) < M_PI / 15){
				odom_correction = diff_correction * odom_correction;
				fit_odom(count, diff_correction);

				std_msgs::Float64 diff_yaw;
				diff_yaw.data = diff;
				flag_diff_pub.publish(diff_yaw);

				cout << "\ncorrected\n\n" << endl;
				flag_correction = 80;

				count++;
			}else{
				count = 0;
				flag_clear = true;
				cout << "\ncorrection RESET\n\n" << endl;
			}
		}
	}
}

void PosEstimator::corrector_tentatively()
{
	if(flag_ignore){
		return;
	}

	cout << "correct tentatively" << endl;

	// Eigen::Vector2d slope = getLine(lines.back());
	Eigen::Vector2d slope;
	double curv;
	getLine(lines.back(), slope, curv);
	cout << "curv: " << curv << endl;

	double diff = nem->getLastLine() - atan2(slope(1), slope(0));
	thetaAround(diff);
	theta2slope(diff);

	cout << "diff :  " << diff * 180 / M_PI << endl;

	if(fabs(diff) < M_PI/12){
		Eigen::Vector2d intersection;
		Node node_corner = nem->getLastIntersection();
		if(lines.size() > 1){
			intersection = getIntersection(*(lines.end()-2), lines.back());
		}else{
			intersection = {0.0, 0.0};
		}

		cout << "edge coner: (" <<  node_corner.x  << ", " <<  node_corner.y  << ")" << endl;
		cout << "odom coner: (" << intersection(0) << ", " << intersection(1) << ")" << endl;

		Eigen::Rotation2D<double> rot(diff);
		Eigen::Translation<double, 2> trans1;
		Eigen::Translation<double, 2> trans2;
		trans2 = Eigen::Translation<double, 2>(node_corner.x, node_corner.y);

		double dist = 0.0;
		if(intersection(0) || intersection(1)){
			trans1 = Eigen::Translation<double, 2>(-intersection(0), -intersection(1));
			dist = pow(node_corner.x-intersection(0), 2) + pow(node_corner.y-intersection(1), 2);
		}else{
			trans1 = Eigen::Translation<double, 2>(-node_corner.x, -node_corner.y);
		}
		if(dist < pow(10, 2)){
			yaw_correction += diff;
			Eigen::Affine2d diff_correction(trans2 * rot * trans1);
			odom_correction = diff_correction * odom_correction;
			fit_odom(lines.size() - 1, diff_correction);

			std_msgs::Float64 diff_yaw;
			diff_yaw.data = diff;
			flag_diff_pub.publish(diff_yaw);

			cout << "corrected tentatively" << endl;
			// flag_correction = 500; // for d_in
			flag_correction = 1500;
		}
	}
}

void PosEstimator::ignoreEdge(int uniq)
{
	static int ignore_cnt = 0;
	static int ignore_ready = 0;
	if((int)ignore_edge.size() == ignore_cnt){
		return;
	}
	if(!(uniq + 1)){
		if(ignore_ready && ignore_ready < (int)lines.size()){
			cout << "ignore begin" << endl;
			flag_ignore = true;
			ignore_ready = 0;
			return;
		}else{
			return;
		}
	}
	if(!ignore_ready){
		if(!(ignore_cnt%2)){ // begin
			if(uniq == ignore_edge[ignore_cnt]){
				cout << "ignore ready" << endl;
				ignore_cnt++;
				ignore_ready = lines.size();
			}
		}else{ // end
			if(uniq == ignore_edge[ignore_cnt]){
				cout << "ignore end" << endl;

				for(auto line = lines.begin(); line != lines.end(); ++line){
					for(auto itr = line->begin(); itr != line->end(); ++itr){
						odomPublisher(itr->x, itr->y);
					}
				}

				int begin = nem->edgeGetter(uniq).begin;
				int end = nem->edgeGetter(uniq).end;
				nem->yawsReset(begin, end);
				cnt_yaw = 0;
				lines.clear();
				flag_ignore = false;
				ignore_cnt++;
			}
		}
	}
}

void PosEstimator::posPublisher(Particle p)
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(p.getYaw());

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = p.getX();
    odom.pose.pose.position.y = p.getY();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    pos_pub.publish(odom);
}

void PosEstimator::tinyPublisher()
{
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "matching_base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tiny_broadcaster.sendTransform(odom_trans);
	
	////////////////////////////////localization_test/////////////////////
    // tf::Transform odom_trans;
	// odom_trans.setOrigin(tf::Vector3(x, y, 0.0));
	// tf::Quaternion q;
	// q.setRPY(0, 0, yaw);
	// odom_trans.setRotation(q);
    // tiny_broadcaster.sendTransform(tf::StampedTransform(odom_trans, ros::Time::now(), "map", "matching_base_link"));
	//////////////////////////////////////////////////////////////////////

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    // odom.child_frame_id = "matching_base_link";
    // odom.twist.twist.linear.x = vx;
    // odom.twist.twist.linear.y = vy;
    // odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    odom.pose.pose.orientation.z = yaw;
    lcl_pub.publish(odom); // q.z <- yaw
}

void PosEstimator::odomPublisher(double odom_x, double odom_y)
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

	static geometry_msgs::PoseArray odoms;

	geometry_msgs::Pose odom;
    odoms.header.stamp = ros::Time::now();
    odoms.header.frame_id = "map";
    odom.position.x = odom_x;
    odom.position.y = odom_y;
    odom.position.z = 0.0;
    odom.orientation = odom_quat;

	odoms.poses.insert(odoms.poses.begin(), odom);
    if(flag_finish){
		// cout << "pub " << odoms.poses.size() << "odoms" << endl;
		corrected_pub.publish(odoms);
	}
    // corrected_pub.publish(odoms);

	if(!odom_x && !odom_y){
		odoms.poses.clear();
	}
}

