
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <Eigen/Core>
#include "mstring/mstring.h"
#include "mmath/mmath.h"
#include "lf/node_edge_manager.h"

using namespace std;

NodeEdgeManager::NodeEdgeManager(string fname)
	: filename(fname), line_last{0, 0}
{
	marker_pub = n.advertise<visualization_msgs::Marker>("/edges/now", 1);
	loadCSV();
}

int NodeEdgeManager::loadCSV()
{
	Node node;
	Edge edge;
	ifstream ifs(filename);
	string str;
	vector<string> v;
	// int cnt = 0;

	if(ifs.fail()){
		cerr << "can't read " << filename << endl;
		return -1;
	}
	cout << "file name: " << filename << endl;

	while(getline(ifs, str)){
		if(!str.empty()){
			v = split(str, ' ');
			if(v[0] == "VERTEX"){
				node.x = atof(v[2].c_str());
				node.y = atof(v[3].c_str());
				nodes.push_back(node);
			}else if(v[0] == "EDGE"){
				edge.begin = atoi(v[1].c_str());
				edge.end = atoi(v[2].c_str());
				edge.orientation = atof(v[3].c_str());
				edges.push_back(edge);
				edge.begin = atoi(v[2].c_str());
				edge.end = atoi(v[1].c_str());
				edge.orientation += (edge.orientation > 0 ? -1 : 1) * M_PI;
				edges.push_back(edge);
			}else{
				cout << "cannot read:\n\t" << str << endl;
			}
			// cnt++;
		}
	}

	return 0;
}

double NodeEdgeManager::yawGetter(int edge_num)
{
	return edges[edge_num].orientation;
}

double NodeEdgeManager::yawCompGetter(int last, int now)
{
	Node begin = nodes[edges[last].end];
	Node end = nodes[edges[now].begin];

	return atan2(end.y - begin.y, end.x - begin.x);
}

double NodeEdgeManager::distGetter(int edge_num)
{
	double x1 = nodes[edges[edge_num].begin].x;
	double y1 = nodes[edges[edge_num].begin].y;
	double x2 = nodes[edges[edge_num].end].x;
	double y2 = nodes[edges[edge_num].end].y;
	return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}

double NodeEdgeManager::distGetter(int begin, int end)
{
	double x1 = nodes[begin].x;
	double y1 = nodes[begin].y;
	double x2 = nodes[end].x;
	double y2 = nodes[end].y;
	return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}

double NodeEdgeManager::distGetter(Node n1, Node n2)
{
	return (n2.x-n1.x)*(n2.x-n1.x) + (n2.y-n1.y)*(n2.y-n1.y);
}

double NodeEdgeManager::distGetter(int edge_num, double x, double y)
{
	double x1 = nodes[edges[edge_num].begin].x;
	double y1 = nodes[edges[edge_num].begin].y;
	return (x-x1)*(x-x1) + (y-y1)*(y-y1);
}

Node NodeEdgeManager::nodeGetter(int node_num)
{
	return nodes[node_num];
}
int NodeEdgeManager::edgeGetter(int edge_num, double yaw)
{
	double min = M_PI;
	int edge_new = edge_num;
	int node_new = edges[edge_num].end;
	double diff = 0;
	for(vector<Edge>::const_iterator it = edges.begin(); it != edges.end(); ++it){
		if(it->begin == node_new){
			diff = fabs(it->orientation - yaw);
			if(diff>M_PI) diff = 2*M_PI - diff;
			if(diff < min){
				min = diff;
				edge_new = it - edges.begin();
			}
		}
	}
	return edge_new;
}

Edge NodeEdgeManager::edgeGetter(int num_edge)
{
	return edges[num_edge];
}

int NodeEdgeManager::edgeGetter(int begin, int end)
{
	int edge_num = -1;
	for(vector<Edge>::const_iterator it = edges.begin(); it != edges.end(); ++it){
		if(it->begin == begin){
			if(it->end == end){
				edge_num = it - edges.begin();
				break;
			}
		}
	}

	if(!(edge_num + 1)){
		cout << "can't find EDGE{ " << begin << " -> " << end << " }" << endl;
		edge_num = 0;
	}

	return edge_num;
}

double NodeEdgeManager::xGetter(int edge_num)
{
	return nodes[edges[edge_num].end].x;
}

double NodeEdgeManager::yGetter(int edge_num)
{
	return nodes[edges[edge_num].end].y;
}

double NodeEdgeManager::yawsGetter(int lines_num)
{
	return yaws[lines_num];
}

double NodeEdgeManager::getLastLine()
{
	Node begin = nodes[edges[line_last[0]].begin];
	Node end = nodes[edges[line_last[1]].end];

	return atan2(end.y - begin.y, end.x - begin.x);
}

Node NodeEdgeManager::intersectionGetter(int lines_num)
{
	if(lines_num < 0){
		Node rtn = {0.0, 0.0};
		return rtn;
	}else{
		return intersection[lines_num];
	}
}

Node NodeEdgeManager::getLastIntersection()
{
	return nodes[edges[line_last[0]].begin];
}

void NodeEdgeManager::eraseIntersection(int lines_num)
{
	Node begin = {0, 0};
	if(lines_num){
		begin = intersection[lines_num - 1];
	}else{
		cout << "cannot find begin node" << endl;
	}
	if(lines_num < (int)intersection.size()){
		intersection.erase(intersection.begin() + lines_num);
		Node end = intersection[lines_num];
		yaws.erase(yaws.begin() + lines_num);
		yaws[lines_num] = atan2(end.y - begin.y, end.x - begin.x);
	}else{
		cout << "intersection size error" << endl;
	}
}

int NodeEdgeManager::isNext(int edge1, int edge2)
{
	int comp = -1; //間がない

	if(edges[edge1].end != edges[edge2].begin){
		for(int i=0; i < (int)edges.size(); i++){
			if(edges[edge1].end == edges[i].begin && edges[edge2].begin == edges[i].end){
				comp = i;
				break;
			}
		}
	}else{
		comp = -2; // 隣同士
	}

	return comp;
}

int NodeEdgeManager::yawsSetter(int edge_num)
{
	const double thr = M_PI / 7;
	static double last = 0;
	static int begin = 0;
	static int end = 0;
	double dist;
	double diff;
	double yaw_edge;
	double x1, x2, y1, y2;
	int next2;
//  && edges[last].begin != edges[edge_num].begin
	if(last - edge_num){
		next2 = isNext(last, edge_num);
		if(next2 == -1){
			diff = fabs(getLastLine() - yawCompGetter(last, edge_num));
		}else{
			if(next2 + 2){
				edge_num = next2; // 間補完
			}
			diff = fabs( edges[last].orientation - edges[edge_num].orientation );
		}
		if(diff > M_PI){
			diff = diff - 2*M_PI;
			diff = fabs(diff);
		}
		if(diff > M_PI / 2) diff = M_PI - diff;
		if(diff > thr){
			if(edges[last].begin != edges[edge_num].begin) end = last;
			x1 = nodes[edges[begin].begin].x;
			y1 = nodes[edges[begin].begin].y;
			x2 = nodes[edges[end].end].x;
			y2 = nodes[edges[end].end].y;
			dist = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
			// if( dist > pow(20, 2) ){
			if( dist > pow(10, 2) || (dist > pow(6, 2) && diff > thr*2 )){
				yaw_edge = atan2(y2-y1, x2-x1);
				yaws.push_back(yaw_edge);
				Node intrs = {nodes[edges[end].end].x, nodes[edges[end].end].y}; //交点にする
				// Node intrs = {nodes[edges[begin].begin].x, nodes[edges[begin].begin].y};
				intersection.push_back(intrs);
				line_last[0] = edge_num;
				begin = line_last[0];
			}
			// cout << "edge- line" << yaws.size() << " : " << yaw_edge*180/M_PI << endl;
			// begin = edge_num;
			cout << "--new line edge: " << edges[edge_num].begin << " -> " << edges[edge_num].end << endl;
		}else{
			end = last;
			cout << "--same line edge: " << edges[edge_num].begin << " -> " << edges[edge_num].end << endl;
		}
		line_last[1] = edge_num;
		last = line_last[1];
		cout << "--edge line size: " << yaws.size() << "\n" << endl;
	}

	return yaws.size();
}

void NodeEdgeManager::yawsReset(int begin, int end)
{
	int edge_num = edgeGetter(begin, end);
	line_last[0] = edge_num;
	line_last[1] = edge_num;
	yawsSetter(edge_num);
	yaws.clear();
	intersection.clear();
}

void NodeEdgeManager::setOnEdge(double &x, double &y, int edge_num)
{
	Eigen::Vector2d vec1, vec2;
	Node node_begin = nodes[edges[edge_num].begin];
	Node node_end = nodes[edges[edge_num].end];
	vec1 << x - node_begin.x, y - node_begin.y;
	vec2 << x - node_end.x, y - node_end.y;

	if(vec1.normalized().dot(vec2.normalized()) > 0){
		if(vec1.norm() < vec2.norm()){
			x = node_begin.x;
			y = node_begin.y;
		}else{
			// x = node_end.x;
			// y = node_end.y;
		}
	}
}

void NodeEdgeManager::showEdge(int edge_num)
{
	static int last = 0;

    visualization_msgs::Marker points, line_list;
    points.header.frame_id = line_list.header.frame_id = "/map";
    points.ns = line_list.ns = "/edge/now";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_list.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

	// points.color.r = 1.0f;
	// points.color.g = 1.0f;
	points.color.b = 1.0f;
	points.color.a = 1.0;

	line_list.color.g = 1.0f;
	line_list.color.a = 1.0;

	line_list.scale.x = 2.1;

	points.header.stamp = line_list.header.stamp = ros::Time::now();

	geometry_msgs::Point p;
	if(edge_num + 1){
		last = edge_num;
		points.scale.x = 2.1;
		points.scale.y = 2.1;

		p.x = nodes[edges[edge_num].begin].x;
		p.y = nodes[edges[edge_num].begin].y;
		p.z = -0.15;
		points.points.push_back(p);
		line_list.points.push_back(p);
		p.x = nodes[edges[edge_num].end].x;
		p.y = nodes[edges[edge_num].end].y;
		p.z = -0.15;
		points.points.push_back(p);
		line_list.points.push_back(p);
	}else{
		points.scale.x = 4.5;
		points.scale.y = 4.5;
		p.x = nodes[edges[last].end].x;
		p.y = nodes[edges[last].end].y;
		p.z = -0.15;
		points.points.push_back(p);
	}
	marker_pub.publish(points);
	marker_pub.publish(line_list);

}

