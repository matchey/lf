
#ifndef LOCALIZATION_NODE_EDGE_MANAGER_H
#define LOCALIZATION_NODE_EDGE_MANAGER_H

struct Node
{
	double x;
	double y;
};

struct Edge
{
	int begin;
	int end;
	double orientation;
};

class NodeEdgeManager
{
	ros::NodeHandle n;
	ros::Publisher marker_pub;
	const std::string filename;
	std::vector<Node> nodes;
	std::vector<Edge> edges;
	std::vector<double> yaws;
	int line_last[2];
	std::vector<Node> intersection;
	int loadCSV();
	int isNext(int, int); // edge_num, edge_num
	public:
	NodeEdgeManager(std::string);
	double yawGetter(int);
	double yawCompGetter(int, int); // edge, edge
	double distGetter(int); // edge_num
	double distGetter(int, int); // node1, node2
	double distGetter(Node, Node); // node1, node2
	double distGetter(int, double, double); // edge_num, x, y
	Node nodeGetter(int); //node_num
	Edge edgeGetter(int); //num_edge
	int edgeGetter(int, double); //yaw
	int edgeGetter(int, int); //node1, node2
	double xGetter(int);
	double yGetter(int);
	double yawsGetter(int); // lines_num
	double getLastLine(); // lines_num
	Node intersectionGetter(int); // lines_num
	Node getLastIntersection();
	void eraseIntersection(int); // lines_num
	int yawsSetter(int); // edge_num
	void yawsReset(int, int); // node1, node2
	void setOnEdge(double&, double&, int); // x, y, edge
	void showEdge(int); // edge_num
};

#endif

