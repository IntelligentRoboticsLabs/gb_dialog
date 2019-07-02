
#include <ros/ros.h>
#include <ros/console.h>
#include "bica_graph/graph_client.h"


class TestGraphDialogExtractor
{
public:
	TestGraphDialogExtractor(): nh_()
	{
    graph_.add_node("leia", "robot");
	  graph_.add_node("jack", "person");
		graph_.add_edge("leia", std::string("say:Hola, como estan mis vidas. Me escuchan, me oyeeen me sieenteen?"), "jack");
    graph_.add_edge("leia", std::string("listen"), "jack");
		ROS_INFO("[%s] inited", ros::this_node::getName().c_str());
	}

private:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_graph_extractor_dialog");
	TestGraphDialogExtractor testGraphDialogExtractor;
	return 0;
}
