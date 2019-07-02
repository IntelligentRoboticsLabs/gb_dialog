#include "graph_dialog_extractor/graph_dialog_extractor_node.h"

namespace grap_dialog_extractor {

	GraphDialogExtractor::GraphDialogExtractor(){}

  std::string GraphDialogExtractor::saySplit(std::string str)
  {
    std::string delimiter = ":";
    size_t token = 0;
    token = str.find(delimiter, 0);
    str.erase(0, str.find(delimiter) + delimiter.length());
    return str;
  }

  bool GraphDialogExtractor::listenExtractor()
  {
    if (getCallTime() + ros::Duration(2) < ros::Time::now() )
    {
      ROS_INFO("[Listen] ...");
      setCallTime(ros::Time::now());
      listen();
      return true;
    }
    return false;
  }

  void GraphDialogExtractor::step()
  {
    auto edges = graph_.get_edges();
    for (auto it_edges = edges.begin(); it_edges != edges.end(); ++it_edges)
      for (auto it_edge_type = it_edges->second.begin(); it_edge_type != it_edges->second.end(); ++it_edge_type)
        if ((*it_edge_type)->get_type() == bica_graph::STRING)
        {
          std::string edge = std::dynamic_pointer_cast<bica_graph::Edge<std::string>>(*it_edge_type)->get();
          if (edge.find("say") != std::string::npos)
          {
            std::string say_splited_edge = saySplit(edge);
            ROS_INFO("[Say] %s", say_splited_edge.c_str());
            speak(say_splited_edge);
            graph_.remove_edge("leia", "jack", edge);
          }
          else if (edge.find("listen") != std::string::npos)
          {
            if (listenExtractor())
              graph_.remove_edge("leia", "jack", edge);
          }
        }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grap_dialog_extractor_node");
  ros::NodeHandle n;
  grap_dialog_extractor::GraphDialogExtractor gExtractorNode;

  ros::Rate loop_rate(5);
	while(ros::ok())
	{
		gExtractorNode.step();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
