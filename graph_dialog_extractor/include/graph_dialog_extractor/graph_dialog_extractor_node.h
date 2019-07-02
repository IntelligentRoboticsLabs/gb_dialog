/*
 * grap_dialog_extractor_node.h
 *
 *  Created on: 1/07/2019
 *      Author: Jonathan Ginés
 */

#ifndef GRAPHDIALOGEXTRACTOR_H_
#define GRAPHDIALOGEXTRACTOR_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <gb_dialog/DialogInterface.h>
#include "bica_graph/graph_client.h"


namespace grap_dialog_extractor {


class GraphDialogExtractor: public gb_dialog::DialogInterface
{
public:
  GraphDialogExtractor();
  void step();

private:
	ros::NodeHandle nh_;
  bica_graph::GraphClient graph_;
  std::string saySplit(std::string str);
  bool listenExtractor();
};

}

#endif /* GRAPHDIALOGEXTRACTOR_H_ */
