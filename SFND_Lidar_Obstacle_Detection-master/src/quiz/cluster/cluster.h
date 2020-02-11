#ifndef CLUSTER_H
#define CLUSTER_H

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include <unordered_set>

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points);

void clusterHelper(std::vector<bool>& processedPoints,
    const std::vector<std::vector<float>>& points,
    KdTree* tree, float distanceTol,
    std::vector<int>& cluster,
    int index);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif
