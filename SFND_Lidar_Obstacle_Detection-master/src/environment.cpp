/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidarObj = new Lidar(cars, 0);

    // Create a point cloud by calling scan method on the lidar
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidarObj->scan();

    // Render the pointCloud using the lidar position

    // renderRays(viewer, lidarObj->position, pointCloud);
    renderPointCloud(viewer, pointCloud, "InputPointCloud");


    // Create point processor  
    ProcessPointClouds<pcl::PointXYZ>* pointProcessorObj 
        = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessorObj->KK_Custom_SegmentPlane(pointCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorObj->KK_Custom_Clustering(segmentCloud.first, 1., 3, 80);
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        // render cluster point cloud
        renderPointCloud(viewer, cluster, "obstacle_cloud " + std::to_string(clusterId), colors[clusterId]);

        // render box
        Box box = pointProcessorObj->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId], 1);

        clusterId++;
    }
}

/********************************************************************************************************************
    [KK]: 10.02.2020

    cityBlock method received 3D points with intensity values along with the point processor object

    It does the following:
        1. Cloud filtering using the voxel grid point reduction and region based filtering.
        2. KK_Custom_SegmentPlane method is called on the filtered cloud from step 1. It provides the plane and obstables PC.
        4. Rendor the plane ( other than obstacles ) in GREEN.
        3. KK_Custom_Clustering method is called on the obstables point cloud for clustering them.
        4. Rendor the obstacles in BLUE, YELLOW, RED and CYAN
        5. Rendox a bounding box in RED around the obstacles.

********************************************************************************************************************/

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorObj, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorObj = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorObj->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // Set the hyperparameters for cloud filtering
    Eigen::Vector4f minPoint(-20, -6, -3, 1);
    Eigen::Vector4f maxPoint(25, 6.5, 3, 1);

    // Experiment with the ? values and find what works best
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorObj->FilterCloud(inputCloud, 0.15, minPoint, maxPoint);
    
    //renderPointCloud(viewer, filterCloud, "filterCloud");
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg_result_pair = pointProcessorObj->SegmentPlane(filterCloud, 100, 0.3);
    // renderPointCloud(viewer, segmentCloud.first, "obstacle");
    
    // Low iteration helps in faster processing ( e.g. > 50 too much time )
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorObj->KK_Custom_SegmentPlane(filterCloud, 25, 0.3);
    
    // Render the plane in GREEN
    renderPointCloud(viewer, segmentCloud.second, "plane", Color(0, 1, 0));
    //renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorObj->KK_Custom_Clustering(segmentCloud.first, .6, 20, 5000);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(0,1,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorObj->numPoints(cluster);
        // Render the Cluster point cloud: Green/Blue/Yellow
        renderPointCloud(viewer, cluster, "obstacle_cloud " + std::to_string(clusterId), colors[clusterId % colors.size()]);

        // Create the bounding box
        Box box = pointProcessorObj->BoundingBox(cluster);
        // Bounding box: Red Color
        renderBox(viewer, box, clusterId, Color(1,0,0), 1);

        clusterId++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}