/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

    while (maxIterations--)
    {
        // Randomly sample subset and fit line

        std::unordered_set<int> set_inliers;

        // Get a random sample
        while (set_inliers.size() < 3)
            set_inliers.insert(rand() % cloud->points.size());

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = set_inliers.begin();

        // First sample coordinates
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        
        // Fetch the next element
        itr++;

        // Second sample coordinates
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        
        // Fetch the next element
        itr++;

        // Third sample coordines
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Calculate vector v1 and v2 to the planes computed using the points above
        //pcl::PointXYZ v1 = (x2-x1, y2-y1, z2-z1);
        //pcl::PointXYZ v2 = (x3-x1, y3-y1, z3-z1);

        // Find a normal vector to the plane using the cross product v1 X v2
        /* pcl::Point v1Xv2 = ((y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1), 
            (z2 - z1)(x3 - x1) - (x2 - x1)(z3 - z1), 
            (x2 - x1)(y3 - y1) - (y2 - y1)(x3 - x1));
        */

        float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

        // Fit a plane in the above points
        float A = i;
        float B = j;
        float C = k;
        float D = -(i*x1 + j*y1 + k*z1);

        // Measure distance between every point and fitted line

        int index = 0;

        for (auto elem : cloud->points)
        {
            float distance = abs(A * elem.x + B * elem.y + C * elem.z + D) / sqrt(A * A + B * B + C * C);

            // Check for the two points set above, if present ignore
            if (set_inliers.count(index) > 0)
                continue;

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceTol)
                set_inliers.insert(index);

            index++;
        }

        if (set_inliers.size() > inliersResult.size())
            inliersResult = set_inliers;
        // Return indicies of inliers from fitted line with most inliers
    }
	return inliersResult;

}

// RANSAC to fit only a line

std::unordered_set<int> RansacForLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations 

    while (maxIterations--)
    {
        // Randomly sample subset and fit line

        std::unordered_set<int> set_inliers;

        // Get a random sample
        while (set_inliers.size() < 2)
            set_inliers.insert(rand() % cloud->points.size());

        float x1, y1, x2, y2;
        auto itr = set_inliers.begin();

        // First sample coordinates
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;

        // Fetch the next element
        itr++;

        // Second sample coordinates
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        // Fit the line in the above points
        float A = y1 - y2;
        float B = x2 - x1;
        float C = (x1 * y2) - (x2 * y1);

        // Measure distance between every point and fitted line

        int index = 0;

        for (auto elem : cloud->points)
        {
            float distance = abs(A * elem.x + B * elem.y + C) / sqrt(A * A + B * B);

            // Check for the two points set above, if present ignore
            if (set_inliers.count(index) > 0)
                continue;

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceTol)
                set_inliers.insert(index);

            index++;
        }

        if (set_inliers.size() > inliersResult.size())
            inliersResult = set_inliers;
        // Return indicies of inliers from fitted line with most inliers
    }
    return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = RansacForLine(cloud, 100, 0.5);
    std::unordered_set<int> inliers = Ransac(cloud, 4000, 0.6);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
