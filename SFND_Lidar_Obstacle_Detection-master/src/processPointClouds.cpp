// PCL lib Functions for processing point clouds 
#include "processPointClouds.h"
#include <pcl/filters/voxel_grid.h>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


/*
    [KK]: 10.02.2020
    This method does voxel grid point reduction and region based filtering.
    This helps in the reduction of the resolution and thus the total number of points being processed
    [Comment]: Mostly copied from the lecture notes
*/

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
   
    // Do voxel grid point reduction
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>());

    // region based filtering
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    // remove roof points
    std::vector<int> indexes;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indexes);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int elem : indexes)
        inliers->indices.push_back(elem);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PCLSeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstablesPC(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr segmentedPlanePC(new pcl::PointCloud<PointT>());

    // All the points that belongs to the plane will be fetched from the cloud
    // The inliers->indices are the points returned from RANSAC that belongs to a plane

    for (auto index : inliers->indices)
        segmentedPlanePC->points.push_back(cloud->points[index]);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // Create the filtering object
    // Set negative true to fetch all the obstables, i.e. not the inliers
    extract.setNegative(true);
    extract.filter(*obstablesPC);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstablesPC, segmentedPlanePC);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PCLSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Points we will be looking at to separate point cloud and separate it
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    // Coefficients:it will define what this plane and our model. To know what plane is and can use this to render plane

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Do segmentation on this cloud
    seg.setInputCloud(cloud);

    // Inliers: 
    // Points belonging to a plane, eg Road. We are only interested in inliers at the moment as it tells us 
    // how to separate the point cloud in two
    
    // Coefficients could be used later for rendering
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


/********************************************************************************************************

    [KK]: 27.01.2020
    KK_Custom_Ransac: Contains the custom implementation of the RANSAC algorithm

********************************************************************************************************/
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::KK_Custom_Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

        float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

        // Fit a plane in the above points
        float A = i;
        float B = j;
        float C = k;
        float D = -(i * x1 + j * y1 + k * z1);

        // Measure distance between every point and fitted line

        for (int index = 0; index < cloud->points.size(); index++)
        {
            // Check for the two points set above, if present ignore
            if (set_inliers.count(index) > 0)
                continue;

            // Assign values to variables: Saves fetching cost
            float point_X = cloud->points[index].x;
            float point_Y = cloud->points[index].y;
            float point_Z = cloud->points[index].z;

            float distance = std::fabs(A * point_X + B * point_Y + C * point_Z + D) / std::sqrt(A * A + B * B + C * C);

            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol)
                set_inliers.insert(index);
        }

        if (set_inliers.size() > inliersResult.size())
            inliersResult = set_inliers;
        // Return indicies of inliers from fitted line with most inliers
    }
    return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstablesPC(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr segmentedPlanePC(new pcl::PointCloud<PointT>());

    // All the points that belongs to the plane will be fetched from the cloud
    // The inliers->indices are the points returned from RANSAC that belongs to a plane

    for (auto index : inliers->indices)
        segmentedPlanePC->points.push_back(cloud->points[index]);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // Create the filtering object
    // Set negative true to fetch all the obstables, i.e. not the inliers
    extract.setNegative(true);
    extract.filter(*obstablesPC);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstablesPC, segmentedPlanePC);
    return segResult;
}


/********************************************************************************************************
    [KK]: 26.01.2020
    SegmentPlane: It calls the custom ransac implementation and returns the pair of outliers and inliers.
********************************************************************************************************/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KK_Custom_SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    // Call to custom RANSAC implementation
    std::unordered_set<int> inliers = KK_Custom_Ransac(cloud, maxIterations, distanceThreshold);

    if (inliers.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    // Fetch the points and segregate as inliers and outliers
    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


/********************************************************************************************************
    [KK]: 01.02.2020
    ClusterHelper: It marks points as processed. The points are then searched to identify the nearby points.
    The nearby points are then recursively calls ClusterHelper until all the points are processed 
 ********************************************************************************************************/

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(std::vector<bool>& processedPoints, const std::vector<std::vector<float>>& points,
    KdTree* tree, float distanceTol, std::vector<int>& cluster, int index)
{

    // Set the points being processed as true
    processedPoints[index] = true;

    // Add the index of the point to the cluster
    cluster.push_back(index);

    // Check for the points in proximity
    std::vector<int> nearbyPoints = tree->search(points[index], distanceTol);

    // Iterate through the nearby points and repeat the above steps for each
    for (int counter : nearbyPoints)
    {
        if (!processedPoints[counter])
        {
            clusterHelper(processedPoints, points, tree, distanceTol, cluster, counter);
        }
    }
}


/********************************************************************************************************
    [KK]: 01.02.2020
    
    euclideanCluster: It uses ClusterHelper to process the points. The pseudocode is as below (Ref: lecture page):
  
        EuclideanCluster():
            list of clusters
            Iterate through each point
                If point has not been processed
                    Create cluster
                    Proximity(point, cluster)
                    cluster add clusters
            return clusters
 ********************************************************************************************************/
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processedPoints(points.size(), false);

    for (unsigned int index = 0; index < points.size(); index++)
    {
        // If True, Skip. Process only when False.
        if (processedPoints[index])
            continue;

        // Store the cluster temporarily
        std::vector<int> cluster;
        clusterHelper(processedPoints, points, tree, distanceTol, cluster, index);

        // Push the temporary cluster to clusters
        clusters.push_back(cluster);
    }

    return clusters;

}

/*******************************************************************************************************
    [KK]: 26.01.2020
    KK_Custom_Clustering: Uses euclideanCluster method to find clusters within the provided point cloud.
********************************************************************************************************/


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KK_Custom_Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // KDTree object for inserting and searching operations

    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    
    // Iterate all the points in the point cloud
    for (int i = 0; i < cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];

        std::vector<float> tmp_points;
        tmp_points.push_back(point.x);
        tmp_points.push_back(point.y);
        tmp_points.push_back(point.z);

        // Insert into the KDTree
        tree->insert(tmp_points, i);
        points.push_back(tmp_points);
    }

    // cluster
    std::vector<std::vector<int>> clustersIndex = euclideanCluster(points, tree, clusterTolerance);

    // Iterate the clusters
    for (std::vector<int> clusterIndex : clustersIndex)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterPointCloud(new pcl::PointCloud<PointT>());

        // Iterate the points within cluster
        for (int index : clusterIndex)
        {
            clusterPointCloud->points.push_back(cloud->points[index]);
        }

        // Set the pcl attributes
        clusterPointCloud->width = clusterPointCloud->points.size();
        clusterPointCloud->height = 1;
        clusterPointCloud->is_dense = true;
        
        // Filter using the min and max size provided from the caller
        if ((clusterPointCloud->width >= minSize) && (clusterPointCloud->width <= maxSize))
            clusters.push_back(clusterPointCloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


/********************************************************************************************************
    [KK]: 30.01.2020
    Identifies the clusters within the provided point cloud using the PCL library
********************************************************************************************************/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PCL_Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices fetchIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

        for (int index: fetchIndices.indices)
            cloud_cluster->points.push_back(cloud->points[index]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


/****************************************************************************************************
    [KK]: 10.02.2020

    Method to find minimum oriented bounding box.
    Source: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

        Limitation: TODO: The z axis transformation still exists, leading to weird transformation
        of the detected obstacles. Z-Axis has to be removed.

****************************************************************************************************/
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::minBoundingBox(typename pcl::PointCloud<PointT>::Ptr obsCluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*obsCluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*obsCluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudSegmented);
    pca.project(*cloudSegmented, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*obsCluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoxQ boxQ;
    boxQ.bboxQuaternion = bboxQuaternion;
    boxQ.bboxTransform = bboxTransform;
    boxQ.cube_height = maxPoint.z - minPoint.z;
    boxQ.cube_width = maxPoint.y - minPoint.y;
    boxQ.cube_length = maxPoint.x - minPoint.x;

    return boxQ;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}