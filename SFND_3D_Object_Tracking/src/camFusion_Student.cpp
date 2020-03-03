
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double sumDistance = 0;
    double meanDistance = 0;
    std::vector<cv::DMatch> keypointMatchesROI;

    // Iterate over the current keypoint matches
    for (cv::DMatch match : kptMatches) {
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)) {
            keypointMatchesROI.push_back(match);
        }
    }

    for(auto kpMatch: keypointMatchesROI)
    {
        sumDistance += kpMatch.distance;
    }

    int kpVectorSize = keypointMatchesROI.size();

    if( kpVectorSize > 0 )
    {
        meanDistance = sumDistance / kpVectorSize;
    }
    else
    {
        // No keypoints matches found. Nothing to do further!
        return;
    }

    // Calculate the threshold distance using the mean
    const double c_THRESHOLD_FACTOR = 0.7;

    double thresholdDistance = meanDistance * c_THRESHOLD_FACTOR;
    
    for( auto kpMatch: keypointMatchesROI )
    {
        if( kpMatch.distance < thresholdDistance )
        {
            // Add the keypoint to filtered bounding boxes
            boundingBox.kptMatches.push_back(kpMatch);
        }
    }

    std::cout << "Total kp match size = " << boundingBox.kptMatches.size() << std::endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double dT = 1.0/frameRate;        // time between two measurements in seconds
    
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame

    // Added to avoid crash when no keypoints were found for certain detectors, e.g. HARRIS
    if( kptMatches.size() == 0 )
        return;

    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }


    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
     // auxiliary variables
    double dT = 1.0/frameRate;        // time between two measurements in seconds
    const double c_LANE_WIDTH = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 0;
    double minXCurr = 0;

    vector<double> vecPrevPoints;
    vector<double> vecCurrPoints;

    /*
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        
        if (abs(it->y) <= c_LANE_WIDTH / 2.0)
        { // 3D point within ego lane?
            //minXPrev = minXPrev > it->x ? it->x : minXPrev;
            vecPrevPoints.push_back(it->x);
        }
    }

    
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {

        if (abs(it->y) <= c_LANE_WIDTH / 2.0)
        { // 3D point within ego lane?
            // minXCurr = minXCurr > it->x ? it->x : minXCurr;
            vecCurrPoints.push_back(it->x);
        }
    }
    */

    for( auto lidarPoints : lidarPointsPrev )
    {
        if( std::abs(lidarPoints.y) <= c_LANE_WIDTH / 2.0)
        { // 3D point within ego lane?
            vecPrevPoints.push_back(lidarPoints.x);
        }
    }

    for( auto lidarPoints : lidarPointsCurr )
    {
        if( std::abs(lidarPoints.y) <= c_LANE_WIDTH / 2.0)
        { // 3D point within ego lane?
            vecCurrPoints.push_back(lidarPoints.x);
        }
    }

    if (lidarPointsPrev.size() > 0)
    {
       for (auto x: vecPrevPoints)
            minXPrev += x;
       minXPrev = minXPrev / lidarPointsPrev.size();
    }
    if (lidarPointsCurr.size() > 0)
    {
       for (auto x: vecCurrPoints)
           minXCurr += x;
       minXCurr = minXCurr / lidarPointsCurr.size();
    }
    // compute TTC from both measurements
    cout << "minXCurr: " << minXCurr << endl;
    cout << "minXPrev: " << minXPrev << endl;
    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // Get the size of bounding boxes
    int previousFrame = prevFrame.boundingBoxes.size();
    int currentFrame = currFrame.boundingBoxes.size();

    // To mark which mapping matches
    int map_matching[previousFrame][currentFrame] = {};
    std::multimap<int, int> mMapForMatches;

    // Temporary storage
    vector<int> vectorQueryIndex;
    vector<int> vectorTrainIndex;
    
    int maxPrevBoxID = 0;
    // Loop over all the matches
    for( auto matchIndex : matches )
    {
        // Get the query and training points from resp frames

        cv::KeyPoint keyPointQuery = prevFrame.keypoints[matchIndex.queryIdx];
        cv::KeyPoint keyPointTrain = currFrame.keypoints[matchIndex.trainIdx];

        // Create the points
        cv::Point pointFromQuery = cv::Point( keyPointQuery.pt.x, keyPointQuery.pt.y);
        cv::Point pointFromTrain = cv::Point( keyPointTrain.pt.x, keyPointTrain.pt.y);

        int foundPrevBoxId = -1;
        int foundCurrBoxId = -1;

        // Search for the query point in the bounding box ROI
        for( auto previousFrameIndex:  prevFrame.boundingBoxes )
        {
            if(previousFrameIndex.roi.contains(pointFromQuery))
            {
                foundPrevBoxId = previousFrameIndex.boxID;
                //vectorQueryIndex.push_back(previousFrameIndex.boxID);
            }
        }

        // Search for the train point in the bounding box ROI
        for( auto currentFrameIndex : currFrame.boundingBoxes )
        {
            if(currentFrameIndex.roi.contains(pointFromTrain))
            {
                foundCurrBoxId = currentFrameIndex.boxID;
                //vectorTrainIndex.push_back(currentFrameIndex.boxID);
            }
        }

        mMapForMatches.insert(std::pair<int, int>(foundCurrBoxId, foundPrevBoxId));
        maxPrevBoxID = std::max(maxPrevBoxID, foundPrevBoxId);
    }

    vector<int> currFrameBoxIDs {};
    for (auto box : currFrame.boundingBoxes) currFrameBoxIDs.push_back(box.boxID);

    for( auto index : currFrameBoxIDs )
    {
        int count = 0;
        auto rangePrevBoxIDs = mMapForMatches.equal_range(index);
        // Create a vector of counts (per current bbox) of prevBoxIDs
        std::vector<int> counts(maxPrevBoxID + 1, 0);
        // Accumulator loop
        for (auto it = rangePrevBoxIDs.first; it != rangePrevBoxIDs.second; ++it) {
            if (-1 != (*it).second) counts[(*it).second] += 1;
        }

        // Get the index of the maximum count (the mode) of the previous frame's boxID
        int modeIndex = std::distance(counts.begin(), std::max_element(counts.begin(), counts.end()));

        // Set the best matching bounding box map with
        // key   = Previous frame's most likely matching boxID
        // value = Current frame's boxID, k
        bbBestMatches.insert({modeIndex, index});
    }

    // Display the matches
    bool bMsg = true;
    if (bMsg)
    {
        for (int i = 0; i < previousFrame; i++)
            cout << "Box " << i << " matches " << bbBestMatches[i] << " box" << endl;
    }
    
}
