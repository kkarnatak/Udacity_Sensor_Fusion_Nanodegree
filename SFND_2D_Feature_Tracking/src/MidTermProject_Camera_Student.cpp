/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results
	bool writeToCSV = true;

	// Keypoint collection for task MP7
	ofstream csv_file_detector;
	csv_file_detector.open ("../Keypoints_Task_MP7_Detectors.csv");
	csv_file_detector << "Detector Name, Img1, Img2, Img3, Img4, Img5, Img6, Img7, Img8, Img9, Img10\n";

	// Keypoint collection for task MP8
	ofstream csv_file_descriptor;
	csv_file_descriptor.open ("../Keypoints_Task_MP8_Descriptors.csv");
	csv_file_descriptor << "Descriptor Name, Img1, Img2, Img3, Img4, Img5, Img6, Img7, Img8, Img9, Img10\n";

	// Time Logging for task MP9
	ofstream csv_file_time_log;
	csv_file_time_log.open ("../Log_Time_Task_MP9.csv");
	csv_file_time_log << "Descriptor Name, Img1, Img2, Img3, Img4, Img5, Img6, Img7, Img8, Img9, Img10\n";
	
	std::vector<std::string> allDetectorTypes;
	allDetectorTypes.push_back("SHITOMASI");
	allDetectorTypes.push_back("HARRIS");
	allDetectorTypes.push_back("FAST");
	allDetectorTypes.push_back("BRISK");
	allDetectorTypes.push_back("ORB");
	allDetectorTypes.push_back("AKAZE");
	allDetectorTypes.push_back("SIFT");
	
	std::vector<std::string> allDescripterTypes;
	allDescripterTypes.push_back("BRISK");
	allDescripterTypes.push_back("BRIEF");
	allDescripterTypes.push_back("ORB");
	allDescripterTypes.push_back("FREAK");
	allDescripterTypes.push_back("AKAZE");
	allDescripterTypes.push_back("SIFT");
	
	for( auto current_detector : allDetectorTypes )
	{
		bool isWriteDetector = true;
		
		for( auto current_descriptor : allDescripterTypes )
		{
			// To avoid combination of AKAZE and AKAZE
			// It leads to an exception, error: (-215:Assertion failed) 0 <= kpts[i].class_id && kpts[i].class_id < static_cast<int>(evolution_.size()) in function 'Compute_Descriptors'

            if(current_detector.compare("AKAZE")==0 && current_descriptor.compare("AKAZE")==0)
                 continue;

			if(current_detector.compare("AKAZE")!=0 && current_descriptor.compare("AKAZE")==0)
                 continue;

			if(dataBuffer.size() > 0)
			{
				// Reset DataBuffer before loading new images
				dataBuffer.clear();
			}

			cout << "\n===========================================================================================\n";
			cout << "      USING DECTECTOR: " << current_detector << ", \t" << "DESCRIPTOR: " << current_descriptor << endl;
			cout << "\n===========================================================================================\n";

			if(writeToCSV)
			{
				if(isWriteDetector)
					csv_file_detector << current_detector;

				csv_file_descriptor << current_detector + "_" + current_descriptor;
				csv_file_time_log << current_detector + "_" + current_descriptor;
			}

			/* MAIN LOOP OVER ALL IMAGES */

			for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
			{
				/* LOAD IMAGE INTO BUFFER */
				
				// Get the time
				double t = (double)cv::getTickCount();

				// assemble filenames for current index
				ostringstream imgNumber;
				imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
				string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

				// load image from file and convert to grayscale
				cv::Mat img, imgGray;
				img = cv::imread(imgFullFilename);
				cout << "\nReading File: " << imgFullFilename << endl;
				cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

				//// STUDENT ASSIGNMENT
				//// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

				// push image into data frame buffer
				DataFrame frame;
				frame.cameraImg = imgGray;
				std::cout << "===========================================================================================\n";
				std::cout << "\n[KKLOG]: Ring Buffer Size = " << dataBuffer.size() << std::endl;

				// Check for the dataBuffer size
				// Add +1 to consider the additional push_back
				if( dataBuffer.size()+1 > dataBufferSize )
				{
					dataBuffer.erase( dataBuffer.begin() );
					std::cout << "\n[KKLOG]: RingBuffer Full! Overwrite image in buffer\n";
				}

				dataBuffer.push_back(frame);

				//// EOF STUDENT ASSIGNMENT
				cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
				std::cout << "===========================================================================================\n";

				/* DETECT IMAGE KEYPOINTS */

				
				// extract 2D keypoints from current image
				vector<cv::KeyPoint> keypoints; // create empty feature list for current image
				string detectorType = current_detector;

				//// STUDENT ASSIGNMENT
				//// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
				//// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

				if (detectorType.compare("SHITOMASI") == 0)
				{
					detKeypointsShiTomasi(keypoints, imgGray, false);
				}
				else if(detectorType.compare("HARRIS") == 0)
				{
					detKeypointsHarris(keypoints, imgGray, false);
				}
				else if(std::find(allDetectorTypes.begin(), allDetectorTypes.end(), detectorType)!= allDetectorTypes.end())
				{
					detKeypointsModern(keypoints, imgGray, detectorType, false);
				}
				//// EOF STUDENT ASSIGNMENT

				//// STUDENT ASSIGNMENT
				//// TASK MP.3 -> only keep keypoints on the preceding vehicle

				vector<cv::KeyPoint>::iterator it_keypoint;
				vector<cv::KeyPoint> region_of_interest;

				// only keep keypoints on the preceding vehicle
				bool bFocusOnVehicle = true;
				cv::Rect vehicleRect(535, 180, 180, 150);
				if (bFocusOnVehicle)
				{
					// ...
					for(it_keypoint = keypoints.begin(); it_keypoint != keypoints.end(); ++it_keypoint)
					{
						if( vehicleRect.contains(it_keypoint->pt) )
						{
							cv::KeyPoint tempKeyPoint;
							tempKeyPoint.pt = cv::Point2d(it_keypoint->pt);
							tempKeyPoint.size = 1;
							region_of_interest.push_back(tempKeyPoint);
						}
					}
					std::cout << "===========================================================================================\n";
					std::cout << "Size of original keypoints: " << keypoints.size() << "\n";
					std::cout << "Size of keypoints within bounding box: " << region_of_interest.size() << "\n";
					std::cout << "Filtered keypoints size: " << keypoints.size() - region_of_interest.size() << "\n";
					std::cout << "===========================================================================================\n";

					keypoints = region_of_interest;
				}
				
				// Task MP7: Detector keypoint collection
				// Make an entry in the detector keypoints csv
				if(writeToCSV && isWriteDetector)
				{
					csv_file_detector << ", " << keypoints.size();
				}

				//// EOF STUDENT ASSIGNMENT

				// optional : limit number of keypoints (helpful for debugging and learning)
				bool bLimitKpts = false;
				if (bLimitKpts)
				{
					int maxKeypoints = 50;

					if (detectorType.compare("SHITOMASI") == 0)
					{ // there is no response info, so keep the first 50 as they are sorted in descending quality order
						keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
					}
					cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
					cout << " NOTE: Keypoints have been limited!" << endl;
				}

				// push keypoints and descriptor for current frame to end of data buffer
				(dataBuffer.end() - 1)->keypoints = keypoints;
				cout << "#2 : DETECT KEYPOINTS done" << endl;

				/* EXTRACT KEYPOINT DESCRIPTORS */

				//// STUDENT ASSIGNMENT
				//// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
				//// -> BRIEF, ORB, FREAK, AKAZE, SIFT

				cv::Mat descriptors;
				string descriptorType = current_descriptor; // BRIEF, ORB, FREAK, AKAZE, SIFT
				descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
				//// EOF STUDENT ASSIGNMENT

				// push descriptors for current frame to end of data buffer
				(dataBuffer.end() - 1)->descriptors = descriptors;

				cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

				if (dataBuffer.size() > 1) // wait until at least two images have been processed
				{

					/* MATCH KEYPOINT DESCRIPTORS */

					vector<cv::DMatch> matches;
					string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
					string descriptorType;// Commented to avoid same setting for every descriptor = "DES_BINARY"; // DES_BINARY, DES_HOG
					string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
					
					// Use HOG descriptor only for SIFT
					if (descriptorType.compare("SIFT") == 0) 
                    {
                        descriptorType == "DES_HOG";
                    }
                    else
                    {
                        descriptorType == "DES_BINARY";
                    }       
					//// STUDENT ASSIGNMENT
					//// TASK MP.5 -> add FLANN matching in file matching2D.cpp
					//// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

					matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
									(dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
									matches, descriptorType, matcherType, selectorType);

					//// EOF STUDENT ASSIGNMENT

					// store matches in current data frame
					(dataBuffer.end() - 1)->kptMatches = matches;

					// Get the time again and calculate the difference: TASK MP9
					t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

					// Task MP8, MP9: Descriptor keypoint collection and time log
					// Make an entry of descriptor keypoint and time log in csv
					if(writeToCSV)
					{
						csv_file_descriptor << ", " << matches.size();
						csv_file_time_log << ", " << 1000*t;
					}

					cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

					// visualize matches between current and previous image
					bVis = false;
					if (bVis)
					{
						cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
						cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
										(dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
										matches, matchImg,
										cv::Scalar::all(-1), cv::Scalar::all(-1),
										vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

						string windowName = "Matching keypoints between two camera images";
						cv::namedWindow(windowName, 7);
						cv::imshow(windowName, matchImg);
						cout << "Press key to continue to next image" << endl;
						cv::waitKey(0); // wait for key to be pressed
					}
					bVis = false;
				}			
			} // eof loop over all images
			
			if(writeToCSV)
			{
				csv_file_descriptor << "\n";
				csv_file_time_log << "\n";
				if(isWriteDetector)
					csv_file_detector << "\n";
			}
			isWriteDetector = false;
		} // eof loop over the descriptors
	} // eof loop over the detectors

	csv_file_descriptor.close();
	csv_file_detector.close();

    return 0;
}
