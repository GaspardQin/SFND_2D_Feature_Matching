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
#include "ringBuffer.h"
using namespace std;

//#define PROFILE_MP7
//#define PROFILE_MP8_9

int run(string detectorType, string descriptorType, string matcherType, string selectorType)
{

    /* INIT VARIABLES AND DATA STRUCTURES */
    std::cout<<"======================="<<std::endl;
    std::cout<<"run with "<<detectorType<<" as detector, "<<descriptorType<<" as descriptor, "<<matcherType<<" as matcher, "<<selectorType<<" as selector."<<std::endl;
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
    RingBuffer<DataFrame> dataBuffer(dataBufferSize); // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    // save the data to analyse
    std::ofstream csv_file;
    #ifdef PROFILE_MP7
    // for performance evaluation 1
    csv_file.open("../analyse_data/detector/" + detectorType + ".csv");
    csv_file << "frame_id,x,y,neighbour_size\n";
    #endif

    #ifdef PROFILE_MP8_9
    // for performance evaluation 2 and 3
    csv_file.open("../analyse_data/combination/" + detectorType + "_" + descriptorType + ".csv");
    csv_file << "frame_id,mached_keypoints,detect_time,extract_time\n";
    #endif

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        
        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame); // here, the dataBuffer is a RingBuffer type

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
       

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        double detectTime;
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detectTime = detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else
        {
            detectTime = detKeypointsModern(keypoints, imgGray, detectorType);
        }


        


        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {   
            vector<cv::KeyPoint> keypointsFiltered;
            for(auto it=keypoints.begin(); it != keypoints.end(); it++){
                if(vehicleRect.contains(it->pt)){
                    keypointsFiltered.push_back(*it);
                }
            }
            keypoints.swap(keypointsFiltered);
        }
        
        #ifdef PROFILE_MP7
        // for performance evaluation 1
        // save key points to csv files 
        for(int i =0; i<keypoints.size(); i++){
            csv_file<<imgIndex<<","<<keypoints[i].pt.x<<","<<keypoints[i].pt.y<<","<<keypoints[i].size<<"\n";
        }
        #endif

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
        
        double descriptTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG Not used, actually

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            #ifdef PROFILE_MP8_9
            // for Performance Evaluation 2 and 3
            csv_file << imgIndex <<","<<matches.size()<<","<<detectTime<<","<<descriptTime<<"\n";
            #endif
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
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
        }

    } // eof loop over all images
    csv_file.close();
    return 0;
}

void testForDetector(vector<string>& detectorTypeArray, string descriptorType, string matcherType, string selectorType){
    for(auto detectorTypeIt=detectorTypeArray.begin(); detectorTypeIt != detectorTypeArray.end(); detectorTypeIt++){
        run(*detectorTypeIt, descriptorType, matcherType, selectorType);
    }
}
void testForDescriptor(string detectorType, vector<string>& descriptorTypeArray, string matcherType, string selectorType){
    for(auto descriptorTypeIt=descriptorTypeArray.begin(); descriptorTypeIt != descriptorTypeArray.end(); descriptorTypeIt++){
        run(detectorType, *descriptorTypeIt, matcherType, selectorType);
    }
}

void testForMatcher(string detectorType, string descriptorType, vector<string>& matcherTypeArray, string selectorType){
    for(auto matcherTypeIt=matcherTypeArray.begin(); matcherTypeIt != matcherTypeArray.end(); matcherTypeIt++){
        run(detectorType, descriptorType, *matcherTypeIt, selectorType);
    }
}
void testForSelector(string detectorType, string descriptorType, string matcherType, vector<string>& selectorTypeArray){
    for(auto selectorTypeIt=selectorTypeArray.begin(); selectorTypeIt != selectorTypeArray.end(); selectorTypeIt++){
        run(detectorType, descriptorType, matcherType, *selectorTypeIt);
    }
}

void testForCombination(vector<string>& detectorTypeArray, vector<string>& descriptorTypeArray){
    for(auto detectorTypeIt=detectorTypeArray.begin(); detectorTypeIt != detectorTypeArray.end(); detectorTypeIt++){
        for(auto descriptorTypeIt=descriptorTypeArray.begin(); descriptorTypeIt != descriptorTypeArray.end(); descriptorTypeIt++){
            run(*detectorTypeIt, *descriptorTypeIt, "MAT_BF", "SEL_KNN");
        }
    }
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[]){
    vector<string> detectorTypeArray = {"SHITOMASI", "FAST", "SIFT", "HARRIS", "BRISK", "ORB", "AKAZE"};
    vector<string> descriptorTypeArray = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    vector<string> matcherType = {"MAT_BF", "MAT_FLANN"};
    vector<string> selectorType = {"SEL_NN", "SEL_KNN"};
    
    #ifdef PROFILE_MP7
    // for MP.7 Performance Evaluation 1
    testForDetector(detectorTypeArray, descriptorTypeArray[0], matcherType[0], selectorType[0]);
    #endif

    #ifdef PROFILE_MP8_9
    // for MP.8 Performance Evaluation 2 and MP.9 Performance Evaluation 3
    testForCombination(detectorTypeArray, descriptorTypeArray);
    #endif
    run(detectorTypeArray[0], descriptorTypeArray[0], matcherType[0], selectorType[0]);
}