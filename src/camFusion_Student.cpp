
#include <iostream>
#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

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

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
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
    float distanceSum = 0.0;
    std::vector<cv::DMatch> matchesInsideROI;

    for (const cv::DMatch& match : kptMatches){
        // check if both keypoints are in this bounding box
        cv::KeyPoint kptPrev = kptsPrev[match.queryIdx];
        cv::KeyPoint kptCurr = kptsCurr[match.trainIdx];
        if (boundingBox.roi.contains(kptPrev.pt) && boundingBox.roi.contains(kptCurr.pt)){
            distanceSum += match.distance;
            matchesInsideROI.push_back(match);
        } 
    }

    float distanceAvg = distanceSum / matchesInsideROI.size();
    float distanceThreshold = 1.75;     // match distance must be less than (average + 75% of average)
    float maxDistanceAllowed = distanceAvg * distanceThreshold;
    //std::cout << "distanceAvg: " << distanceAvg << std::endl;
    //std::cout << "maxDistanceAllowed: " << maxDistanceAllowed << std::endl;
    
    for (const cv::DMatch& match : matchesInsideROI){
        // check if match distance is below threshold and add to bounding box match vector
        if (match.distance <= maxDistanceAllowed) boundingBox.kptMatches.push_back(match);
    }

    // TODO: Fill out boundingBox.keypoints
    // get keypoints in current and previous bounding boxes
    /*
    std::vector<cv::KeyPoint> kptsCurrBB, kptsPrevBB;

    for (const auto& match : boundingBox.kptMatches){
        cv::KeyPoint kptPrev = kptsPrev[match.queryIdx];
        cv::KeyPoint kptCurr = kptsCurr[match.trainIdx];
        boundingBox.keypoints.push_back(kptPrev);
        kptsCurrBB.push_back(kptCurr);
    }
    */
    //std::cout << "\nmatchesInsideROI.size(): " << matchesInsideROI.size() << std::endl;
    //std::cout << "boundingBox.kptMatches.size(): " << boundingBox.kptMatches.size() << std::endl;

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // find the highest and lowest keypoints in each bounding box
    std::vector<double> ratios;
    for (auto kptMatch1 = kptMatches.begin(); kptMatch1 != kptMatches.end(); kptMatch1++){
        cv::KeyPoint kptPrev1 = kptsPrev[kptMatch1->queryIdx];
        cv::KeyPoint kptCurr1 = kptsCurr[kptMatch1->trainIdx];

        for (auto kptMatch2 = kptMatches.begin(); kptMatch2 != kptMatches.end(); kptMatch2++){

            cv::KeyPoint kptPrev2 = kptsPrev[kptMatch2->queryIdx];
            cv::KeyPoint kptCurr2 = kptsCurr[kptMatch2->trainIdx];

            if (kptPrev1.pt != kptPrev2.pt){
                double distP = cv::norm(kptPrev1.pt - kptPrev2.pt);
                double distC = cv::norm(kptCurr1.pt - kptCurr2.pt);

                if (distC >= 100) ratios.push_back(distC/distP);
            }
        }
    }
    
    // find median ratio and use it for TTC calculation
    std::sort(ratios.begin(), ratios.end());
    double ratioMedian = ratios[floor(ratios.size()/2)];

    // comput ttc based on height ratio
    TTC = -(1/frameRate) / (1 - ratioMedian);

    std::cout << "TTC camera: " << TTC << std::endl;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Step 1. Get closest lidar point in boundingbox to camera (ignore outliers)
    // find average x values of lidar points in the previous and current frames
    double xSumPrev, xSumCurr;
    for (const auto& pt : lidarPointsPrev){
        xSumPrev += pt.x;
    }
    for (const auto& pt : lidarPointsCurr){
        xSumCurr += pt.x;
    }
    double xAvgPrev = xSumPrev/lidarPointsPrev.size();
    double xAvgCurr = xSumCurr/lidarPointsCurr.size();

    // Step 2. Calculate TTC
    TTC = xAvgCurr * ((1/frameRate)/(xAvgPrev - xAvgCurr));
    //std::cout << "TTC LiDAR: " << TTC << std::endl;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //TODO: stop passing matches because they are already stored in DataFrame
    std::map<std::pair<int,int>, int> bbMatches;    //std::map<bounding box pair, number of occurences>

    for (auto m : matches){
        cv::KeyPoint kptPrev = prevFrame.keypoints[m.queryIdx];
        cv::KeyPoint kptCurr = currFrame.keypoints[m.trainIdx];

        int bb1ID, bb2ID;
        bool kptCurrInBB = false, kptPrevInBB = false;
        for (auto bb : currFrame.boundingBoxes){
            if (bb.roi.contains(kptCurr.pt)){
                bb1ID = bb.boxID;
                bb.keypoints.push_back(kptCurr);
                kptCurrInBB = true;
                break;
            }
        }

        for (auto bb : prevFrame.boundingBoxes){
            if (bb.roi.contains(kptPrev.pt)){
                bb2ID = bb.boxID;
                bb.keypoints.push_back(kptPrev);
                kptPrevInBB = true;
                break;
            }
        }

        if (kptCurrInBB && kptPrevInBB){        //only consider keypoint matches where both keypoints are in bounding boxes in their respective frames
            std::pair<int,int> bbMatch (bb2ID, bb1ID);
            if (bbMatches.count(bbMatch) > 0){
                bbMatches[bbMatch] += 1;
            }
            else{
                bbMatches[bbMatch] = 1;
            }
        }
    }
    
    std::unordered_set<int> prevIDs;
    // for every bounding box in the previous frame within the map, find out which bounding box in the current frame it is most frequently matched to
    for (const auto& [bbPair1, count1] : bbMatches){
        
        if (prevIDs.contains(bbPair1.first)) continue;      // only process a prevFrame once
        prevIDs.insert(bbPair1.first);

        int maxCount = 0;
        std::pair<int,int> bestPair;
        for (const auto& [bbPair2, count2] : bbMatches){
            if (bbPair1.first == bbPair2.first){
                // get count and swap if larger than current count.
                if (count2 > maxCount){
                    maxCount = count2;
                    bestPair = bbPair2;
                }
            }
        }

        // update the best bounding box match for the current prevFrame
        bbBestMatches[bestPair.first] = bestPair.second;    //bbBestMatches[previous Frame ID] = current Frame ID;
    }
}
