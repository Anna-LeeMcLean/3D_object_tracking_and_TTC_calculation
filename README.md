# 3D_object_tracking_and_TTC_calculation
This project was done in order to track preceding objects on a highway and compute the Time-To-Collision (TTC) with an ego car. Two estimates for the TTC were calculated for each of the eighteen (18) image frames using image keypoints from a camera and LiDAR measurements. The image stream shows 18 images where the preceding vehicle slows down and the distance between it and the ego car gradually decreases.

![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/70b5fa16-1229-4c7e-9995-2bffddc8ff10)

*Figure 1 showing Highway Scene with BoundingBox and LiDAR points for preceding vehicle in the ego lane*

## Project Workflow
![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/921e0362-6c5a-4dfb-86ad-ff9f5a6aeaef)

*Figure 2 showing workflow used to calculate separate TTC estimates using camera and LiDAR data*

## TTC Estimates using LiDAR Data
![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/8a73db14-b7ad-4a97-8ee0-5cf00060a0e0)

*Figure 3 showing TTC estimates using LiDAR data for all 18 images*

Figure 3 shows the time-to-collision estimates for each of the 18 images in seconds. Ideally, the graph would gradually decrease from image 1 to image 18 since the distance between the preceding vehicle and the ego car gradually decreases with each image frame. The estimates were calculated using a constant velocity model where:

TTC = d1 * (dT / (d0-d1))            

*d1 - distance between ego car and preceding vehicle in current frame; d0 - distance between ego car and preceding vehicle in previous frame*

d1 and d0 were estimated using the median of all the values for each LiDAR point in the respective frames. Only LiDAR points within the region of interest were considered. A median estimate of the preceding vehicle distance was done instead of using the closest data point in order to reduce the effect of outliers due to noise. This method however does not completely filter out the noise effect which has seemingly resulted in an increase of the TTC estimate for images 3 and 17 in Figure 3. 

## TTC Estimates using Feature Keypoints from Camera Image

A second TTC estimate was calculated for the same dataset using distance ratios between keypoints in consecutive images. A keypoint detector/descriptor combo was used to detect, describe and match keypoints which lie in the region of interest. The distance between multiple keypoints in the previous frame is determined and the same done for the matching keypoints in the current frame. The increase/decrease in distance between keypoints allows us to get an estimate of the TTC using the following equation:

TTC = - (dT / (1 - h1/h0))

*h0 - distance between keypoint A1 and B1 in previous frame; h1 - distance between keypoint A2 and B2 in the current frame where A1 and A2 are a matching pair and B1 and B2 are another matching pair*

If the matching algorithm has done its job correctly, the h1/h0 ratio should be the same for every combination of keypoints. However, it is important to make sure that keypoint outliers which arise from potential incorrect matches are not considered. To filter outlier ratios, the median of ratios for all combinations of keypoints was used in the equation. The results below show the TTC estimates using different combinations of keypoint detectors and descriptors. OpenCV's Brute Force KNN matching algorithm was used with k=2. The plot title indicates the detector used while the legend indicates the results with various descriptors and the equivalent LiDAR estimates for comparison.

![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/51e0895a-ae68-4e38-9dc6-455cdcf19ebc)
![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/11104ee5-fb8a-4c14-ba8f-78dce89420d6)
![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/4b828108-ed4f-4f01-ad45-4fc89acd3205)
![image](https://github.com/Anna-LeeMcLean/3D_object_tracking_and_TTC_calculation/assets/60242931/e731d184-ce91-427d-b574-03a88fbe458c)

The AKAZE detector shows the best result when paired with various descriptors. The FAST detector plot also shows a decent range of results but also has the least steep slope of all plots shown. This implies that the TTC does not decrease that much as the preceding car gets closer to the ego car. One limitation of the FAST detector for this application is its variance with scale. The difference in scale is used to estimate the change in distance between the cars and therefore a scale invariant detector is required for this application. The BRISK detector is scale invariant and therefore starts out as a good candidate to use for this application. Looking at its plot, there are however obvious estimate errors, particularly on frame 5 with the SIFT and BRISK descriptor pairings. The SIFT detector has similar obvious errors on image 4 with all of its descriptor pairings. These errors could be due to the size of the bounding box which indicates the region of interest. The box allows keypoints not on the car to be included in the TTC estimate. One way to alleviate this is to reduce the region of interest to the rectagle that encloses just the bottom of the car (tailights down to back bumper) and exclude the back windshield. The number of keypoints will reduce significantly but this will guarantee that the keypoints detected lie on the car and the scale ratios calculated would better represent the change in distance between frames. Multiple incorrect keypoint matches could also be a source of error. The HARRIS and ORB detectors gave unpresentable results which included huge estimates (on the order of 10000s), NaNs and negative infinites. The results with these detectors were therefore left out. 

### Recommendation
The BRIEF descriptor and FAST detector are the fastest algorithms but both are not robust to scale variance. We will therefore need to sacrifice speed for accuracy in this application. The fastest combination of scale-invariant detector and descriptor should therefore be used while reducing the region of interest to ignore keypoints not on the car. 





