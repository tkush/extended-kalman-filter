# extended-kalman-filter
This code implements a 2D Extended Kalman filter (EKF) that does the following:
1. Predict and update pedestrian location (x, y) and velocity (vx, vy) in 2D 
2. Fuse sensor data from a LIDAR and a RADAR
3. Implement an EKF to do the above
4. Calculate the RMSE between ground truth and estimations

The sample RADAR and LASER data is obtained from Udacity's Term 2, Project 1. 

Briefly, the steps are the following: 
1. Initialize the EKF matrices (covariances, state transition, measurement functions)
2. Initialize the states with the first measurement (radar/lidar)
3. Predict the current position/velocity based on the last measurement
4. After a measurement comes in (radar/lidar), update the prediction for that time step
5. Repeat

### Key points
* The LASER data contains (x,y) location only. There is no velocity information
* The RADAR data contains information in polar coordinates. This must be transformed to cartesian coordinates before use
* The RADAR measurement mapping to the states is non-linear. Therefore a linearization must be done about the point of measurement each time a new RADAR data comes in

The source code and the sample data is uploaded here. This project was built using Ubuntu Bash on Windows (10). 

### Results
Using the initializations in the code, I am able to achieve an RMSE of (0.097, 0.085, 0.451, 0.439) for (px, py, vx, vy). 

![RMSE values](/images/img_01.png)

Here is a comparison of the estimated positions and velocities w.r.t. the ground truth

![Comparison](/images/img_02.png)

Finally, here is the same comparison plot, zoomed in to show how the estimations fare with the ground truth

![Comparison2](/images/img_03.png)

### Inferences
From the plots above, the following can be deduced:
* Fused data provides the best estimates for both position and velocity as compared to LASER or RADAR data alone
* Positions are estimated better than velocities. This can be thought to be expected since both LASER and RADAR data provide a measure of position whereas only the RADAR data provides a measure of velocity
* The starting value of velocities does not seem to affect the estimator much. The estimator converges quickly with some initial transients
* There is an increase in prediction error between measurements # 100-200. At this point, the object is peaking in py and both vx and vy. The cause for this could not be determined merely by looking at the plots
