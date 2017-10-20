# Unscented Kalman Filters

---

**Unscented Kalman Filters Project**

The goals / steps of this project are the following:

* Initialize properties
* Tune noise parameters
* Predict Sigma points, state, and covariance matrix
* Update the state and covariance matrix based on Lidar and Radar measurements
* The RMSE (root-mean-square error) of the position data px, py, vx, vy should be less than or equal to the values [.09, .10, .40, .30]
* Calculate the NIS (filter consistency)

[//]: # (References)
[simulator]: https://github.com/udacity/self-driving-car-sim/releases
[win 10 update]: https://support.microsoft.com/de-de/help/4028685/windows-get-the-windows-10-creators-update
[uWebSocketIO]: https://github.com/uWebSockets/uWebSockets
[linux on win 10]: https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/
[MinGW]: http://www.mingw.org/
[CMake]: https://cmake.org/install/
[udacity code]: https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project

---

## Files Submitted & Code Quality

### 1. Submission includes all required files and every TODO task has been accomplished 

For this project, I have used the [Unscented Kalman Filter Project Starter Code][udacity code] from Udacity and I have modified the following five files:
```cpp
ukf.cpp
ukf.h
tools.cpp
```

The ```ukf.cpp``` file initializes all process and measurement matrices from line 77 - 98. After the first initialization of the state `x_`, the state then gets predicted in the `void UKF::Prediction()` function on line 106. After the prediction, we update the state and covariance matrices based on the sensor type from line 111 - 115 in ```ukf.cpp```. The functions `void UKF::UpdateRadar()` on line 304 and `void UKF::UpdateLidar()` on line 245 update the measurements for the sensor types. The main difference between both sensor types is the transformation of sigma points into measurement space. While Radar has three rows in the sigma matrix (rho, phi, rho dot), Lidar has only two rows (px, py). The calculation of the covariance (line 125), the cross-correlation (line 141) and NIS (line 155) stays the same.
The `VectorXd Tools::CalculateRMSE()` function on line 12 in the ```tools.cpp``` file calculates the root-mean-squared error and returns the value. 

### 2. Code must compile without errors

This project was done on Windows 10. In order to set up this project I had to:
* update my Windows 10 Version with the [Windows 10 Creators Update][win 10 update]
* install the [Linux Bash Shell][linux on win 10] (with Ubuntu 16.04) for Windows 10
* set up and install [uWebSocketIO][uWebSocketIO] through the Linux Bash Shell for Windows 10
* [download the simulator from Udacity][simulator]

**To update the Linux Bash Shell to Ubuntu 16.04 the Windows 10 Creators Update has to be installed!**

Also, [CMake][CMake] and a gcc/g++ compiler like [MinGW][MinGW] is required in order to compile and build the project.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory in the Linux Bash Shell.

1. `mkdir build`
2. `cd build`
3. `cmake .. -G "Unix Makefiles" && make` on Windows 10 or `cmake .. && make` on Linux or Mac
4. `./UnscentedKF`

Then the simulator has to be started and *Project 1/2: EKF and UKF* has to be selected. When everything is set up the Linux Bash Shell should print: 
```bash 
Listening to Port 4567
Connected
```

---

## Discussion

### 1. Briefly discuss any problems / issues you faced in your implementation of this project.
I had the problem that the car in the simulator stopped after the 3rd time step. After some debugging I found out that my program got stuck in an infinite loop in the `void UKF::AngleNormalization()` function on line 120 in `ukf.cpp`. This was because I've messed up the calculation of the predicted sigma points and therefore some values in the matrix were too big. 
After I have fixed this problem, I got rid of the infinite loop and my program was running as expected.
