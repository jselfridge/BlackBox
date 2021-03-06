
BLACK BOX
=========

Originally developed a cape for a BeagleBone Black to enable 
"Black Box" functionality for research and development.  Currently, 
the BeagleBone Blue, along with a more developed API coded in C, 
looks to offer a better solution.  Starting a new repo to house a 
collection of robotics projects with this new hardware and 
software implementation.  Thsi repo will continue to exist, while 
the features and capabilities are being ported over into the new 
repo, and will be removed once it has fully served its purpose.    


Current Work-In-Progress
------------------------
Enable EKF with Kalman gain update inside INS timing loop.
<ul> 
  <li> TODO: Initialize and exit EKF data structures. </li>
  <li> TODO: Run EKF inside stabilization loop. </li>
  <li> TODO: Run gain update in INS loop. </li>
  <li> TODO: Enable data logging. </li>
  <li> TODO: Print results to debugging terminal. </li>
  <li> TODO: Flight test EKF system. </li>
  <li> TODO: Compare results to Matlab sysid results. </li>
</ul>


v0.2.1 - Inertial Navigation System Loop
----------------------------------------
<ul> 
  <li> Createed INS souce code and header files. </li>
  <li> Setup INS timing loop. </li>
  <li> Enabled data logging on timing loop. </li>
  <li> Established debgging terminal print statements. </li>
</ul>


v0.2.0 - Quadrotor Basic Flight Capabilities
--------------------------------------------
This software version focused on the sensors and algorithms to stabilize 
a quadrotor system flying in the air.  It adds a second IMU sensor with a
9DOF Attitude Heading and Reference SYstem (AHRS) algorithm, GPS framework, 
a Ground Control Station (GCS), the framework for an Extended Kalman Filter
(EKF), and state feedback controller for inner loop stabilization.

<ul>
<li><b> v0.1.8 - State Feedback Control </b></li>
<ul>
  <li> Revised data structure. </li>
  <li> Obtained ref model param from settling time (ts) and percent overshoot (mp). </li>
  <li> Developed reference model signals. </li>
  <li> Hard code a second order SISO algorithm. </li>
  <li> Running control law within the stabilization thread. </li>
  <li> Implemented data logging. </li>
  <li> Added parameters to GCS interface. </li> 
  <li> Bench tested and flight tested new code. </li>
</ul>
<li><b> v0.1.7 - Extended Kalman Filter (EKF) Development </b></li>
<ul>
  <li> Created module for Extended Kalman Filter (EKF). </li>
  <li> Doing a fast update within the stabilization thread. </li>
  <li> Setup slower timing thread for the EKF gain update. </li>
  <li> Added library for matrix math functionality. </li>
  <li> Investigating Chol, LU, and QR factorizations for matrix inverse. </li>
  <li> TODO: Limited by matrix inverse calculation; currently fixed gain. </li>
  <li> TODO: Revise Jacobian (F matrix) at each update; currently static matrix. </li>
</ul>
<li><b> v0.1.6 - Revised AHRS code </b></li>
<ul>
  <li> Merged AHRS into IMU source code. </li>
  <li> Added both 6DOF (gyr/acc) and 9DOF (gyr/acc/mag) solutions. </li>
  <li> Includes only one tuning (weighting) parameter. </li>
  <li> Added complimentary filters for roll and pitch states. </li>
  <li> Crude state estimation is the average of all att and ang states. </li>
</ul>
<li><b> v0.1.5 - Signal filtering and parameter tuning </b></li>
<ul>
  <li> Revised source code and header file structure. </li>
  <li> Incorporated second IMU into the system. </li>
  <li> Set I2C bus to 200kHz until replacement resistors are available. </li>
  <li> Configured tunable control gains for quadrotor vehicle. </li>
  <li> Developed module for filter functionality.  </li>
  <li> Configured tunable filter settings from GCS. </li>
  <li> Log parameter changes made by GCS. </li>
  <li> Tuned LPF settings for small quad. </li>
</ul>
<li><b> v0.1.4 - Ground Control Station (GCS) </b></li>
<ul> 
  <li> Set up timing thread code for GCS and send generic msg </li>
  <li> Read transmission through external USB/FTDI cable </li>
  <li> Download and setup standard MavLink header code library </li>
  <li> Send heartbeat over UART </li>
  <li> Connect to QGC and read heartbeat in software </li>
  <li> Send state info to GCS </li>
  <li> Receive parameter updates from GCS </li>
</ul>
<li><b>v0.1.3 - GPS Receiver </b></li>
<ul>
  <li> Reading GPS through UART port. </li>
  <li> Added code to configure GPS parameters. </li>
  <li> Running checksum to verify accurate message. </li>
  <li> Logging GPS message data. </li>
</ul>
<li><b> v0.1.2 - Configure serial UARTS </b></li>
<ul>
  <li> Enabled four UART channels within DTO. </li>
  <li> Added new header and source code files for UART functionality. </li>
  <li> Incorporated new timing threads. </li>
  <li> Enable specific UARTs as needed. </li>
</ul>
<li><b> v0.1.1 - Second I2C channel </b></li>
<ul>
  <li> Revised MPU code directory structure. </li>
  <li> Revised MPU function calls to accept second I2C channel. </li>
  <li> Implemented custom I2C code framework. </li>
  <li> Added second IMU sensor. </li>
  <li> Added code to toggle which IMUs are operational. </li>
</ul>
</ul>


v0.1.0 - Operational system
---------------------------
This software version focused on making the BlackBox system operational
as an avionics board.  It includes the bare essentials needed to get a 
simple quadrotor system into the air.

<ul>
<li><b> v0.0.5 - Controller framework </b></li>
<ul>
  <li> Added new timing thread for control loop </li>
  <li> Developed a "pass through" one-to-one I/O mapping for debugging </li>
  <li> Added control data structure </li>
  <li> Tested the disarm feature with system output </li>
  <li> Implemented first pass PID control law for quadrotor </li>
  <li> Enableed datalog for control parameters </li>
</ul>
<li><b> V0.0.4 - Attitude and heading reference system </b></li>
<ul>
  <li> Added new timing thread for AHR </li>
  <li> Added AHR data structure </li> 
  <li> Evaluated LPF algorithm </li>
  <li> Added 9DOF data fusion algorithm </li>
  <li> Enabled AHR datalogging </li>
  <li> Calibrated IMU/AHR settings </li>
</ul>
<li><b> V0.0.3 - Program execution flags </b></li>
<ul>
  <li> New timing thread for program execution flags </li>
  <li> Register stick endpoint positions </li>
  <li> Counter records hold duration </li> 
  <li> Reset counters once stick condition is released </li>
  <li> Trips boolean flags for stick positions </li> 
  <li> Enable/Disable datalog </li>
  <li> Disable IMU and other loops during data download </li>
  <li> Arm/Disarm system outputs </li>
  <li> Exit program through radio commands </li>
</ul>
<li><b> V0.0.2 - System input and output signals </b></li>
<ul>
  <li> New timing thread for I/O signals </li>
  <li> Developed data structure for I/O signals </li>
  <li> Included datalog system I/O signals </li>
  <li> Configured system for PRU capabilities </li>
  <li> Enabled PRU-based PWM inputs </li>
  <li> Enabled PRU-based PWM outputs </li>
  <li> Added mutex conditions </li>
  <li> Enabled I2C in DTO </li>
  <li> Confirmed signals on scope </li>
  <li> Adjusted PWM loop value to achieve 400Hz </li>
</ul>
<li><b> V0.0.1 - Real time multithreading development </b></li>
<ul>
  <li> Linux beaglebone 4.1.15-ti-rt-r41 SMP PREEMPT RT armv7l </li>
  <li> Configured high speed 400kHz I2C timer </li>
  <li> Program exit condition </li>
  <li> Establish real time priority </li>
  <li> Lock and reserve memeory </li>
  <li> Setup multithread for 'debug' and 'imu' systems </li> 
  <li> Established mutex conditions for data protection </li>
  <li> Enable single IMU sensor </li>
  <li> Read calibration data from file </li>
  <li> Read gyr/acc at 1000hz, and mag at 100 hz </li>
  <li> Low pass filter on IMU data </li>
  <li> Scale raw IMU data to physical values </li> 
  <li> Setup crude initial datalogging system </li>
  <li> GPIO functions to toggle pins </li>
  <li> LED functions to manipulate user LEDs </li>
</ul>
</ul>




Pending Work
------------
The following list outlines some proposed upcoming change.

<b> Other tasks </b>
<ul>
<li><b> Top Priority </b></li>
<ul>
  <li> Develop more sophisticated quad controllers </li>
  <li> Add safety features </li>
</ul>
<li><b> Controls </b></li>
<ul>
  <li> Multisine signal generator </li>
  <li> System ID quad </li>
  <li> SISO MRAC quad </li>
  <li> MIMO MRAC quad </li>
  <li> Add Kalman filter </li>
</ul>
<li><b> Safety Features </b></li>
<ul>
  <li> Transistor power distribution kill switch </li> 
  <li> Add health/status monitoring thread </li>
  <li> Add warning for missed timing on thread </li>
  <li> Revise certain functions to have int error return values </li>
</ul>
<li><b> Modularity </b></li>
<ul>
  <li> Abstract the controller to be applied to various vehicles with same structure </li>
  <li> Calibrated radio ranges </li>
  <li> Define statements to enable specific datalogs </li>
  <li> Log generation within a function </li>
  <li> Modular structure to allow for differnet vehicles </li>
  <li> Incorporate doxygen into code development </li>
</ul>
<li><b> IMU and I2C Improvements </b></li>
<ul>
  <li> VICON data comparision </li>
  <li> Replace hard coded gyr bias with saved parameters from a file </li>
  <li> Make I2C source/header code more modular </li>
  <li> Test pull up resistors on IMU breakout board </li>
  <li> IMU convergence on start up </li>
  <li> IMU indicator for successful convergence </li>
  <li> IMU interrupt when data is available </li>
  <li> Enable address 0x69 in IMU code </li>
  <li> Second IMU sensor on single I2C channel </li>
  <li> Independent/parallel I2C channel threads </li>
  <li> Add altitude sensor </li>
  <li> Incorporate altitude hold </li> 
  <li> Calculate and apply rotation transformation for accelerometer mounting errors </li>
</ul>
</ul>



