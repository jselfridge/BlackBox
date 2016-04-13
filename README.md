
BLACK BOX
=========

Develops a cape for a BeagleBone Black to enable "Black 
Box" functionality for research and development.


Current Tasks
-------------
This software version focuses on expanding the minimum working 
software set, to include features that are needed for autonomous 
navigation capabilities.

<b> Component Integration and Bench Testing </b>
<ul>
  <li> Incorporated second IMU into the system. </li>
  <li> Configured tunable control gains for quadrotor vehicle. </li>
  <li> (WIP) Log changes to the system parameters. </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
  <li> (WIP) </li>
</ul>


v0.1.4 - Ground Control Station
-------------------------------
<ul> 
  <li> Set up timing thread code for GCS and send generic msg </li>
  <li> Read transmission through external USB/FTDI cable </li>
  <li> Download and setup standard MavLink header code library </li>
  <li> Send heartbeat over UART </li>
  <li> Connect to QGC and read heartbeat in software </li>
  <li> Send state info to GCS </li>
  <li> Receive parameter updates from GCS </li>
</ul>


v0.1.3 - GPS Receiver
---------------------
<ul>
  <li> Reading GPS through UART port. </li>
  <li> Added code to configure GPS parameters. </li>
  <li> Running checksum to verify accurate message. </li>
  <li> Logging GPS message data. </li>
</ul>


v0.1.2 - Configure serial UARTS
-------------------------------
<ul>
  <li> Enabled four UART channels within DTO. </li>
  <li> Added new header and source code files for UART functionality. </li>
  <li> Incorporated new timing threads. </li>
  <li> Enable specific UARTs as needed. </li>
</ul>


v0.1.1 - Second I2C channel
---------------------------
<ul>
  <li> Revised MPU code directory structure. </li>
  <li> Revised MPU function calls to accept second I2C channel. </li>
  <li> Implemented custom I2C code framework. </li>
  <li> Added second IMU sensor. </li>
  <li> Added code to toggle which IMUs are operational. </li>
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

<ul>
<li><b> Top Priority </b></li>
<ul>
  <li> Revise source/header code structure </li>
  <li> Add notes section to document parameter changes from GCS </li>
  <li> Add filtering to signals </li>
  <li> Develop more sophisticated quad controllers </li>
  <li> Add safety features </li>
</ul>
<li><b> Filtering </b></li>
<ul>
  <li> Dedicated filter source code </li>
  <li> Low pass filter function <li>
  <li> Butterworth filter function </li>
  <li> Selectable filter from compiler settings <li>
  <li> Tunable filter gains from GCS </li>
  <li> Add Kalman filter </li>
</ul>
<li><b> Controls </b></li>
<ul>
  <li> Generic PID function <li>
  <li> Generic state feedback (SISO) functions <li>
  <li> Generic state feedback (MIMO) functions <li>
  <li> Multisine signal generator <li>
  <li> System ID quad </li>
  <li> SISO MRAC quad </li>
  <li> MIMO MRAC quad </li>
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



