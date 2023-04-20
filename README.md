# DDBOAT python3 drivers version 2 - fork modified by morgan

The original drivers are in the folder drivers_ddboat_v2
* IMU (MAG, ACCEL, GYRO) : imu9_driver_v2.py
* GPS (serial line, GPGLL message) : gps_driver_v2.py
* Encoders on propeller rotation (serial line) : encoders_driver_v2.py
* Arduino motors command (serial line) : arduino_driver_v2.py
* TC74 temperature sensors (one per motor) : tc74_driver_v2.py

In addition of the driver, I have developed the following tools in the folder lib:
* controllers : DDBOAT_controller_v2.py
* filters : DDBOAT_filter_v2.py
* mission script reader  DDBOAT_mission_v2.py
* log writer : DDBOAT_log_v2.py

In the missions folder, you can run the mission main files:
* M0 - the robot return to home position
* M1 - the robot follow a list of predefined lines
* M2 - the robot move in a square by timing the desired heading
* M3 - the robot follow a list of waypoints defined in the mission script
* M4 - ???
* M5 - the circle consensus
Mission instruction are descibed in : mission_script/mission_script.json

Additional tests:
* calibration of the compass : Tool_compass_calibration.py
* test the compass calibration  Tool_test_compass.py
* display heading and local global position : Tool_scout.py
* set the date manually : Tool_set_date.py
* test the motor control loop : Tool_test_motor_control_loop.py

Compass parameters in compass_calibration/
