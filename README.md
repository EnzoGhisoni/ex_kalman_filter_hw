# ex_kalman_filter_hw

Implementation of an Extended Kalman Filter in python using ROS2 foxy.

## Package content

- The node republish_bag.py is used to perform preprocessing on arriving data, compute dead-reckoning and format the gps_data into an odometry message in the same frame reference.
- The node ex_kalman_filter.py performs the computation of the extended kalman filter.
- A launch file is implemented to start the nodes and the rviz visualization with the configuration already setup.
