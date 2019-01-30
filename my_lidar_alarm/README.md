# lidar_alarm

A simple, illustrative program to show how to subscribe to a LIDAR signal and interpret the signal.
This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator.
The signal interpretation in this example merely looks at a single ping--straight ahead.  If this
ping distance is less than some danger threshold, the lidar listener publishers a warning signal on
topic `lidar_alarm`.  The distance of the forward ping is also published, on topic `lidar_dist`.

To make a rich and quick interpretation of LIDAR data, ping in front and pings that are +-15 degree, +-30 degree, +-45 degree, +-60 degree, +-75 degree and +-90 degree from front are used to check has collision or not. 

## Example usage
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the lidar alarm node:
 `rosrun my_lidar_alarm lidar_alarm`
 Have the controller code monitor the `lidar_alarm` topic and do something intelligent with the information.

    
