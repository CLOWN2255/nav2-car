#  1.Background of the problem 
##  During autonomous navigation, the trolley on the Gazebo does not rotate, while the trolley and radar on the RVIZ2 rotate in parallel. Please see the [video](https://github.com/CLOWN2255/nav2-car/blob/main/problem.webm) for details 
#  2. The cause of the problem
## gz::sim::systems::DiffDrive odom topics and tf topics posted in DiffDrive do not correspond to the actual rotation of the trolley on the gazebo platform.
## rviz2 receives the odom topic and tf topic published in gz::sim::systems::D iffDrive, and it rotates smoothly. DiffDrive doesn't check the actual selection on gazebo.
## This problem occurs when the trolley rotates smoothly on the RVIZ2, because there is no rotation on the Gazebo and the SCAN topic received by the RVIZ2 does not change.
## When there is a deviation in the odometer, the AMCL will accumulate the deviation of the odometer, which will eventually cause the autonomous navigation to crash.
#  3. Solution 
## Reduce the mass and inertia of the main body and four wheels to about the same speed as the odometer published by gz::sim::systems::D iffDrive
#  4. Revise the results
(https://github.com/CLOWN2255/nav2-car/blob/main/success.webm)
