### Turtlebot3 Lane Follower

**Project:** Turtlebot3 Lane follower using ROS Framework C++/OpenCV 

<p align="center">
  <img width="600" src="https://user-images.githubusercontent.com/74437465/191564552-0e59faa8-d2e2-4a07-bbd6-42489188b660.png">
</p>
This project consists into detect two different color lines (the lane edges) by using on-board camera and Computer Vision (OpenCV).

First this ROS package is being tested on a custom **Gazebo** environment which tries to represent the real case (in the following image)

<p align="center">
  <img width="350" src="https://user-images.githubusercontent.com/74437465/191564727-938a9f99-c94a-4f25-a7c5-465f9ffa2d6a.jpg">
  <img width="350" src="https://user-images.githubusercontent.com/74437465/191564759-cdeb6764-9912-4dd3-82b4-88d5a3bf514d.png">
  
</p>
In virtual enviroment the two lines are colored Green and Red,while in the rieal case Yellow and Red, so the center of the robot, need to keep as close as possible to the two line mid-distance.
Once tested on virtual enviroment, by connecting via SSH to the Robot, we can build and run our custom package.

It is possible to simulate also the onboard camera inside the virtual environment (see image below)
![centroid](https://user-images.githubusercontent.com/74437465/191564797-9a7f0386-1e5b-4ad9-a468-4b69f568a7a8.png)
![rotated_img](https://user-images.githubusercontent.com/74437465/191564806-371cc4ac-d9e4-4d73-aed9-9bac6a5a14f1.png)


https://user-images.githubusercontent.com/74437465/191570681-b555537c-8454-42f5-9bd4-28911756f4f8.mp4

