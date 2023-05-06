# **rUBot Custom model & control**

We have designed and assembles a custom mecanum robot.

The mechanical structure is descrived below:<br>
![](./Images/rubot_custom/1_osoyoo.png)

His main characteristics are: 
- Arduino based control for sensors & actuators
    - Servomotr actuartos for the 4 mecanum wheels
- As onboard controll it's used RaspberryPi4 (due to out of stock, can be used other onboard controller such as ROCK PI 4)
    - RPlidar distance sensor
    - Two logitech C270 to capture the image and realize the line following and traffic sign detection

In this document we will describe:
- Software needed to start.
- Bringup SW and HW
- Kinematic Movement Control
- Self-Navigation
- Wall follower
- Line follower
- Traffic signal identification


## **1. Software needed to start**

First, you'll need to install the necessary software on your computer. We will work with Docker Desktop. This software will allow us to emulate an Ubuntu SO on our computer running Windows or MAC.
Then we can download the sofware from the official website:  [Docker Desktop - Download](https://www.docker.com/products/docker-desktop/).

Once downloaed, we have to download the image of Docker (a package that contains everything needed to run the application in different environments) and configure the contariner.
- Open Docker Desktop
- Go to the top search bar (or press "CTRL+K" on windows, "command+K" on MAC)
- Type <b>arvinskushwaha/ros-noetic-desktop-vnc</b> and select "Images" below the search bar, then donwload the image<br><img src="./Images/rubot_custom/docker_0.png" alt="Texto alternativo" width="800">
- After downloaded, click on Images in the left-hand side menu (second option on the previous image) and click on the Play button to mount the image onto a Docker container<br><img src="./Images/rubot_custom/docker_1.png" alt="Texto alternativo" width="600">
- Below are the cointainer settings:
    - <b>Container name:</b> Name that our cointainer will have. Spaces and special characters are not allowed.
    - <b>Host port:</b> By default we will use 80.
    - <b>Host path:</b> You should have downloaded rUBot_mecanum_ws, if not download it from [THIS LINK](https://github.com/manelpuig/rUBot_mecanum_ws/archive/refs/heads/master.zip), unzip and save it to your computer. <u>Try to save it to a local folder, rather than a folder that syncs with onedrive.</u> For example: C:\Users\uri\Desktop\rUBot_tutorioal_ws
    - <b>Container path:</b> Directory where we will have it synchronised with our WS in Windows. By default /home/ubuntu/rUBot_mecanum_ws<br><img src="./Images/rubot_custom/docker_2.png" alt="Texto alternativo" width="600">
- At this point 







"rubot_mecanum_description" package where we will create the rUBot model. In case you want to create it from scratch, type:
```shell
cd ~/Desktop/ROS_github/rubot_mecanum_ws/src
catkin_create_pkg rubot_mecanum_description rospy
cd ..
catkin_make
```