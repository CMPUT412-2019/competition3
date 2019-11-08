# Competition 3

The project shares much of its code with the previous competition, [competition 2](https://github.com/CMPUT412-2019/cmput412-competition2). As such, not all details of our algorithms and approach appear here, as we sometimes refer back to the documentation of that competition. In addition, occasional images and text within this Readme are sourced from the Readme from that competition.

## Setup and building

As before, the project requires ROS Kinetic with the various turtlebot and openni packages detailed on eClass. In addition, the project requires

  * [gTTs](https://github.com/pndurette/gTTS) to generate sounds
    ```shell script
    pip install gTTS
    ```
  
  * The RGB-D camera in its default position on the robot, facing forward:
  ![](images/rgbd.jpg)
  
  * A webcam at the bottom front of the robot, looking down:
  ![](images/usb-cam.jpg)
  (the code assumes this webcam corresponds with the device `/dev/video2`)
  
Once you have the requirements, download the source from the [release](https://github.com/CMPUT412-2019/competition3/releases/tag/1.0.0) on Github. Unpack it to `competition3`, and run

    cd competition3
    catkin build
    source devel/setup.bash

to build the project. If you don't have `catkin build`, use `catkin_make` instead.

As in the previous competition, you now need to generate sound files:

    cd sound
    python gen.py
    cd ..

This will take a while. Please be patient.


You now need to download the pre-built map (called `rtabmap.db`) from the [release](https://github.com/CMPUT412-2019/competition3/releases/tag/1.0.0). Put it into the [src/competition3/rtabmap/realworld](src/competition3/rtabmap/realworld) directory, so its new name is `src/competition3/rtabmap/realworld/rtabmap.db`