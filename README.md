# ROS Doggo

A quadruped robot platform running ROS2 Humble on a Raspberry Pi 4 with 3DOF legs and an optional SLAM capability via either Lidar and stereo vision.<br>

**Design**<br>
I started off by CADing the entire thing, keeping in mind that this will be an actively developed platform — no fancy covers, just raw plate and servo mounts.<br>
Went with a central plate that houses all the power distribution + electronics, and simple oval-style end mounts for the legs.<br>
The legs are 3DOF and use MG996R servos — cheap and decently powerful<br>
![leg](Assets/leg.png)<br>

Later added a lid to keep things _mostly_ protected.<br>
![<alt text>](image-1.png)<br>

**Electronics**<br>
I made a custom, 63 by 48 mm PCBA that has 5 major functions:<BR>

- step down power from my 4s 14.8v lipo battery to 5v<br>
- step up power from 5v to 7v to effectively power my servoes
- output that 5v power to my raspi on USB<br>
- control my servoes with a PCA9685<br>
- one of the best IMUs, the BNO086 for optimizing gait<br>
  ![alt text](image-4.png)
  ![alt](img.png)
  ![alt text](image-3.png)

**Firmware**<br>
**ROS Packages**  
The whole ROS workspace is structured into 4 main packages:

- **servo_driver**  
  The hardware interface layer. This node communicates with one or more PCA9685 boards via I2C to send position signals to each servo motor. It exposes a simple ROS2 service + topic interface that allows other nodes to command joint angles (in degrees or radians). It also implements basic limits, deadzone filtering, and optional torque hold toggling. Each servo is mapped to a specific joint name and leg index.

- **leg_walker**  
  This package does all the IK heavy lifting. It:

  - Subscribes to target paw positions (either via a `/cmd_paw_pos` topic or internal walking scripts)
  - Solves 3DOF inverse kinematics per leg (coxa, femur, tibia)
  - Publishes joint angles to the `servo_driver`
  - Supports gait generation — currently just a trot gait, but it's modular
  - Includes walking state machines, leg timing, and paw trajectory interpolation
    This is where all the math happens.

- **slam_yplidar_x2**  
  SLAM package using the YDLidar X2. It launches:

  - A serial connection to the Lidar via `ydlidar_ros2_driver`
  - `slam_toolbox` in synchronous mode
  - RViz config for map + robot visualization
    Outputs a 2D occupancy map and robot pose estimate. Currently uses fake odom + map merging but will be upgraded later with IMU + gait odometry.

- **slam_stereo_picams**  
  Alternate SLAM method using two Raspberry Pi cameras as a stereo pair (synced via GPIO trigger). It:
  - Captures stereo image pairs
  - Runs semi-global block matching or other stereo algorithm (WIP: OpenCV + rtabmap)
  - Feeds into `rtabmap_ros` for visual SLAM
  - Outputs a 3D point cloud and pose estimate
    This is mostly experimental right now but promising for GPS-denied areas and outdoor mapping.

![alt text>](image-2.png)<br>

**NOTE**: Reusing the lipo battery from a diff project<br>

**BOM**
|ITEM |QTY|TOTAL |SRC |NOTES |
|-----------------------|---|------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
|Raspberry Pi 5 8GB |1 |94.82 |https://www.silverlineelectronics.in/collections/raspberry-pi-5-official-accessories/products/raspberry-pi-5-model-b-8gb-ram |I require an 8GB because I dont just need to use ROS - within it, I also need to do SLAM(even more computationally expensive)|
|RPi 5 Active Cooler |1 |5.27 |https://www.silverlineelectronics.in/collections/raspberry-pi-cooling-tech | |
|SD Card |1 |6.93 |https://www.amazon.in/SAMSUNG-Adaptor-Expanded-Storage-MB-MC64SA/dp/B0CXJ5Q222/ref=sr_1_3?crid=2JX27GGIJJTGI&dib=eyJ2IjoiMSJ9.W0mEzbMqa_UC3GB83swgkYea9f_3bRa4nMoh5j8uNEkChp42zWEkQpLMc0DnmWqmeu-AuuhK89UciyKu_5SFswRarCA-pBkkO60Q20oIBA_fkTCwYzRS_bEmYCP8yWifGQWiN7IS3pzrRxGHj_bR59EVNZ7nKRTTJQgCTymu6a5_LKu5mFX9u4Qt-aeNY8PHtSEAPnHDRGtQwz8awrEL67j54Ov63FOAKBYw6o2EqSo.z2cyXBa8m8xP1fLej5w40OuPwxTdFYv78CPP1mV_vY4&dib_tag=se&keywords=64gb+sd+card+evo&qid=1750190942&sprefix=64gb+sd+card+evo,aps,226&sr=8-3&th=1| |
|Waveshare Stereo Camera|1 |47.76 |https://hubtronics.in/imx219-83-stereo-camera | |
|YDLIDAR X2 |1 |60.99 |https://www.electronicscomp.com/ydlidar-x2-360-degree-ros-scanner-for-navigation-collision-avoidance-8m?srsltid=AfmBOorwLINheGIH6_5-cDKKJgGNAVWmLa5pv0aLsJ3wuyl8X8SN1hhzP3Q&gRefinements=SORT_BY:Price:+low+to+high | |
|PCBA |1 |129 |https://hc-cdn.hel1.your-objectstorage.com/s/v3/cd5c9a744fc6b144acc7592c6a4c4ebfb7888e05_image.png |LionCircuits |
|XT60PW-m |1 |1.91 |https://www.drkstore.in/amass-xt60pw-f20-connector/ |Not available to procure on LionCircuits, also including delivery |
|USB A |1 |0.17 |https://robu.in/product/u221-041n-1wr69-s5-xkb-1-5a-1-4p-female-40%e2%84%8385%e2%84%83-type-a-plugin-usb-connectors-rohs/ |Will cover myself, not available on LionCircuits |
|MG996R |12 |48.62 |https://sharvielectronics.com/product/mg996r-metal-gear-servo-motor/?srsltid=AfmBOoqSlyks-1TuudDgubd_qj9fNCwaQsQTs6MFmUkCxiBuGM9Gln68fNU | |
| | | | | |
| | | | | |
| | |395.47| | |
**NOTE: Will also cover the MG996s myself to ensure that i'm under budget**
