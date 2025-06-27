**Total Time: 25hrs**

# June 11th-12th: Fully CADed

Started on this project by CADing everything i needed for the dog<br>
Decided to not make any crazy covers for anything, as this will be a heavily developed robot and I dont want to encumber myself with that<br>
Went with a plate in the middle and a simple two oval thing at each end for mounting the servoes<br>
On the plate, I intend to put all the power circuitry()
the leg will be 3DOF, with mg996Rs chosen for their(relative) power and cheapness<br>
![leg](Assets/leg.png)
I want to see if I can fit an RPi Lidar into the budget, but it looks unlikely as just printing this will cost me 6k, a 8gb pi will cost me 10k(as I want to run ROS on this doggo) assorted electronics and battery will cost me 5k<br>
Anyways, I'm fairly sure im done with cadding this bad boi, but it took me WAYYYY too long(i'm 3 redbulls deep)
**Time spent: 15hrs**

# June 13th - Schematic

Made a schematic for this doggo, as I will not be using a PCB<br>
The schematic contains 12 servoes, one pca9685 driver, the USB power board for the Pi and a GPS module<br>
![alt text](image.png)<br>
**Time Spent: 1hr**

# June 14th - Firmware Pt1

Started on the IK solver for this doggo. Had to do some research on how to actually(properly) do inverse kinematics(math on how to move the joints to move the paw to a specific point)

**Time Spent: 3hr**

# June 17th - Updated Cad and finished firmware

Added a lid to the dog, to protecc the dog. Also, my firmware is pretty much finished, and i'll upload the ROS stuff today<br>
![<alt text>](image-1.png)<br>
**Time Spent: 4hr**

# June 17th-18th - ROS TIME BAYBEEEEEEEEEEE!

Finally finished my ros packages for the doggo. The project can be split into 4 distinct packages:<br>
**servo_driver**:the 'hardware interface' for this. Runs the dual PCA9685 drivers required for our doggo<br>
**leg_walker**: does the actual heavy lifting for IK<br>
**slam_yplidar_x2**: uses the slam_toolbox for(you guessed it) SLAM, also known as simultaneous localization and mapping. basically, the missile needs to know where it is but dog<br>
![alt text>](image-2.png)<br>
**slam_stereo_picams**: another way to do slam, with a jetson stereo cam<br>
**Time Spent: 2hr**
