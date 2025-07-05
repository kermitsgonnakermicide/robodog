**Total Time: 25hrs**

# June 11th-12th: Fully CADed

Started on this project by CADing everything I needed for the dog.<br>
Decided _very early_ not to make any fancy enclosure — this robot is going to be constantly disassembled, hacked, and upgraded. Don't want to box myself in (literally).<br>

The base design is simple but functional:

- A central flat plate to mount all power electronics
- Two oval end plates for mounting the servo legs
- Holes for zip ties and cable routes (obviously essential engineering features)

Each leg is a 3DOF system using MG996R servos — not the best, but they get the job done and are _everywhere_.<br>
![leg](Assets/leg.png)

Also tried budgeting a full SLAM setup — Lidar + Pi 4 8GB — but it's tight:

- Printing cost: ₹6k
- Pi 4: ₹10k
- Misc electronics + batteries: ₹5k

Fairly happy with how the CAD turned out, though I **massively** underestimated how long it would take.  
**Time spent: 15hrs**

---

# June 13th - Schematic

Did the full schematic layout for wiring everything together.

The build doesn’t use a custom PCB (at least not yet), so this schematic was mostly to organize:

- 12x MG996R servos (3 per leg)
- 1x PCA9685 servo driver
- Pi 4 power via USB boost board
- GPS module via UART
- Optional second PCA driver for expansion

Used the schematic as a build reference, mainly so I don’t fry something expensive.<br>
![alt text](image.png)<br>
**Time Spent: 1hr**

---

# June 14th - Firmware Pt1

Started writing the IK solver. First step was just getting the math right:

- Solving inverse kinematics for a 3DOF leg
- Using basic trig (law of cosines/sines, planar assumptions)
- Sanity-checking paw reachability and angle limits

I built it as a simple Python module for now, just to get the math debugged. Later it’ll be integrated into a ROS node.

**Time Spent: 3hr**

---

# June 17th - Updated CAD and Finished Firmware

Added a lid! The barebones open-top chassis was good for dev, but I was worried about connector damage or shorting stuff on the metal servo cases. So: added a snap-on lid.<br>
![<alt text>](image-1.png)

Finished the Python-based firmware:

- IK engine is now stable and modular
- It supports per-leg calibration offsets
- Outputs angle commands in degrees (for PCA9685)

Next step: slap it into ROS.<br>

Also, I decided against using a PCB because of one simple reason: cost<BR>
If I were to make a PCB, i could make two different ones for two different purposes<br>

- One could be a power distribution PCB with the XY3006, just with an XT60 input but with PCBA costs and import duties, a 150 rupee xy3006 board + a cheap XT60 plug to wire thing looks much better in front of a 5k+ PCBA(not even including the 100%+ import duties(thanks, nirmala))
- Another one could be a PCA9685 carrier board with an I2C multiplexer but honestly I can slap both of those together for 15-20x cheaper because yet again(lets say it together), our lord and saviour, nirmala sitaraman<br>
  I do understand the point of highway is for us to do things like this but honestly it just seems irresponsible, and I think my project will have enough complexity with the ROS stuff coming up<br>
  **Time Spent: 4hr**

---

# June 17th-18th - ROS TIME BAYBEEEEEEEEEEE!

Wrapped up the ROS2 workspace! This was the big milestone. The whole project is now structured as 4 packages:

---

### **servo_driver**

The hardware abstraction layer.  
Written in `rclpy`, this node:

- Talks to one (or two) PCA9685 servo drivers via I2C
- Receives joint angle commands (in radians or degrees)
- Publishes current servo state (mostly for debugging)
- Has a watchdog system to stop movement if no command received

It exposes:

- `/set_joint_angle` service (per joint)
- `/set_leg_pose` topic (batch commands per leg)
- Optional `/enable_torque` toggle

---

### **leg_walker**

This is the brain of the walking system. It:

- Handles IK calculations per leg
- Has a gait generator (currently trot, more coming)
- Coordinates paw trajectories using phase offsets and timing
- Publishes joint angles to `servo_driver`
- Includes a `walk_controller` node to send commands like `walk_forward(speed)` or `rotate_left(angle)`

Uses a parameterized `robot.yaml` config to define joint lengths, servo orientations, and bounds.

---

### **slam_yplidar_x2**

Runs 2D SLAM using a YDLidar X2 sensor.

Launches:

- `ydlidar_ros2_driver` (reads Lidar via serial)
- `slam_toolbox` (in synchronous mode)
- Static transforms to `/base_link`
- RViz with a custom config

Map is built in real-time and saved to a `.pgm`+`.yaml` pair. Robot pose is published on `/slam_pose`.

---

### **slam_stereo_picams**

Alternate 3D SLAM method using two Raspberry Pi cameras as a stereo pair (synchronized via GPIO).  
Still very much a WIP, but:

- Captures stereo image streams using `picamera2`
- Uses OpenCV stereo depth maps (SGM block matcher)
- Optionally feeds depth + RGB into `rtabmap_ros`
- Will be used for outdoor SLAM and low-light mapping

This branch will probably get moved to a Jetson board for performance.

---

![alt text>](image-2.png)<br>
**Time Spent: 2hr**
