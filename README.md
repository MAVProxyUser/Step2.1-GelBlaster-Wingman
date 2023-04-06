# Step2.1-GelBlaster-Wingman
Step 2.1 version of the Gelblaster AI Turret

Expects Robotis Dynamixel 2.0 firmware on a Pan & Tilt servo with ID 1 & 2
https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/

[![protocol 2.0](http://img.youtube.com/vi/aVZytXRc_r8/0.jpg)](https://www.youtube.com/watch?v=aVZytXRc_r8)<br>

Also ensure the protocol speed is set to 3 to match the yaml config

```
$ cat ~/robotis_turret_ws/src/interbotix_ros_turrets/interbotix_ros_xsturrets/interbotix_xsturret_control/config/vxxms.yaml
port: /dev/ttyDXL

joint_order: [pan, tilt]
sleep_positions: [0, 0]

joint_state_publisher:
  update_rate: 100
  publish_states: true
  topic_name: joint_states

groups:
  turret: [pan, tilt]

motors:
  pan:
    ID: 1
    Baud_Rate: 3
    Return_Delay_Time: 0
    Drive_Mode: 0
    Velocity_Limit: 131
    Min_Position_Limit: 0
    Max_Position_Limit: 4095
    Secondary_ID: 255

  tilt:
    ID: 2
    Baud_Rate: 3
    Return_Delay_Time: 0
    Drive_Mode: 1
    Velocity_Limit: 131
    Min_Position_Limit: 1020
    Max_Position_Limit: 3076
    Secondary_ID: 255
```
