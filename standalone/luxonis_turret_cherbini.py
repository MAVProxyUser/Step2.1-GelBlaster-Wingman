import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import HiwonderServoController as servo
import os
import time
import pygame
import cv2
import numpy as np
from pathlib import Path
import depthai as dai
import argparse

# Setup the HiWonder Servo
servo.setConfig('/dev/ttyUSB0', 1)
g = [1, 5]
tilt, pan = g

# Setup GPIO
os.system('echo 20 > /sys/class/gpio/export')
os.system('echo out > /sys/class/gpio/gpio20/direction')

## Query and Display current servo position
#boot_pos = servo.multServoPosRead(g)
##print("Boot Up Servo Position: Tilt/Pan")
#print(boot_pos[1], boot_pos[5])

# Set home position of servos
home_pan = 530
home_tilt = 500

def xy_home():
    servo.moveServo(tilt, home_pan, 500)
    servo.moveServo(pan, home_tilt, 500)

    time.sleep(0.5)

    global boot_pos
    boot_pos = servo.multServoPosRead(g)
    print("Boot Up Servo Position: Tilt/Pan")
    print(boot_pos[1], boot_pos[5])

# Test Pan Sweep
def test_pan():
    for i in range(300, 700):
         servo.moveServo(pan, i, 500)

# Test Tilt
def test_tilt():
    for i in range(400, 500):
         servo.moveServo(tilt, i, 500)

def start_fire():
    print("start firing")
    os.system('echo 1 > /sys/class/gpio/gpio20/value')

def stop_fire():
    print("stop firing")
    os.system('echo 0 > /sys/class/gpio/gpio20/value')

xy_home()
test_pan()

pan_max = 750
pan_min = 300
tilt_max = 650
tilt_min = 500

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.get_logger().info('Joystick controller node started')
        self.pan_increment = 5  # Increment in degrees for pan
        self.tilt_increment = 5  # Increment in degrees for tilt
        self.deadzone = 0.2  # Deadzone for joystick input
        self.current_pan = home_pan  # Starting position of pan
        self.current_tilt = home_tilt  # Starting position of tilt
        # PID controller variables
        self.pan_error_integral = 0
        self.pan_last_error = 0
        self.tilt_error_integral = 0
        self.tilt_last_error = 0
        self.pan_kp = 0.3
        self.pan_ki = 0.000
        self.pan_kd = 0.8
        self.tilt_kp = 0.3 
        self.tilt_ki = 0.000
        self.tilt_kd = 0.8
        self.axis_data = {}
        self.button_data = {}
        self.hat_data = {}
        self.pan_pub = self.create_publisher(Float64, '/pan_controller/command', 10)
        self.tilt_pub = self.create_publisher(Float64, '/tilt_controller/command', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        """Listen for joystick events"""
    
        normalized_pan = msg.axes[2] * -1
        normalized_tilt = msg.axes[5]
    
        # Scale increments by the difference between max and min positions
        pan_scaled_increment = (pan_max - pan_min) * (self.pan_increment / 180)
        tilt_scaled_increment = (tilt_max - tilt_min) * (self.tilt_increment / 180)
    
        target_pan = self.current_pan + (normalized_pan * pan_scaled_increment)
        target_pan = min(max(target_pan, pan_min), pan_max)
    
        target_tilt = self.current_tilt + (normalized_tilt * tilt_scaled_increment)
        target_tilt = min(max(target_tilt, tilt_min), tilt_max)
    
        # PID controller for pan
        pan_error = target_pan - self.current_pan
        self.pan_error_integral += pan_error
        pan_error_derivative = pan_error - self.pan_last_error
        pan_output = self.pan_kp * pan_error + self.pan_ki * self.pan_error_integral + self.pan_kd * pan_error_derivative
        self.pan_last_error = pan_error
    
        self.current_pan += pan_output
        self.pan_pub.publish(Float64(data=float(self.current_pan)))
        servo.moveServo(pan, int(self.current_pan), 1)
        time.sleep(0.01)
    
        # PID controller for tilt
        tilt_error = target_tilt - self.current_tilt
        self.tilt_error_integral += tilt_error
        tilt_error_derivative = tilt_error - self.tilt_last_error
        tilt_output = self.tilt_kp * tilt_error + self.tilt_ki * self.tilt_error_integral + self.tilt_kd * tilt_error_derivative
        self.tilt_last_error = tilt_error
    
        self.current_tilt += tilt_output
        self.tilt_pub.publish(Float64(data=float(self.current_tilt)))
        servo.moveServo(tilt, int(self.current_tilt), 1)
        time.sleep(0.01)

    def move_servos_with_error(self, errorX, errorY):

        if abs(errorX) >= 60 or abs(errorY) >= 60:
            # Scale increments by the difference between max and min positions
            pan_scaled_increment = (pan_max - pan_min) * (self.pan_increment / 180)
            tilt_scaled_increment = (tilt_max - tilt_min) * (self.tilt_increment / 180)
        
            # Calculate target pan and tilt based on incoming error
            target_pan = self.current_pan + (errorX * pan_scaled_increment)
            target_tilt = self.current_tilt + (errorY * tilt_scaled_increment)
        
            # Ensure target positions are within limits
            target_pan = min(max(target_pan, pan_min), pan_max)
            target_tilt = min(max(target_tilt, tilt_min), tilt_max)
        
            # PID controller for pan
            pan_error = target_pan - self.current_pan
            self.pan_error_integral += pan_error
            pan_error_derivative = pan_error - self.pan_last_error
            pan_output = self.pan_kp * pan_error + self.pan_ki * self.pan_error_integral + self.pan_kd * pan_error_derivative
            self.pan_last_error = pan_error
        
            self.current_pan += pan_output
            self.pan_pub.publish(Float64(data=float(self.current_pan)))
            servo.moveServo(pan, int(self.current_pan), 2000)
            time.sleep(0.05)
        
            # PID controller for tilt
            tilt_error = target_tilt - self.current_tilt
            self.tilt_error_integral += tilt_error
            tilt_error_derivative = tilt_error - self.tilt_last_error
            tilt_output = self.tilt_kp * tilt_error + self.tilt_ki * self.tilt_error_integral + self.tilt_kd * tilt_error_derivative
            self.tilt_last_error = tilt_error
        
            self.current_tilt += tilt_output

            antinutshot = 20
            self.current_tilt += antinutshot

            self.tilt_pub.publish(Float64(data=float(self.current_tilt)))
            servo.moveServo(tilt, int(self.current_tilt), 2000)
            time.sleep(0.05)
        
            # Are we close?
            if abs(errorX) < 30 and abs(errorY) < 30:
                start_fire()
                time.sleep(.25)
                stop_fire()
        
            # Show where we're gonna go
            print("Here's where the servos are going:")
            print(int(self.current_pan), int(self.current_tilt))


#labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
labelMap = ["person"]

nnPathDefault = str((Path(__file__).parent / Path('examples/models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)
parser.add_argument('-ff', '--full_frame', action="store_true", help="Perform tracking on full RGB frame", default=False)

args = parser.parse_args()

fullFrameTracking = args.full_frame

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
detectionNetwork = pipeline.create(dai.node.MobileNetDetectionNetwork)
objectTracker = pipeline.create(dai.node.ObjectTracker)

xlinkOut = pipeline.create(dai.node.XLinkOut)
trackerOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("preview")
trackerOut.setStreamName("tracklets")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(40)

# testing MobileNet DetectionNetwork
detectionNetwork.setBlobPath(args.nnPath)
detectionNetwork.setConfidenceThreshold(0.85)
detectionNetwork.input.setBlocking(False)

objectTracker.setDetectionLabelsToTrack([15])  # track only person
# possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
# take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
#objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)

# Linking
camRgb.preview.link(detectionNetwork.input)
objectTracker.passthroughTrackerFrame.link(xlinkOut.input)

if fullFrameTracking:
    camRgb.video.link(objectTracker.inputTrackerFrame)
else:
    detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
detectionNetwork.out.link(objectTracker.inputDetections)
objectTracker.out.link(trackerOut.input)

# Connect to device and start pipeline
def detection_process(joystick_controller):
    with dai.Device(pipeline) as device:
    
        preview = device.getOutputQueue("preview", 4, False)
        tracklets = device.getOutputQueue("tracklets", 4, False)
        
        startTime = time.monotonic()
        counter = 0
        fps = 0
        frame = None
        
        while(True):
            imgFrame = preview.get()
            track = tracklets.get()
        
            counter+=1
            current_time = time.monotonic()
            if (current_time - startTime) > 1 :
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time
        
            color = (255, 0, 0)
            frame = imgFrame.getCvFrame()
            trackletsData = track.tracklets
            for t in trackletsData:
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)
        
                try:
                    label = labelMap[t.label]
                except:
                    label = t.label
                
                hitX = int((x1 + x2) / 2)
                hitY = int((y1 + y2) / 2)

                centerX = 150
                centerY = 150

                errorX = centerX - hitX
                errorY = centerY - hitY

                joystick_controller.move_servos_with_error(errorX, errorY)
                print("detect moving to: ")
                print(int(hitX), int(hitY))

                dot_radius = 4
                dot_color = (0,255,0) #green
                #cv2.circle(frame, (hitX, hitY), dot_radius, dot_color, -1)
                #cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                #cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                #cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                #cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
        
            #cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        
            #cv2.imshow("tracker", frame)
        
            #if cv2.waitKey(1) == ord('q'):
            #    break

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()

    xy_home()

    # Call the detection_process function directly
    detection_process(joystick_controller)

    # Run the joystick controller
    rclpy.spin(joystick_controller)

    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

