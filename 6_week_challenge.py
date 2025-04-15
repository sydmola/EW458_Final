import time
import roslibpy
import pygame
from lights_function import play_lights
import threading
import math

# Initialize pygame and joystick control
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a joystick and restart.")
    exit(1)

# Connect to the ROS bridge
ros_node = roslibpy.Ros(host='192.168.8.104', port=9012)
ros_node.run()

robot_name = 'foxtrot'

# Joystick class
class Joystick:
    def __init__(self):
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.stop_event = threading.Event()

        # State variables
        self.manual_mode = False
        self.idle_mode = True
        self.autonomous_mode = False
        self.armed = False
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.color = 'Off'
        self.blink = False

        # Create and start thread
        self.thread = threading.Thread(target=self.get_commands, daemon=True)
        self.thread.start()

    def get_commands(self):
        while not self.stop_event.is_set():
            pygame.event.pump()  # Process joystick events

            if self.joystick.get_button(0):  # "A" button
                self.manual_mode = not self.manual_mode
                self.idle_mode = False
                self.autonomous_mode = False
                print(f"Manual mode {'activated' if self.manual_mode else 'deactivated'}")
                time.sleep(0.3)  # Debounce delay

            if self.joystick.get_button(2):  # "X" button
                self.idle_mode = not self.idle_mode
                self.manual_mode = False
                self.autonomous_mode = False
                print(f"Secondary mode {'activated' if self.idle_mode else 'deactivated'}")
                time.sleep(0.3)

            if self.joystick.get_button(1):  # "B" button
                self.autonomous_mode = not self.autonomous_mode
                self.manual_mode = False
                self.idle_mode = False
                print(f"Autonomous mode {'activated' if self.autonomous_mode else 'deactivated'}")
                time.sleep(0.3)

            if self.joystick.get_button(4):  # Left bumper
                self.armed = not self.armed
                print(f"Robot {'armed' if self.armed else 'disarmed'}")
                time.sleep(0.3)

            # Determine movement & LED state
            if self.manual_mode:
                self.linear_x = -self.joystick.get_axis(1)  # Invert Y-axis for forward/backward
                self.angular_z = -self.joystick.get_axis(0)  # X-axis for rotation
                self.color = 'Green'
            elif self.idle_mode:
                self.linear_x = 0.0
                self.angular_z = 0.0
                self.color = 'Blue'
            elif self.autonomous_mode:
                self.color = 'Yellow'

            self.blink = self.armed  # Blink if armed
            if self.armed == False:
                self.linear_x = 0
                self.angular_z = 0

            time.sleep(0.1)  # Loop at 5 Hz

    def stop(self):
        self.stop_event.set()
        self.thread.join()
        self.joystick.quit()

# Robot class
class RobotController:
    def __init__(self, joystick):
        self.joystick = joystick
        self.stop_event = threading.Event()

        # ROS publishers
        self.led_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.drive_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')
        self.audio_pub = roslibpy.Topic(ros_node, f'/{robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')
        self.odom_topic = roslibpy.Topic(ros_node, f'/{robot_name}/odom', 'nav_msgs/Odometry')
        self.ir_topic = roslibpy.Topic(ros_node, f'/{robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
        self.odom_topic.subscribe(self.odom_callback)

        # Create and start threads
        self.drive_thread = threading.Thread(target=self.drive, daemon=True)
        self.led_thread = threading.Thread(target=self.leds, daemon=True)
        self.audio_thread = threading.Thread(target=self.audio, daemon=True)
        self.auto_mow_thread = threading.Thread(target=self.auto_mow, daemon=True)
        self.sense_ir_thread=threading.Thread(target=self.sense_ir, daemon=True)
        self.drive_thread.start()
        self.led_thread.start()
        self.audio_thread.start()
        self.auto_mow_thread.start()
        #self.sense_ir_thread.start()

    def odom_callback(self, message): #read odometer data
        self.latest_odom = {
            "position": message['pose']['pose']['position'],
            "orientation": message['pose']['pose']['orientation'],
            "linear_velocity": message['twist']['twist']['linear'],
            "angular_velocity": message['twist']['twist']['angular']
        }
    
    def get_odom(self):
        return self.latest_odom

    def ir_sensor(self):
        self.sense_ir_thread.start()
    
    def callback_ir(self, message):
        values = [reading['value'] for reading in message['readings']]
        front_value = values[3]  # Adjust index based on sensor configuration
        left_value = values[0]
        right_value = values[6]
        left_center= values[2]
        right_center=values[5]
        #error = left_value - right_value

        if front_value>10:
            print('object in FRONT')
            drive_message = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # Stop moving forward
                            'angular': {'x': 0.0, 'y': 0.0, 'z': -1.0}}  # Rotate right
            self.drive_pub.publish(roslibpy.Message(drive_message))
            
        elif left_center>10:    
            print('object in FRONT LEFT')
            drive_message = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # Stop moving forward
                            'angular': {'x': 0.0, 'y': 0.0, 'z': -1.0}}  # Rotate right
            self.drive_pub.publish(roslibpy.Message(drive_message))

        elif right_center>10:
            print('object in FRONT RIGHT')
            drive_message = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # Stop moving forward
                            'angular': {'x': 0.0, 'y': 0.0, 'z': 1.0}}  # Rotate right
            self.drive_pub.publish(roslibpy.Message(drive_message))
            
        elif left_value>10:
            print('object LEFT')
            
        elif right_value>10:
            print('object RIGHT')
        
        else:
            drive_message = {'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0},  # Stop moving forward
                                        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}  # Rotate right
            self.drive_pub.publish(roslibpy.Message(drive_message))

    def sense_ir(self):
        while not self.stop_event.is_set():
            if self.joystick.manual_mode == False:
                if self.joystick.armed == True:
                    self.ir_topic.subscribe(self.callback_ir)

    def drive_straight(self, dist):
        # retrieve data and set start position
        msg = self.get_odom()
        if msg:
            start_x = msg["position"]['x']
            start_y = msg["position"]['y']
        else:
            print('waiting for odometry data')

        position = 0.0
        # Drive straight for 1 meter
        while position < dist:
            drive_message = {'linear': {'x': 0.15, 'y': 0.0, 'z': 0.0},
                            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}} 
            self.drive_pub.publish(roslibpy.Message(drive_message)) #drive forward
            
            msg = self.get_odom() #retrieve data and calculate relative position
            odom_x = msg["position"]['x']
            odom_y = msg["position"]['y']
            position = math.sqrt((odom_x - start_x)**2 + ((odom_y - start_y)**2))
            print(position)

            time.sleep(0.1) #10Hz repetition
    
    def turn_right(self):
        # Right turn sequence
        print("Making right turn")
        # retrieve data and set start position
        msg = self.get_odom()
        if msg:
            start_angle = msg["orientation"]['z']
            if start_angle < 0:
                start_angle = start_angle+2
        else:
            print('waiting for odometry data')
        
        angle = 0.0
        while angle > -0.5:
            drive_message = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 
                            'angular': {'x': 0.0, 'y': 0.0, 'z': -0.5}}  # Rotate right
            self.drive_pub.publish(roslibpy.Message(drive_message))

            msg = self.get_odom() #retrieve data and calculate relative turn angle
            current_angle = msg["orientation"]['z']
            if current_angle < 0:
                current_angle = current_angle+2
            angle = current_angle-start_angle
            if 1.5<angle<2:
                angle = angle-2 
            print(angle)
            time.sleep(0.1)

    def turn_left(self):
        # Left turn sequence
        print("Making left turn")
        # retrieve data and set start position
        msg = self.get_odom()
        start_angle = 0.0
        if msg:
            start_angle = msg["orientation"]['z']
            if start_angle < 0:
                start_angle = start_angle+2
        else:
            print('waiting for odometry data')

        angle = 0.0
        current_angle = 0.0
        while angle < 0.5:
            drive_message = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.5}}  # Rotate left
            self.drive_pub.publish(roslibpy.Message(drive_message))

            msg = self.get_odom() #retrieve data and calculate relative turn angle
            current_angle = msg["orientation"]['z']
            if current_angle < 0:
                current_angle = current_angle+2
            angle = current_angle-start_angle
            if -2<angle<-1.5:
                angle = angle+2
            print(angle)
            time.sleep(0.1)

    def auto_mow(self): #autonomous mode
        last_turn = 'left'  # track the last turn direction
        while not self.stop_event.is_set():
            if self.joystick.autonomous_mode and self.joystick.armed:           
                self.drive_straight(1.5)
                # Decide on the turn direction (alternate turns)
                if last_turn == 'right':
                    self.turn_left()
                    self.drive_straight(0.35)
                    self.turn_left()
                    last_turn = 'left'  # Update last turn direction to 'left'
                elif last_turn == 'left':
                    self.turn_right()
                    self.drive_straight(0.35)
                    self.turn_right()
                    last_turn = 'right'  # Update last turn direction to 'right'

                # Sleep to allow the turn to complete before the next action
                time.sleep(0.1)

    def drive(self): #manual mode
        while not self.stop_event.is_set():
            if self.joystick.manual_mode == True:
                if self.joystick.armed == False:
                    drive_message = {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                    }
                    self.drive_pub.publish(roslibpy.Message(drive_message))
                elif self.joystick.armed == True:
                    drive_message = {
                    "linear": {"x": self.joystick.linear_x, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": self.joystick.angular_z}
                    }
                    self.drive_pub.publish(roslibpy.Message(drive_message))
            
            time.sleep(0.1)  # 10Hz

    def leds(self): #control lightring
        while not self.stop_event.is_set():
            play_lights(ros_node, robot_name, self.joystick.color)
            if self.joystick.armed:
                time.sleep(0.5)  # Blink on
                play_lights(ros_node, robot_name, 'Off')
                time.sleep(0.5)  # Blink off
            else:
                time.sleep(1)  # Keep the LED color

    def audio(self): #play audio
        last_mode = None  # Track the last executed mode

        while not self.stop_event.is_set():
            # Detect the current mode
            current_mode = None
            notes = []
            sleep_duration = 0

            if self.joystick.manual_mode:
                current_mode = "manual"
                notes = [
                    {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}},
                    {'frequency': 750, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}}
                ]
                sleep_duration = 1
            
            elif self.joystick.idle_mode:
                current_mode = "idle"
                notes = [
                    {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}},
                    {'frequency': 450, 'max_runtime': {'sec': 0, 'nanosec': int(5e8)}}
                ]
                sleep_duration = 1

            elif self.joystick.autonomous_mode:
                current_mode = "autonomous"
                notes = [
                    {'frequency': 600, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}},
                    {'frequency': 750, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}},
                    {'frequency': 900, 'max_runtime': {'sec': 0, 'nanosec': int(3e8)}}
                ]
                sleep_duration = 0.9
            
            elif self.joystick.armed == True:
                current_mode = 'armed'
                notes = [{'frequency': 300, 'max_runtime': {'sec': 3, 'nanosec': 0}}]
                sleep_duration = 3

            # Play notes only if the mode has changed
            if current_mode and current_mode != last_mode:
                audio_message = {'notes': notes, 'append': False}
                self.audio_pub.publish(roslibpy.Message(audio_message))
                time.sleep(sleep_duration)
                last_mode = current_mode  # Update last mode to prevent re-triggering

            time.sleep(0.1)

    def stop(self):
        self.stop_event.set()
        self.drive_thread.join()
        self.led_thread.join()
        self.audio_thread.join()
        self.auto_mow_thread.join()
        #self.sense_ir_thread.join()
        self.cleanup()

    def cleanup(self):
        play_lights(ros_node, robot_name, 'Off')
        self.drive_pub.publish(roslibpy.Message({"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}))
        self.led_pub.unadvertise()
        self.drive_pub.unadvertise()
        self.audio_pub.unadvertise()
        self.odom_topic.unsubscribe()
        self.ir_topic.unsubscribe()


# Main loop
if __name__ == "__main__":
    try:
        joystick = Joystick()
        robot = RobotController(joystick)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt  # Graceful exit

            time.sleep(0.1)  # 10Hz loop

    except KeyboardInterrupt:
        print("\nShutting down...")
        robot.stop()
        joystick.stop()
        pygame.quit()
        ros_node.terminate()
        print("Shutdown complete.")