import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, messagebox
import math
import threading
import time
import json

class RobotControlGUI(Node):
    def __init__(self):
        super().__init__('stepper_motor_control_gui')
        self.publisher_ = self.create_publisher(String, 'robot_command', 10)
        self.last_publish_time = 0
        
        # Animation control
        self.animation_running = False
        self.walking = False
        
        # Recording control
        self.recording = False
        self.recorded_frames = []
        self.recording_start_time = None
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Stepper Motor Leg Control")
        
        # Canvas for robot visualization
        self.canvas = tk.Canvas(self.root, width=1000, height=400, bg='white')
        self.canvas.pack(pady=10)
        
        # Joint angles (degrees) and segment vectors
        self.angles = {
            'leg1_upper': 93, 
            'leg1_lower': 88,   
            'leg2_upper': 86,
            'leg2_lower': 90,
            'leg3_upper': 68, 
            'leg3_lower': 87,   
            'leg4_upper': 92,
            'leg4_lower': 86 
        }
        
        # Store vectors for segments
        self.segment_vectors = {
            'leg1_upper': (0, 0),
            'leg1_lower': (0, 0),
            'leg2_upper': (0, 0),
            'leg2_lower': (0, 0)
        }
        
        # Store initial positions
        self.initial_angles = self.angles.copy()
        
        # Control panel
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(pady=10)
        
        # Sliders for each joint
        self.sliders = {}
        for i, (joint, angle) in enumerate(self.angles.items()):
            frame = ttk.Frame(self.control_frame)
            frame.grid(row=i//2, column=i%2, padx=10, pady=5)
            
            label = ttk.Label(frame, text=f"{joint}")
            label.pack()
            
            slider = ttk.Scale(frame, from_=0, to=180, orient='horizontal',
                             command=lambda v, j=joint: self.update_angle(j, float(v)))
            slider.set(angle)
            slider.pack()
            self.sliders[joint] = slider
        
        # Motion control buttons
        self.button_frame = ttk.Frame(self.root)
        self.button_frame.pack(pady=10)
        
        ttk.Button(self.button_frame, text="Grip", command=self.start_grip).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.button_frame, text="Reset Position", command=self.reset_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.button_frame, text="Start Walking", command=self.start_walking).pack(side=tk.LEFT, padx=5)
        ttk.Button(self.button_frame, text="Stop Walking", command=self.stop_walking).pack(side=tk.LEFT, padx=5)
        
        # Recording buttons frame
        self.record_frame = ttk.Frame(self.root)
        self.record_frame.pack(pady=5)
        
        self.record_button = ttk.Button(self.record_frame, text="Start Recording", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=5)
        
        self.playback_button = ttk.Button(self.record_frame, text="Play Recording", command=self.play_recording)
        self.playback_button.pack(side=tk.LEFT, padx=5)
        
        # Recording indicator
        self.record_indicator = ttk.Label(self.record_frame, text="●", foreground='gray')
        self.record_indicator.pack(side=tk.LEFT, padx=5)
        
        # Drawing parameters
        self.leg_length = 80
        self.base_y = 300
        self.leg1_x = 150
        self.leg2_x = 350
        self.leg3_x = 650
        self.leg4_x = 800
        
        # Setup canvas dragging
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<Button-1>', self.on_click)
        self.selected_joint = None
        
        # Debug mode for showing angle calculations
        self.debug_mode = True
        
        # Initial drawing
        self.draw_robot()

    # [Previous vector calculation methods remain the same]
    def calculate_vector(self, x1, y1, x2, y2):
        """Calculate vector between two points"""
        return (x2 - x1, y2 - y1)

    def calculate_vector_angle(self, vector):
        """Calculate angle of vector from vertical (clockwise)"""
        x, y = vector
        angle = math.degrees(math.atan2(x, -y))
        return (angle + 360) % 360

    def calculate_relative_angle(self, child_vector, parent_vector):
        """Calculate angle of child vector relative to parent's perpendicular"""
        parent_angle = self.calculate_vector_angle(parent_vector)
        child_angle = self.calculate_vector_angle(child_vector)
        relative_angle = (child_angle - (parent_angle + 90)) % 360
        if relative_angle > 180:
            relative_angle -= 360
        return relative_angle + 90

    def calculate_joint_pos(self, base_x, base_y, angle, length=80):
        """Calculate joint position using angle from vertical"""
        rad = math.radians(angle)
        x = base_x + length * math.cos(rad)
        y = base_y + length * math.sin(rad)
        return x, y


    def draw_robot(self):
        self.canvas.delete('all')
        
        # Draw ground line
        self.canvas.create_line(0, self.base_y, 500, self.base_y, fill='gray')
        
        # Calculate and draw leg 1
        l1_knee_x, l1_knee_y = self.calculate_joint_pos(
            self.leg1_x, self.base_y - 100, self.angles['leg1_upper'])
        
        self.segment_vectors['leg1_upper'] = self.calculate_vector(
            self.leg1_x, self.base_y - 100, l1_knee_x, l1_knee_y)
        
        # Calculate lower leg position based on upper leg's perpendicular
        l1_foot_x, l1_foot_y = self.calculate_joint_pos(
            l1_knee_x, l1_knee_y, 
            self.angles['leg1_upper'] + self.angles['leg1_lower'] - 90)
        
        self.segment_vectors['leg1_lower'] = self.calculate_vector(
            l1_knee_x, l1_knee_y, l1_foot_x, l1_foot_y)
        
        # Calculate and draw leg 2 (similar to leg 1)
        l2_knee_x, l2_knee_y = self.calculate_joint_pos(
            self.leg2_x, self.base_y - 100, self.angles['leg2_upper'])
        
        self.segment_vectors['leg2_upper'] = self.calculate_vector(
            self.leg2_x, self.base_y - 100, l2_knee_x, l2_knee_y)
        
        l2_foot_x, l2_foot_y = self.calculate_joint_pos(
            l2_knee_x, l2_knee_y,
            self.angles['leg2_upper'] + self.angles['leg2_lower'] - 90)
        
        self.segment_vectors['leg2_lower'] = self.calculate_vector(
            l2_knee_x, l2_knee_y, l2_foot_x, l2_foot_y)


        # Calculate and draw leg 3
        l3_knee_x, l3_knee_y = self.calculate_joint_pos(
            self.leg3_x, self.base_y - 100, self.angles['leg3_upper'])
        
        self.segment_vectors['leg3_upper'] = self.calculate_vector(
            self.leg3_x, self.base_y - 100, l3_knee_x, l3_knee_y)
        
        # Calculate lower leg position based on upper leg's perpendicular
        l3_foot_x, l3_foot_y = self.calculate_joint_pos(
            l3_knee_x, l3_knee_y, 
            self.angles['leg3_upper'] + self.angles['leg3_lower'] - 90)
        
        self.segment_vectors['leg3_lower'] = self.calculate_vector(
            l3_knee_x, l3_knee_y, l3_foot_x, l3_foot_y)
        
        # Calculate and draw leg 3
        l4_knee_x, l4_knee_y = self.calculate_joint_pos(
            self.leg4_x, self.base_y - 100, self.angles['leg4_upper'])
        
        self.segment_vectors['leg4_upper'] = self.calculate_vector(
            self.leg4_x, self.base_y - 100, l4_knee_x, l4_knee_y)
        
        # Calculate lower leg position based on upper leg's perpendicular
        l4_foot_x, l4_foot_y = self.calculate_joint_pos(
            l4_knee_x, l4_knee_y, 
            self.angles['leg4_upper'] + self.angles['leg4_lower'] - 90)
        
        self.segment_vectors['leg4_lower'] = self.calculate_vector(
            l4_knee_x, l4_knee_y, l4_foot_x, l4_foot_y)
        # Draw legs
        self.canvas.create_line(self.leg1_x, self.base_y - 100, l1_knee_x, l1_knee_y, 
                            width=3, fill='black')
        self.canvas.create_line(l1_knee_x, l1_knee_y, l1_foot_x, l1_foot_y, 
                            width=3, fill='black')
        self.canvas.create_line(self.leg2_x, self.base_y - 100, l2_knee_x, l2_knee_y, 
                            width=3, fill='black')
        self.canvas.create_line(l2_knee_x, l2_knee_y, l2_foot_x, l2_foot_y, 
                            width=3, fill='black')
        self.canvas.create_line(self.leg3_x, self.base_y - 100, l3_knee_x, l3_knee_y, 
                            width=3, fill='black')
        self.canvas.create_line(l3_knee_x, l3_knee_y, l3_foot_x, l3_foot_y, 
                            width=3, fill='black')
        self.canvas.create_line(self.leg4_x, self.base_y - 100, l4_knee_x, l4_knee_y, 
                            width=3, fill='black')
        self.canvas.create_line(l4_knee_x, l4_knee_y, l4_foot_x, l4_foot_y, 
                            width=3, fill='black')
        
        # Draw joints and their angles
        # Leg 1 joints
        self.canvas.create_oval(self.leg1_x-5, self.base_y-105, self.leg1_x+5, self.base_y-95, fill='red')
        self.canvas.create_text(self.leg1_x-20, self.base_y-100, 
                            text=f"{self.angles['leg1_upper']:.1f}°", anchor='e')
        
        self.canvas.create_oval(l1_knee_x-5, l1_knee_y-5, l1_knee_x+5, l1_knee_y+5, fill='red')
        self.canvas.create_text(l1_knee_x-20, l1_knee_y, 
                            text=f"{self.angles['leg1_lower']:.1f}°", anchor='e')
        
        self.canvas.create_oval(l1_foot_x-5, l1_foot_y-5, l1_foot_x+5, l1_foot_y+5, fill='red')
        
        # Leg 2 joints
        self.canvas.create_oval(self.leg2_x-5, self.base_y-105, self.leg2_x+5, self.base_y-95, fill='red')
        self.canvas.create_text(self.leg2_x+20, self.base_y-100, 
                            text=f"{self.angles['leg2_upper']:.1f}°", anchor='w')
        
        self.canvas.create_oval(l2_knee_x-5, l2_knee_y-5, l2_knee_x+5, l2_knee_y+5, fill='red')
        self.canvas.create_text(l2_knee_x+20, l2_knee_y, 
                            text=f"{self.angles['leg2_lower']:.1f}°", anchor='w')
        
        self.canvas.create_oval(l2_foot_x-5, l2_foot_y-5, l2_foot_x+5, l2_foot_y+5, fill='red')
        
        # Leg 3 joints
        self.canvas.create_oval(self.leg3_x-5, self.base_y-105, self.leg3_x+5, self.base_y-95, fill='red')
        self.canvas.create_text(self.leg3_x+20, self.base_y-100, 
                            text=f"{self.angles['leg3_upper']:.1f}°", anchor='w')
        
        self.canvas.create_oval(l3_knee_x-5, l3_knee_y-5, l3_knee_x+5, l3_knee_y+5, fill='red')
        self.canvas.create_text(l3_knee_x+20, l3_knee_y, 
                            text=f"{self.angles['leg3_lower']:.1f}°", anchor='w')
        self.canvas.create_oval(l3_foot_x-5, l3_foot_y-5, l3_foot_x+5, l3_foot_y+5, fill='red')
        
        # Leg 4 joints
        self.canvas.create_oval(self.leg4_x-5, self.base_y-105, self.leg4_x+5, self.base_y-95, fill='red')
        self.canvas.create_text(self.leg4_x+20, self.base_y-100, 
                            text=f"{self.angles['leg4_upper']:.1f}°", anchor='w')
        
        self.canvas.create_oval(l4_knee_x-5, l4_knee_y-5, l4_knee_x+5, l4_knee_y+5, fill='red')
        self.canvas.create_text(l4_knee_x+20, l4_knee_y, 
                            text=f"{self.angles['leg4_lower']:.1f}°", anchor='w')
        self.canvas.create_oval(l4_foot_x-5, l4_foot_y-5, l4_foot_x+5, l4_foot_y+5, fill='red')


        if self.debug_mode:
            # Draw perpendicular reference lines and angles
            self.draw_debug_info(l1_knee_x, l1_knee_y, self.angles['leg1_upper'], self.angles['leg1_lower'], 'leg1')
            self.draw_debug_info(l2_knee_x, l2_knee_y, self.angles['leg2_upper'], self.angles['leg2_lower'], 'leg2')
            self.draw_debug_info(l3_knee_x, l3_knee_y, self.angles['leg3_upper'], self.angles['leg3_lower'], 'leg3')
            self.draw_debug_info(l4_knee_x, l4_knee_y, self.angles['leg4_upper'], self.angles['leg4_lower'], 'leg4')
        
        # Store joint positions for drag detection
        self.joint_positions = {
            'leg1_upper': (l1_knee_x, l1_knee_y),
            'leg1_lower': (l1_foot_x, l1_foot_y),
            'leg2_upper': (l2_knee_x, l2_knee_y),
            'leg2_lower': (l2_foot_x, l2_foot_y),
            'leg3_upper': (l3_knee_x, l3_knee_y),
            'leg3_lower': (l3_foot_x, l3_foot_y),
            'leg4_upper': (l4_knee_x, l4_knee_y),
            'leg4_lower': (l4_foot_x, l4_foot_y)
        }

    # [Previous drawing methods remain the same]
    def draw_debug_info(self, x, y, upper_angle, lower_angle, leg_prefix):
        """Draw debug information for a leg"""
        # Draw vertical reference
        self.canvas.create_line(x, y-30, x, y+30, fill='blue', dash=(2, 2))
        
        # Draw perpendicular to upper segment
        rad = math.radians(upper_angle - 90)
        px = x + 30 * math.cos(rad)
        py = y + 30 * math.sin(rad)
        self.canvas.create_line(x, y, px, py, fill='green', dash=(2, 2))
        
        # Show angles
        #self.canvas.create_text(x + 20, y,
        #                      text=f"Upper: {upper_angle:.1f}°\nLower: {lower_angle:.1f}°")

    def on_click(self, event):
        x, y = event.x, event.y
        self.selected_joint = None
        for joint, (joint_x, joint_y) in self.joint_positions.items():
            if math.sqrt((x - joint_x)**2 + (y - joint_y)**2) < 10:
                self.selected_joint = joint
                return

    def on_drag(self, event):
        if not self.selected_joint:
            return
            
        x, y = event.x, event.y
        
        if self.selected_joint.endswith('upper'):
            # Base joint calculations
            base_x = self.leg1_x if 'leg1' in self.selected_joint else self.leg2_x
            base_y = self.base_y - 100
            
            # Calculate new upper leg vector and angle
            new_vector = self.calculate_vector(base_x, base_y, x, y)
            new_angle = self.calculate_vector_angle(new_vector)
            
            # Update upper angle
            self.angles[self.selected_joint] = new_angle
            
            # Recalculate lower angle relative to new upper segment
            lower_joint = self.selected_joint.replace('upper', 'lower')
            current_lower_vector = self.segment_vectors[lower_joint]
            new_lower_angle = self.calculate_relative_angle(current_lower_vector, new_vector)
            self.angles[lower_joint] = new_lower_angle
            
        else:
            # Knee joint calculations
            upper_joint = self.selected_joint.replace('lower', 'upper')
            knee_x, knee_y = self.joint_positions[upper_joint]
            
            # Calculate new lower leg vector and angle relative to upper segment
            new_vector = self.calculate_vector(knee_x, knee_y, x, y)
            new_angle = self.calculate_relative_angle(new_vector, self.segment_vectors[upper_joint])
            self.angles[self.selected_joint] = new_angle
        
        # Update sliders and redraw
        for joint, angle in self.angles.items():
            # Ensure angle is in 0-180 range for sliders
            slider_angle = angle % 180
            self.sliders[joint].set(slider_angle)
        
        self.draw_robot()
        self.publish_angles()

    def animate_to_angles(self, target_angles, duration=1.0, completion_callback=None):
        """Animate joints to target angles"""
        start_angles = self.angles.copy()
        start_time = time.time()
        
        self.animation_running = True
        def update():
            current_time = time.time()
            progress = min((current_time - start_time) / duration, 1.0)
            
            if progress < 1.0 and self.animation_running:
                # Interpolate angles
                for joint in self.angles:
                    #all the conditions are ment to handel if I want to turn off a motor during animation
                    if start_angles[joint] == 999 and target_angles[joint] !=999:
                        current_angle = target_angles[joint]
                    else:
                        current_angle = start_angles[joint] + (target_angles[joint] - start_angles[joint]) * progress

                    if target_angles[joint] != 999:
                        self.angles[joint] = current_angle
                        self.sliders[joint].set(current_angle)
                        self.update_angle(joint, current_angle)
                    else:
                        self.update_angle(joint, 999)

                
                self.draw_robot()
                self.root.after(16, update)  # ~60 FPS
            elif completion_callback and progress >= 1.0:
                completion_callback()

        update()

    def start_grip(self):
        """Start grip animation"""
        self.animation_running = False
        time.sleep(0.1)
        
        grip_angles = {
            'leg1_upper': 32,
            'leg1_lower': 135,
            'leg2_upper': 135,
            'leg2_lower': 48,
            'leg3_upper': 68, 
            'leg3_lower': 87,   
            'leg4_upper': 92,
            'leg4_lower': 86 
        }
        
        self.animate_to_angles(grip_angles)

    def start_walking(self):
        """Start walking animation"""
        self.walking = True
        self.walk_cycle()

    def stop_walking(self):
        """Stop walking animation"""
        self.walking = False
        self.reset_position()

    def walk_cycle(self):
        """Perform walking cycle animation"""
        if not self.walking:
            return
            
        keyframes = [
        # 1.up right 
        {
            'leg1_upper': 93, 
            'leg1_lower': 88,   
            'leg2_upper': 86,
            'leg2_lower': 90,
            'leg3_upper': 68, 
            'leg3_lower': 87,   
            'leg4_upper': 92,
            'leg4_lower': 86 
        },
        #2. in possition to sway
        {
            'leg1_upper': 93, 
            'leg1_lower': 88,   
            'leg2_upper': 86,
            'leg2_lower': 90,
            'leg3_upper': 139, 
            'leg3_lower': 11,   
            'leg4_upper': 11,
            'leg4_lower': 157 
        },
        # 3. sway to the side
        {
            'leg1_upper': 93, #the weight of the robot is increacing this angle :(   
            'leg1_lower': 111,    
            'leg2_upper': 97,   
            'leg2_lower': 82,    
            'leg3_upper': 139, 
            'leg3_lower': 11,   
            'leg4_upper': 11,
            'leg4_lower': 157 
        },
        {
            'leg1_upper': 93, 
            'leg1_lower': 175,    
            'leg2_upper': 97,   
            'leg2_lower': 140,    
            'leg3_upper': 68, 
            'leg3_lower': 87,   
            'leg4_upper': 92,
            'leg4_lower': 86 
        },
        {
            'leg1_upper': 42, 
            'leg1_lower': 175,    
            'leg2_upper': 48,   
            'leg2_lower': 140,    
            'leg3_upper': 68, 
            'leg3_lower': 87,   
            'leg4_upper': 92,
            'leg4_lower': 86 
        },
        ]


        def animate_next_keyframe(index=0):
            if not self.walking:
                return
                
            target_angles = keyframes[index]
            next_index = (index + 1) % len(keyframes)
            
            self.animate_to_angles(
                target_angles,
                duration=0.8,
                completion_callback=lambda: animate_next_keyframe(next_index)
            )
        
        animate_next_keyframe()

    # [Recording methods]
    def toggle_recording(self):
        """Toggle recording state"""
        if not self.recording:
            self.recording = True
            self.recorded_frames = []
            self.recording_start_time = time.time()
            self.record_button.configure(text="Stop Recording")
            self.record_indicator.configure(foreground='red')
            self.record_frame_callback()
        else:
            self.recording = False
            self.record_button.configure(text="Start Recording")
            self.record_indicator.configure(foreground='gray')
            if self.recorded_frames:
                messagebox.showinfo("Recording Complete", 
                                  f"Recorded {len(self.recorded_frames)} frames of motion")

    def record_frame_callback(self):
        """Record current frame"""
        if self.recording:
            frame = {
                'timestamp': time.time() - self.recording_start_time,
                'angles': self.angles.copy()
            }
            self.recorded_frames.append(frame)
            self.root.after(33, self.record_frame_callback)  # 30 FPS

    def play_recording(self):
        """Play recorded animation"""
        if not self.recorded_frames:
            messagebox.showwarning("Playback Error", "No recorded motion available")
            return
            
        self.animation_running = False
        self.walking = False
        time.sleep(0.1)
        self.play_recorded_frame(0)

    def play_recorded_frame(self, frame_index):
        """Play a single frame of recorded animation"""
        if frame_index >= len(self.recorded_frames):
            return
            
        frame = self.recorded_frames[frame_index]
        
        for joint, angle in frame['angles'].items():
            self.angles[joint] = angle
            self.sliders[joint].set(angle)
            self.publish_angle()
        
        self.draw_robot()
        
        if frame_index + 1 < len(self.recorded_frames):
            current_timestamp = frame['timestamp']
            next_timestamp = self.recorded_frames[frame_index + 1]['timestamp']
            delay = int((next_timestamp - current_timestamp) * 1000)
            self.root.after(delay, lambda: self.play_recorded_frame(frame_index + 1))

    def publish_angle(self):
        """Convert GUI angles to robot servo angles and publish"""
        servo_map = {
            'leg1_upper': 'A',
            'leg1_lower': 'B',
            'leg3_upper': 'C',
            'leg3_lower': 'D',
            'leg2_upper': 'E',
            'leg2_lower': 'F',
            'leg4_upper': 'H',
            'leg4_lower': 'G'
        }
        
        command = ""
        angles = []
        joints = []
        for joint, angle in self.angles.items():
            if servo_map[joint] in ['A', 'F', 'C', 'H', 'G']:
                angles.append(180 - angle)
            else:
                angles.append(angle)
            joints.append(servo_map[joint])

            #self.publish_angle(joint, angle)
        # Normalize angle to 0-180 range
        #servo_angle = angle % 180
        

        current_time = time.time()
        if current_time - self.last_publish_time >= 0.1:  # 0.1 seconds throttle
            msg = String()
            msg.data = ""
            for i in range(8):
                msg.data += f"{joints[i]} {int(angles[i])};"
            self.publisher_.publish(msg)
            self.last_publish_time = current_time
            self.get_logger().info(f"Publishing: {msg.data}")


    def update_angle(self, joint, angle):
        """Update angle from slider"""
        self.angles[joint] = angle
        self.publish_angle()
        self.draw_robot()

    def reset_position(self):
        #self.animation_running = False
        time.sleep(0.1)
        #self.animate_to_angles(self.initial_angles)
        """Reset all joints to initial positions"""
        for joint, angle in self.initial_angles.items():
            #self.angles[joint] = angle
            self.sliders[joint].set(angle)
            print(joint, angle)

def main(args=None):
    rclpy.init(args=args)
    gui = RobotControlGUI()
    
    # Run ROS spin in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(gui), daemon=True)
    ros_thread.start()
    
    # Run GUI main loop
    gui.root.mainloop()
    
    # Cleanup
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()