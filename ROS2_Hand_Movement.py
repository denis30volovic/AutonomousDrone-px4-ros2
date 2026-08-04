import torch
import warnings

# Patch torch.load for ultralytics compatibility
original_load = torch.load
def patched_load(*args, **kwargs):
    kwargs['weights_only'] = False
    return original_load(*args, **kwargs)
torch.load = patched_load

warnings.filterwarnings("ignore")

import cv2
import mediapipe as mp
import numpy as np
import time
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from PIL import Image, ImageTk
from ultralytics import YOLO

class DroneControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control - Hand Gesture + Manual Control")
        self.root.geometry("1600x900")
        
        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8)
        
        # YOLO setup
        print("Loading YOLOv8 model...")
        import os
        model_path = os.path.join(os.path.dirname(__file__), 'yolov8n.pt')
        self.yolo_model = YOLO(model_path)
        print("YOLO model loaded successfully")
        
        # Camera setup
        self.cap = cv2.VideoCapture("http://192.168.0.113:8080/video")
        
        # Gesture recognition variables
        self.gesture_label = ""
        self.gesture_label_time = 0
        self.DISPLAY_DURATION = 1.0
        self.HOLD_DURATION = 1.0
        self.COOLDOWN = 3.3
        
        # Cooldown timers
        self.last_forward_time = 0
        self.last_backward_time = 0
        self.last_orbit_time = 0
        self.last_forward_return_time = 0
        
        # Gesture start times
        self.forward_start_time = None
        self.backward_start_time = None
        self.orbit_start_time = None
        self.forward_return_start_time = None
        
        # GUI setup
        self.setup_gui()
        
        # Start video processing
        self.is_running = True
        self.update_video()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Video frames container
        video_container = ttk.Frame(main_frame)
        video_container.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # Hand Detection Video frame
        hand_frame = ttk.LabelFrame(video_container, text="Hand Detection & Gesture Control", padding="5")
        hand_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        self.hand_video_label = tk.Label(hand_frame)
        self.hand_video_label.pack()
        
        # Object Detection Video frame
        object_frame = ttk.LabelFrame(video_container, text="Object Detection", padding="5")
        object_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        
        self.object_video_label = tk.Label(object_frame)
        self.object_video_label.pack()
        
        # Control panels
        self.setup_manual_controls(main_frame)
        self.setup_gesture_info(main_frame)
        
    def setup_manual_controls(self, parent):
        # Manual control frame
        control_frame = ttk.LabelFrame(parent, text="Manual Control", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        # Forward/Backward controls
        movement_frame = ttk.LabelFrame(control_frame, text="Movement", padding="5")
        movement_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(movement_frame, text="Distance (m):").grid(row=0, column=0, padx=5)
        self.distance_var = tk.StringVar(value="5.0")
        distance_entry = ttk.Entry(movement_frame, textvariable=self.distance_var, width=10)
        distance_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(movement_frame, text="Forward", command=self.send_forward).grid(row=0, column=2, padx=5)
        ttk.Button(movement_frame, text="Backward", command=self.send_backward).grid(row=0, column=3, padx=5)
        
        # Orbit controls
        orbit_frame = ttk.LabelFrame(control_frame, text="Orbit Mission", padding="5")
        orbit_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(orbit_frame, text="Radius (m):").grid(row=0, column=0, padx=5)
        self.orbit_var = tk.StringVar(value="2.0")
        orbit_entry = ttk.Entry(orbit_frame, textvariable=self.orbit_var, width=10)
        orbit_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(orbit_frame, text="Start Orbit", command=self.send_orbit).grid(row=0, column=2, padx=5)
        
        # Forward Return controls
        return_frame = ttk.LabelFrame(control_frame, text="Forward Return Mission", padding="5")
        return_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(return_frame, text="Distance (m):").grid(row=0, column=0, padx=5)
        self.return_var = tk.StringVar(value="8.0")
        return_entry = ttk.Entry(return_frame, textvariable=self.return_var, width=10)
        return_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(return_frame, text="Start Mission", command=self.send_forward_return).grid(row=0, column=2, padx=5)
        
        # Status
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.status_var, font=("Arial", 12, "bold"))
        status_label.grid(row=3, column=0, columnspan=2, pady=10)
        
    def setup_gesture_info(self, parent):
        # Gesture info frame
        info_frame = ttk.LabelFrame(parent, text="Gesture Control", padding="10")
        info_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        
        gestures = [
            "🖐️ Open Hand → Forward (4m)",
            "☝️ Index Up → Backward (4m)",
            "✌️ Peace Sign → Orbit Mission",
            "🤟 Three Fingers → Forward Return (5m)",
            "",
            "Hold gesture for 1 second",
            "3.3 second cooldown between commands"
        ]
        
        for i, gesture in enumerate(gestures):
            label = ttk.Label(info_frame, text=gesture, font=("Arial", 10))
            label.grid(row=i, column=0, sticky=tk.W, pady=2)
            
        # Current gesture display
        self.gesture_var = tk.StringVar(value="No gesture detected")
        gesture_label = ttk.Label(info_frame, textvariable=self.gesture_var, 
                                 font=("Arial", 12, "bold"), foreground="green")
        gesture_label.grid(row=len(gestures), column=0, pady=10)
        
    def send_forward(self):
        try:
            distance = float(self.distance_var.get())
            self.send_command(f'ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: {distance}"')
            self.status_var.set(f"Forward {distance}m command sent")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for distance")
            
    def send_backward(self):
        try:
            distance = float(self.distance_var.get())
            self.send_command(f'ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: {-distance}"')
            self.status_var.set(f"Backward {distance}m command sent")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for distance")
            
    def send_orbit(self):
        try:
            radius = float(self.orbit_var.get())
            self.send_command(f'ros2 topic pub -1 /drone_command/orbit std_msgs/msg/Float32 "data: {radius}"')
            self.status_var.set(f"Orbit mission (radius {radius}m) started")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for radius")
            
    def send_forward_return(self):
        try:
            distance = float(self.return_var.get())
            self.send_command(f'ros2 topic pub -1 /drone_command/forward_return std_msgs/msg/Float32 "data: {distance}"')
            self.status_var.set(f"Forward return mission ({distance}m) started")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for distance")
            
    def send_command(self, command):
        try:
            subprocess.Popen(command, shell=True)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send command: {str(e)}")
    
    def update_video(self):
        if not self.is_running:
            return
            
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.flip(frame, 1)
            current_time = time.time()
            
            # Create copies for each processing stream
            hand_frame = frame.copy()
            object_frame = frame.copy()
            
            # Process hand detection stream
            rgb = cv2.cvtColor(hand_frame, cv2.COLOR_BGR2RGB)
            result = self.hands.process(rgb)
            
            # Process hand gestures
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    lm = hand_landmarks.landmark
                    self.mp_drawing.draw_landmarks(hand_frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    # Process gestures
                    self.process_gestures(lm, current_time)
            
            # Update gesture display on hand detection frame
            if current_time - self.gesture_label_time < self.DISPLAY_DURATION:
                cv2.putText(hand_frame, f"Gesture: {self.gesture_label}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
                self.gesture_var.set(f"Detected: {self.gesture_label}")
            else:
                self.gesture_var.set("No gesture detected")
            
            # Process object detection stream
            try:
                yolo_results = self.yolo_model(object_frame)
                object_frame = yolo_results[0].plot()
            except Exception as e:
                # If YOLO fails, just show the original frame with error text
                cv2.putText(object_frame, f"YOLO Error: {str(e)[:50]}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            
            # Convert hand detection frame to PhotoImage for tkinter
            hand_frame_rgb = cv2.cvtColor(hand_frame, cv2.COLOR_BGR2RGB)
            hand_frame_pil = Image.fromarray(hand_frame_rgb)
            hand_frame_pil = hand_frame_pil.resize((640, 480))
            hand_frame_tk = ImageTk.PhotoImage(hand_frame_pil)
            
            # Convert object detection frame to PhotoImage for tkinter
            object_frame_rgb = cv2.cvtColor(object_frame, cv2.COLOR_BGR2RGB)
            object_frame_pil = Image.fromarray(object_frame_rgb)
            object_frame_pil = object_frame_pil.resize((640, 480))
            object_frame_tk = ImageTk.PhotoImage(object_frame_pil)
            
            # Update both video displays
            self.hand_video_label.configure(image=hand_frame_tk)
            self.hand_video_label.image = hand_frame_tk
            
            self.object_video_label.configure(image=object_frame_tk)
            self.object_video_label.image = object_frame_tk
        
        # Schedule next update
        self.root.after(10, self.update_video)
    
    def process_gestures(self, lm, current_time):
        # MOVE FORWARD: Open hand
        if self.all_fingers_extended_strict(lm):
            if self.forward_start_time is None:
                self.forward_start_time = current_time
            elif (current_time - self.forward_start_time >= self.HOLD_DURATION and
                  current_time - self.last_forward_time >= self.COOLDOWN):
                self.gesture_label = "Move Forward"
                self.gesture_label_time = current_time
                self.last_forward_time = current_time
                self.forward_start_time = None
                self.send_command('ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: 4.0"')
        else:
            self.forward_start_time = None
        
        # MOVE BACKWARD: Index finger
        if self.only_index_extended_strict(lm):
            direction = self.get_index_direction(lm)
            if direction is not None and -direction[1] > 0.75:
                if self.backward_start_time is None:
                    self.backward_start_time = current_time
                elif (current_time - self.backward_start_time >= self.HOLD_DURATION and
                      current_time - self.last_backward_time >= self.COOLDOWN):
                    self.gesture_label = "Move Backward"
                    self.gesture_label_time = current_time
                    self.last_backward_time = current_time
                    self.backward_start_time = None
                    self.send_command('ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: -4.0"')
            else:
                self.backward_start_time = None
        else:
            self.backward_start_time = None
        
        # ORBIT MISSION: Peace sign
        if self.peace_sign_gesture(lm):
            if self.orbit_start_time is None:
                self.orbit_start_time = current_time
            elif (current_time - self.orbit_start_time >= self.HOLD_DURATION and
                  current_time - self.last_orbit_time >= self.COOLDOWN):
                self.gesture_label = "Orbit Mission"
                self.gesture_label_time = current_time
                self.last_orbit_time = current_time
                self.orbit_start_time = None
                self.send_command('ros2 topic pub -1 /drone_command/orbit std_msgs/msg/Float32 "data: 1.0"')
        else:
            self.orbit_start_time = None
        
        # FORWARD RETURN MISSION: Three fingers
        if self.three_fingers_gesture(lm):
            if self.forward_return_start_time is None:
                self.forward_return_start_time = current_time
            elif (current_time - self.forward_return_start_time >= self.HOLD_DURATION and
                  current_time - self.last_forward_return_time >= self.COOLDOWN):
                self.gesture_label = "Forward Return Mission"
                self.gesture_label_time = current_time
                self.last_forward_return_time = current_time
                self.forward_return_start_time = None
                self.send_command('ros2 topic pub -1 /drone_command/forward_return std_msgs/msg/Float32 "data: 5.0"')
        else:
            self.forward_return_start_time = None
    
    def is_finger_strictly_extended(self, lm, tip_id, pip_id):
        tip = lm[tip_id]
        pip = lm[pip_id]
        return tip.y < pip.y
    
    def all_fingers_extended_strict(self, lm):
        return (
            self.is_finger_strictly_extended(lm, 4, 3) and
            self.is_finger_strictly_extended(lm, 8, 6) and
            self.is_finger_strictly_extended(lm, 12, 10) and
            self.is_finger_strictly_extended(lm, 16, 14) and
            self.is_finger_strictly_extended(lm, 20, 18)
        )
    
    def only_index_extended_strict(self, lm):
        return (
            self.is_finger_strictly_extended(lm, 8, 6) and
            not self.is_finger_strictly_extended(lm, 12, 10) and
            not self.is_finger_strictly_extended(lm, 16, 14) and
            not self.is_finger_strictly_extended(lm, 20, 18)
        )
    
    def peace_sign_gesture(self, lm):
        return (
            self.is_finger_strictly_extended(lm, 8, 6) and
            self.is_finger_strictly_extended(lm, 12, 10) and
            not self.is_finger_strictly_extended(lm, 16, 14) and
            not self.is_finger_strictly_extended(lm, 20, 18)
        )
    
    def three_fingers_gesture(self, lm):
        return (
            self.is_finger_strictly_extended(lm, 8, 6) and
            self.is_finger_strictly_extended(lm, 12, 10) and
            self.is_finger_strictly_extended(lm, 16, 14) and
            not self.is_finger_strictly_extended(lm, 20, 18)
        )
    
    def get_index_direction(self, lm):
        tip = np.array([lm[8].x, lm[8].y, lm[8].z])
        mcp = np.array([lm[5].x, lm[5].y, lm[5].z])
        direction = tip - mcp
        norm = np.linalg.norm(direction)
        if norm == 0:
            return None
        return direction / norm
    
    def on_closing(self):
        self.is_running = False
        self.cap.release()
        self.hands.close()
        # Clear YOLO model from memory
        del self.yolo_model
        self.root.destroy()

def main():
    root = tk.Tk()
    app = DroneControlGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()

if __name__ == "__main__":
    main()