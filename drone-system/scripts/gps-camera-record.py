import time
import csv
import threading
from datetime import datetime
from picamera2 import Picamera2
from pymavlink import mavutil

class DroneRecorder:
    def __init__(self, mavlink_port='/dev/ttyS0', mavlink_baud=57600, output_folder='drone_recordings'):
        self.drone = mavutil.mavlink_connection(mavlink_port, baud=mavlink_baud)
        
        self.picam2 = Picamera2()
        self.camera_config = self.picam2.create_preview_configuration()
        self.picam2.configure(self.camera_config)
        
        self.recording = False
        self.gps_thread = None
        self.output_folder = output_folder
        self.start_time = None
        self.video_filename = None
        self.csv_filename = None
        
        print("Waiting for drone heartbeat...")
        self.drone.wait_heartbeat()
        print("Drone connected!")
        
    def log_gps_data(self):
        with open(self.csv_filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['timestamp', 'seconds_since_start', 'latitude', 'longitude', 'altitude'])
            
            while self.recording:
                msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    current_time = datetime.now()
                    timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                    seconds_since_start = (current_time - self.start_time).total_seconds()
                    
                    lat = msg.lat * 1.0e-7
                    lon = msg.lon * 1.0e-7
                    alt = msg.alt / 1000.0
                    
                    csv_writer.writerow([timestamp, seconds_since_start, lat, lon, alt])
                    csvfile.flush()
                    
                    print(f"[{seconds_since_start:.2f}s] Lat: {lat}, Lon: {lon}, Alt: {alt}m")
                
                time.sleep(0.01)
    
    def start_recording(self, duration=60, video_prefix="drone_video"):
        if self.recording:
            print("Already recording!")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_filename = f"{video_prefix}_{timestamp}.mp4"
        self.csv_filename = f"{video_prefix}_{timestamp}_gps.csv"
        
        print(f"Starting recording to {self.video_filename}")
        print(f"Logging GPS data to {self.csv_filename}")
        
        self.picam2.start()
        time.sleep(1)
        
        self.recording = True
        self.start_time = datetime.now()
        
        self.gps_thread = threading.Thread(target=self.log_gps_data)
        self.gps_thread.start()
        
        self.picam2.start_and_record_video(self.video_filename, duration=duration)
        
        time.sleep(duration)
        self.stop_recording()
    
    def stop_recording(self):
        if not self.recording:
            return
        
        self.recording = False
        
        self.picam2.stop_recording()
        self.picam2.stop()
        
        if self.gps_thread and self.gps_thread.is_alive():
            self.gps_thread.join()
        
        print(f"Recording stopped. Video saved to {self.video_filename}")
        print(f"GPS data saved to {self.csv_filename}")
    
    def close(self):
        if self.recording:
            self.stop_recording()
        if hasattr(self, 'picam2'):
            self.picam2.close()

if __name__ == "__main__":
    recorder = DroneRecorder()
    
    try:
        print("Starting 5-minute recording...")
        recorder.start_recording(duration=360)
        
    except KeyboardInterrupt:
        print("Recording interrupted")
        recorder.stop_recording()
    finally:
        recorder.close()
        print("Recording session ended")
