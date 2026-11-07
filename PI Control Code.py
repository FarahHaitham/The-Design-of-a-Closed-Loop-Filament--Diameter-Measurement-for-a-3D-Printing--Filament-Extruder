import tkinter as tk
from tkinter import ttk
import threading
import time
import math  
import socket
from collections import deque

# Hardware Libraries
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero import PWMOutputDevice

# =====================================================================
# 1. HARDWARE INITIALIZATION (ADS1115 & L298N Motor Driver)
# =====================================================================
print("Initializing I2C connection to ADS1115...")
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS1115(i2c)
    chan = AnalogIn(ads, 0)
    print("Successfully connected to the ADC!")
    ads_connected = True
except Exception as e:
    print(f"ADS1115 Connection failed. Running in simulation mode. Error: {e}")
    ads_connected = False
    chan = None

print("Initializing L298N Motor Driver (Direct PWM on IN1)...")
try:
    motor_in1 = PWMOutputDevice(23)
    print("Motor driver pin configured successfully!")
except Exception as e:
    print(f"Failed to initialize GPIO pin for motor driver: {e}")
    motor_in1 = None

# =====================================================================
# 2. GLOBAL VARIABLES & UDP SETUP
# =====================================================================
target_diameter = 1.75
current_rpm = 22.0              
live_sensor_diameter = 1.70  
live_sensor_voltage = 0.0  
integral_memory = 0.0  
calibration_offset = 0.17          

# --- MATLAB TELEMETRY SETUP ---
MATLAB_IP = "172.20.10.2"  # <--- CHANGE THIS TO YOUR COMPUTER's IP
MATLAB_PORT = 5005
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =====================================================================
# 3. SENSOR READING FUNCTION
# =====================================================================
def get_physical_diameter_mm():
    """Reads the ADS1115 analog pin, updates global voltage, and outputs mm."""
    global live_sensor_voltage, target_diameter, calibration_offset
    
    if not ads_connected or chan is None:
        return target_diameter
        
    try:
        live_sensor_voltage = chan.voltage
        
        a = 0.0099
        b = 0.0348
        c = 1.8921 - live_sensor_voltage
        
        discriminant = (b**2) - (4 * a * c)
        
        if discriminant >= 0:
            measured_mm = (-b + math.sqrt(discriminant)) / (2 * a)
            
            # Apply offset FIRST so empty readings don't bug out
            measured_mm += calibration_offset
            
            if measured_mm < 0:
                measured_mm = 0.0
        else:
            measured_mm = 0.0
            
        return measured_mm
                
    except OSError as e:
        print(f"Read error from ADS1115: {e}. Retrying...")
        return live_sensor_diameter

# =====================================================================
# 4. CLOSED-LOOP BACKGROUND CONTROL THREAD (PID with Dead-Time Compensation)
# =====================================================================
def closed_loop_worker():
    global target_diameter, current_rpm, live_sensor_diameter, integral_memory
    
    # --- TRANSPORT DELAY CONFIGURATION ---
    # Distance from nozzle to sensor in millimeters (1 meter = 1000mm)
    DISTANCE_TO_SENSOR_MM = 1000.0
    
    # Calibration factor: Establishes line speed (mm/s) relative to RPM.
    # Adjust this so: Line_Speed (mm/s) = RPM * RPM_TO_MM_PER_SEC
    RPM_TO_MM_PER_SEC = 0.8  
    
    # --- PID GAINS (Tuned for long physical lag) ---
    Kp = 12.0                 # Moderate proportional push
    Ki = 0.4                  # Slow, steady correction of persistent errors
    Kd = 3.0                  # Damping to stop overshoot
    
    # --- SENSOR LAG/RESOLUTION DEADBAND ---
    SENSOR_RESOLUTION = 0.1   # 0.1 mm resolution limit
    filter_alpha = 0.4        # Low-pass filter (0.0 = frozen, 1.0 = raw sensor)
    smoothed_diameter = target_diameter
    
    # --- INTERNAL CONTROLLER STATE ---
    internal_baseline_rpm = 22.0  
    last_error = 0.0
    loop_time = 0.5           # Loop interval (seconds)
    
    # Sliding memory queue to track past speeds
    # Standard assumption: 20 seconds of maximum physical lag history
    max_history_elements = int(20.0 / loop_time)
    speed_history = deque([(time.time(), current_rpm)] * max_history_elements, maxlen=max_history_elements)
    
    while True:
        loop_start_time = time.time()
        actual_raw = get_physical_diameter_mm()
        
        # 1. Apply Exponential Smoothing Filter to RAW Sensor Reading
        if actual_raw is not None:
            smoothed_diameter = (filter_alpha * actual_raw) + ((1.0 - filter_alpha) * smoothed_diameter)
            live_sensor_diameter = smoothed_diameter
        
        # 2. Estimate transit delay based on pulling speed
        estimated_line_speed_mms = max(current_rpm * RPM_TO_MM_PER_SEC, 1.0) # Avoid division by zero
        transit_delay_seconds = DISTANCE_TO_SENSOR_MM / estimated_line_speed_mms
        
        # 3. Log current speed with a timestamp
        speed_history.append((loop_start_time, current_rpm))
        
        # 4. Find the historical speed that actually produced the filament currently at the sensor
        target_lookup_time = loop_start_time - transit_delay_seconds
        
        historical_rpm = internal_baseline_rpm
        for timestamp, past_rpm in reversed(speed_history):
            if timestamp <= target_lookup_time:
                historical_rpm = past_rpm
                break
        
        # 5. Calculate Error with a Deadband (ignores tiny changes below sensor resolution)
        error = live_sensor_diameter - target_diameter
        if abs(error) < (SENSOR_RESOLUTION / 2.0):
            error = 0.0  # Lock onto target if within deadband
        
        # 6. Proportional Term
        p_term = error * Kp
        
        # 7. Integral Term (Anti-windup: only integrate if motor isn't saturated)
        if (current_rpm >= 130 and error > 0) or (current_rpm <= 10 and error < 0):
            pass
        else:
            integral_memory += error * loop_time
            
        # Clamp integral memory to prevent windup delays
        if integral_memory > 15.0:
            integral_memory = 15.0
        elif integral_memory < -15.0:
            integral_memory = -15.0
        
        i_term = integral_memory * Ki
        
        # 8. Derivative Term
        derivative = (error - last_error) / loop_time
        d_term = derivative * Kd
        last_error = error  
        
        # 9. Determine Target RPM
        # Calculates correction relative to the historical speed, avoiding speed freezing
        target_rpm = historical_rpm + p_term + i_term + d_term
        
        # Limit target speed to safe hardware boundaries
        if target_rpm < 10: target_rpm = 10
        if target_rpm > 130: target_rpm = 130
        
        # 10. Apply Smooth Acceleration Ramps
        max_acceleration = 1.5  # Prevents motor step spikes
        if target_rpm > current_rpm + max_acceleration:
            current_rpm += max_acceleration
        elif target_rpm < current_rpm - max_acceleration:
            current_rpm -= max_acceleration
        else:
            current_rpm = target_rpm  
        
        print(f"Raw: {actual_raw:.2f}mm | Smooth: {live_sensor_diameter:.2f}mm | Err: {error:+.2f} | Delay: {transit_delay_seconds:.1f}s | RPM: {current_rpm:.1f}")
        
        # --- SEND TELEMETRY TO MATLAB ---
        try:
            telemetry_string = f"{live_sensor_diameter:.3f},{current_rpm:.1f}"
            udp_socket.sendto(telemetry_string.encode('utf-8'), (MATLAB_IP, MATLAB_PORT))
        except Exception:
            pass  # Ignore network drops to avoid pausing the script
            
        # --- PHYSICAL OUTPUT ---
        if motor_in1 is not None:
            min_rpm, max_rpm = 10.0, 130.0
            min_pwm, max_pwm = 0.15, 1.0  
            
            pwm_duty_cycle = min_pwm + ((current_rpm - min_rpm) * (max_pwm - min_pwm) / (max_rpm - min_rpm))
            motor_in1.value = pwm_duty_cycle

        # Maintain exact loop timing
        elapsed = time.time() - loop_start_time
        time.sleep(max(0.01, loop_time - elapsed))

# =====================================================================
# 5. GUI INTERACTION CODE
# =====================================================================
def apply_settings():
    global target_diameter, integral_memory
    try:
        target_diameter = float(diameter_entry.get())
        integral_memory = 0.0  # Reset integration memory when changing target
        status_label.config(text="Target Updated! Reading physical hardware...", fg="#00ffcc")
    except ValueError:
        status_label.config(text="Error: Enter a valid number for diameter", fg="#ff3333")

def update_gui_readouts():
    live_dia = live_sensor_diameter
    live_rpm = current_rpm
    live_volts = live_sensor_voltage
    
    live_dia_label.config(text=f"{live_dia:.2f} mm")
    live_rpm_label.config(text=f"{int(live_rpm)} RPM")
    voltage_debug_label.config(text=f"ADS1115 AIN0 Voltage: {live_volts:.3f} V")
    
    error = abs(live_dia - target_diameter)
    if error < 0.05:  # Fits the 0.1mm resolution step
        status_label.config(text="Status: Locked Onto Target Zone", fg="#00ffcc")
    elif live_dia < target_diameter:
        status_label.config(text="Status: Filament Thin -> Dropping Speed...", fg="#ffcc00")
    else:
        status_label.config(text="Status: Filament Thick -> Raising Speed...", fg="#ff9900")
    
    root.after(200, update_gui_readouts)

# =====================================================================
# 6. GUI LAYOUT SETUP
# =====================================================================
root = tk.Tk()
root.title("ADS1115 & L298N Closed-Loop Extruder")
root.geometry("650x580")
root.configure(bg="#222222")

title = tk.Label(root, text="HARDWARE CLOSED-LOOP EXTRUDER", font=("Arial", 16, "bold"), bg="#222222", fg="#ffffff")
title.pack(pady=15)

input_frame = tk.LabelFrame(root, text=" 1. Operator Target ", font=("Arial", 12, "bold"), bg="#222222", fg="#00ffcc", padx=15, pady=15)
input_frame.pack(pady=10, fill="x", padx=30)

tk.Label(input_frame, text="Required Diameter:", bg="#222222", fg="white", font=("Arial", 11)).pack(side="left", padx=5)
diameter_entry = tk.Entry(input_frame, font=("Arial", 12), bg="#333333", fg="#ffffff", width=8, insertbackground="white")
diameter_entry.pack(side="left", padx=10)
diameter_entry.insert(0, "1.75")
tk.Label(input_frame, text="mm", bg="#222222", fg="white").pack(side="left")

apply_btn = tk.Button(root, text="APPLY TO MACHINE", command=apply_settings, font=("Arial", 12, "bold"), bg="#00ffcc", fg="#222222", activebackground="#00ccaa", padx=20)
apply_btn.pack(pady=15)

dash_frame = tk.LabelFrame(root, text=" 2. Live Machine Dashboard (Sensors) ", font=("Arial", 12, "bold"), bg="#222222", fg="#ffcc00", padx=15, pady=15)
dash_frame.pack(pady=10, fill="x", padx=30)

tk.Label(dash_frame, text="Measured Filament Dia:", font=("Arial", 12), bg="#222222", fg="#aaaaaa").grid(row=0, column=0, sticky="w", pady=10)
live_dia_label = tk.Label(dash_frame, text="0.00 mm", font=("Arial", 18, "bold"), bg="#222222", fg="#ffffff")
live_dia_label.grid(row=0, column=1, padx=40, sticky="e")

tk.Label(dash_frame, text="Live Motor Speed:", font=("Arial", 12), bg="#222222", fg="#aaaaaa").grid(row=1, column=0, sticky="w", pady=10)
live_rpm_label = tk.Label(dash_frame, text="60 RPM", font=("Arial", 18, "bold"), bg="#222222", fg="#ffffff")
live_rpm_label.grid(row=1, column=1, padx=40, sticky="e")

voltage_debug_label = tk.Label(dash_frame, text="ADS1115 AIN0 Voltage: 0.000 V", font=("Arial", 10, "italic"), bg="#222222", fg="#888888")
voltage_debug_label.grid(row=2, column=0, columnspan=2, sticky="w", pady=5)

status_label = tk.Label(root, text="Status: Hardware Active", font=("Arial", 11, "italic"), bg="#222222", fg="#888888")
status_label.pack(pady=15)

# =====================================================================
# 7. RUN BACKGROUND THREADS AND LOOPS
# =====================================================================
control_thread = threading.Thread(target=closed_loop_worker, daemon=True)
control_thread.start()

root.after(200, update_gui_readouts)
root.mainloop()
