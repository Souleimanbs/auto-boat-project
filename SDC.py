#!/usr/bin/env python3
"""
Autonomous Boat System
Sensors: DHT11, DS18B20, Ultrasonic, Flame Sensor, GPS
Camera: libcamera
AI Model: Faster R-CNN (PyTorch)
Control: Servo Motor for steering, ESC for propulsion
"""
import time
import cv2
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image
import serial
import pynmea2
import requests
import RPi.GPIO as GPIO
import adafruit_dht
import subprocess

# ------------------------------
# ✅ GPIO Setup
# ------------------------------
GPIO.setmode(GPIO.BCM)

# --- DHT11 Sensor ---
DHT_PIN = 4
dht_device = adafruit_dht.DHT11(DHT_PIN)
def read_dht11():
    try:
        temperature = dht_device.temperature
        humidity = dht_device.humidity
        return round(temperature, 2), round(humidity, 2)
    except RuntimeError:
        return None, None

# --- DS18B20 Water Temperature Sensor ---
DS18B20_SENSOR = "/sys/bus/w1/devices/28-xxxxxxxxxxxx/w1_slave"
def read_ds18b20():
    try:
        with open(DS18B20_SENSOR, "r") as f:
            lines = f.readlines()
        if "YES" in lines[0]:
            temp_idx = lines[1].find("t=")
            temp_c = float(lines[1][temp_idx+2:]) / 1000.0
            return round(temp_c, 2)
    except:
        return None

# --- Ultrasonic Sensors ---
TRIG_DEPTH = 23
ECHO_DEPTH = 24
TRIG_AVOID = 20
ECHO_AVOID = 21
GPIO.setup(TRIG_DEPTH, GPIO.OUT)
GPIO.setup(ECHO_DEPTH, GPIO.IN)
GPIO.setup(TRIG_AVOID, GPIO.OUT)
GPIO.setup(ECHO_AVOID, GPIO.IN)

def read_ultrasonic(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    start = time.time()
    while GPIO.input(echo) == 0:
        start = time.time()
    while GPIO.input(echo) == 1:
        end = time.time()
    distance = (end - start) * 17150
    return round(distance, 2)

def read_depth_ultrasonic():
    return read_ultrasonic(TRIG_DEPTH, ECHO_DEPTH)

def read_obstacle_ultrasonic():
    return read_ultrasonic(TRIG_AVOID, ECHO_AVOID)

# --- Flame Sensor ---
FLAME_SENSOR_PIN = 17
GPIO.setup(FLAME_SENSOR_PIN, GPIO.IN)
def read_flame_sensor():
    return "Fire detected!" if GPIO.input(FLAME_SENSOR_PIN) == 0 else "No fire."

# ------------------------------
# ✅ GPS Setup
# ------------------------------
GPS_PORT = "/dev/ttyAMA0"
gps_serial = serial.Serial(GPS_PORT, baudrate=9600, timeout=1)
def read_gps():
    while True:
        try:
            line = gps_serial.readline().decode("ascii", errors="replace")
            if line.startswith("$GPGGA"):
                msg = pynmea2.parse(line)
                return msg.latitude, msg.longitude, str(msg.timestamp)
        except:
            continue

# ------------------------------
# ✅ Camera (libcamera)
# ------------------------------
IMAGE_PATH = "/home/pi/detection_image.jpg"
def capture_image():
    subprocess.run(["libcamera-still", "-o", IMAGE_PATH, "--timeout", "500"], check=True)
    return IMAGE_PATH

# ------------------------------
# ✅ AI Object Detection Setup
# ------------------------------
MODEL_PATH = "fishing_detection_model.pth"
model = torch.load(MODEL_PATH, map_location=torch.device("cpu"))
model.eval()
CLASSES = ["background", "swimmer", "boat", "jetski", "life-saving appliance", "buoy"]

def detect_objects(image_path, conf_threshold=0.5):
    img = Image.open(image_path).convert("RGB")
    transform = T.Compose([T.ToTensor()])
    tensor_img = transform(img).unsqueeze(0)
    with torch.no_grad():
        detections = model(tensor_img)

    for i in range(len(detections[0]["labels"])):
        confidence = detections[0]["scores"][i].item()
        if confidence > conf_threshold:
            label_idx = detections[0]["labels"][i].item()
            label = CLASSES[label_idx]
            if label == "boat":
                return label, detections[0]["boxes"][i].tolist(), confidence
    return None, None, None

# ------------------------------
# ✅ Boat Control (Servo & ESC)
# ------------------------------
SERVO_PIN = 18
ESC_PIN = 25
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(ESC_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
motor = GPIO.PWM(ESC_PIN, 100)
servo.start(7.5)
motor.start(0)

def set_servo_angle(angle):
    duty = 7.5 + (angle / 18.0)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.3)

def follow_target():
    set_servo_angle(-10)
    motor.ChangeDutyCycle(50)

def go_straight():
    set_servo_angle(0)
    motor.ChangeDutyCycle(0)

# ------------------------------
# ✅ Send Data to Server
# ------------------------------
def send_data_to_server(payload):
    server_url = "http://your-server.com/api/receive_data"
    headers = {"Content-Type": "application/json", "Authorization": "Bearer <YOUR_API_KEY>"}
    try:
        response = requests.post(server_url, json=payload, headers=headers, timeout=5)
        print("Data sent. Status:", response.status_code)
    except:
        print("Error sending data.")

# ------------------------------
# ✅ Main Execution Loop
# ------------------------------
try:
    while True:
        ambient_temp, humidity = read_dht11()
        water_temp = read_ds18b20()
        depth_distance = read_depth_ultrasonic()
        obstacle_distance = read_obstacle_ultrasonic()
        flame_status = read_flame_sensor()
        image_file = capture_image()
        detected_label, bbox, confidence = detect_objects(image_file)

        if detected_label:
            follow_target()
        else:
            go_straight()

        latitude, longitude, gps_time = read_gps()
        payload = {
            "ambient_temperature": ambient_temp,
            "humidity": humidity,
            "water_temperature": water_temp,
            "depth_distance": depth_distance,
            "obstacle_distance": obstacle_distance,
            "flame_status": flame_status,
            "image": image_file,
            "gps": {"latitude": latitude, "longitude": longitude, "timestamp": gps_time},
            "detection": {"target": detected_label if detected_label else "none", "bbox": bbox if bbox else [], "confidence": confidence if confidence else 0}
        }

        send_data_to_server(payload)
        time.sleep(10)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    servo.stop()
    motor.stop()
    GPIO.cleanup()