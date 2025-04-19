import RPi.GPIO as GPIO
import time

# 🔸 Define the GPIO pin for ESC control
ESC_PIN = 18  # Update this based on your wiring

# 🔸 Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# 🔸 Initialize PWM (ESC typically operates at 50Hz)
pwm = GPIO.PWM(ESC_PIN, 50)
pwm.start(7.5)  # Neutral signal (adjust based on ESC requirements)

def set_throttle(duty_cycle):
    """Set motor speed with PWM duty cycle."""
    pwm.ChangeDutyCycle(duty_cycle)

try:
    print("🚀 Arming ESC...")
    set_throttle(7.5)  # Neutral signal for arming (adjust if needed)
    time.sleep(2)

    print("🌀 Running motor continuously...")
    set_throttle(10)  # Adjust this duty cycle for your desired speed

    while True:
        time.sleep(1)  # Keep running until stopped manually

except KeyboardInterrupt:
    print("\n🔴 Stopping motor...")
    set_throttle(7.5)  # Return to neutral before stopping
    time.sleep(2)
    pwm.stop()
    GPIO.cleanup()
    print("✅ Motor stopped safely.")