"""
3D Scanner — Pico W Firmware
Daniel Reynolds — REYN Consultancy / La Trobe University, 2024

Handles BLE UART communication, dual sensor support (ultrasonic + VL53L0X ToF),
pan/tilt servo control with backlash compensation, and scan sequencing.
Communicates with host Python GUI via Nordic UART Service (NUS) over BLE.

Hardware:
    - Raspberry Pi Pico W
    - VL53L0X Time-of-Flight sensor (I2C, pins 12/13)
    - HC-SR04 ultrasonic sensor (pins 14/15)
    - 2x servo motors — pan (pin 1) and tilt (pin 0)
    - Laser pointer (pin 17), flashing LED (pin 18)
    - Push button (pin 16)

Commands received over BLE:
    sensor=ultra / sensor=tof   — select active sensor
    pan_start=N / pan_end=N     — set pan range (0-180 deg)
    tilt_start=N / tilt_end=N   — set tilt range (0-180 deg)
    move_pan=N / move_tilt=N    — manual servo positioning
    min_distance=N / max_distance=N — filter thresholds (mm)
    start                       — begin scan sequence
    stop                        — halt scan
    pause / resume              — suspend/continue scan
    pointer_on / pointer_off    — laser pointer control
    led_on / led_off            — onboard LED control

Data transmitted over BLE:
    pan=N,tilt=N,distance=N     — scan data point
    scan_complete               — end of scan notification
    error: <message>            — error response
"""

from machine import Pin, I2C
from ble_simple_peripheral import BLESimplePeripheral
from servo import Servo
import utime
import time
import vl53l0x
import bluetooth
import sys

# Constants for angle limits
PAN_ANGLE_MIN = 0
PAN_ANGLE_MAX = 180
TILT_ANGLE_MIN = 0
TILT_ANGLE_MAX = 180

# BLE UUIDs — Nordic UART Service (NUS)
UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
UART_RX_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # RX (write)
UART_TX_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # TX (notify)

# Pin assignments
I2C_SCL_PIN = 13
I2C_SDA_PIN = 12

TILT_SERVO_PIN = 0
PAN_SERVO_PIN = 1

ECHO_PIN = 14
TRIGGER_PIN = 15

BUTTON_PIN = 16
POINTER_PIN = 17
FLASHING_LED_PIN = 18

ONBOARD_LED_PIN = 'LED'  # For Pico W

# Set up onboard LED
led = Pin(ONBOARD_LED_PIN, Pin.OUT)

# BLE setup
ble = bluetooth.BLE()
ble.active(True)
sp = BLESimplePeripheral(ble)

distance = 1000  # Placeholder for out-of-bounds distance
max_distance = 2000  # Large error trigger
min_distance = 50  # Minimum valid distance in mm

# Initialize I2C for VL53L0X ToF sensor
i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN))

# Servo setup
tilt_servo = Servo(pin_id=TILT_SERVO_PIN)
pan_servo = Servo(pin_id=PAN_SERVO_PIN)

# Ultrasonic sensor setup
echo = Pin(ECHO_PIN, Pin.IN)
trigger = Pin(TRIGGER_PIN, Pin.OUT)

# Button and LED setup
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
pointer = Pin(POINTER_PIN, Pin.OUT)
fl_led = Pin(FLASHING_LED_PIN, Pin.OUT)

# Scan state variables
sensor_type = None  # 'ultra' or 'tof'
paused = False
stop_scan = False
servo_active = False
delay_ms = 80
delay_lg = 200
debounce_time = 0.2

# Scan parameters
pan_start = None
pan_end = None
tilt_start = None
tilt_end = None
sensor = None
pan_inc = 2
tilt_inc = 2

# Current scan position
pan_angle = None
tilt_angle = None
scan_complete_flag = False


# ─────────────────────────────────────────────
# Sensor functions
# ─────────────────────────────────────────────

def ultra():
    """Read distance from HC-SR04 ultrasonic sensor. Returns distance in mm or None on timeout."""
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()
    signaloff = 0
    signalon = 0
    timeout = 1000000  # 1 second timeout in microseconds
    start_time = utime.ticks_us()
    while echo.value() == 0:
        signaloff = utime.ticks_us()
        if utime.ticks_diff(signaloff, start_time) > timeout:
            return None
    while echo.value() == 1:
        signalon = utime.ticks_us()
        if utime.ticks_diff(signalon, start_time) > timeout:
            return None
    timepassed = signalon - signaloff
    distance = (timepassed * 0.343) / 2  # Speed of sound: 343 m/s, round trip
    return round(distance, 1)


def tof():
    """Initialise VL53L0X ToF sensor."""
    global sensor
    sensor = vl53l0x.VL53L0X(i2c)
    sensor.start()
    sensor.set_measurement_timing_budget(20000)


# ─────────────────────────────────────────────
# Servo control functions
# ─────────────────────────────────────────────

def move_servo_smoothly(servo, start_angle, end_angle, step, delay):
    """Move servo from start_angle to end_angle in increments of step with delay (ms)."""
    if start_angle > end_angle:
        step = -abs(step)
    else:
        step = abs(step)
    for angle in range(start_angle, end_angle + step, step):
        servo.write(angle)
        time.sleep(delay / 1000.0)


def remove_backlash(servo, start_angle, backlash_angle=20):
    """
    Compensate for gear backlash by overshooting and returning to start position.
    Critical for geared pan mechanism — ensures consistent tooth engagement.
    """
    backlash_comp_angle = max(0, start_angle - backlash_angle)
    move_servo_smoothly(servo, start_angle, backlash_comp_angle, -1, 50)
    move_servo_smoothly(servo, backlash_comp_angle, start_angle, 1, 50)


def move_servos_to_start_positions():
    """Move both servos to scan start positions before beginning."""
    if pan_start is not None and tilt_start is not None:
        pan_servo.write(pan_start)
        tilt_servo.write(tilt_start)
        time.sleep(0.5)


# ─────────────────────────────────────────────
# Scan state machine
# ─────────────────────────────────────────────

def scan_step():
    """Execute one step of the scan sequence — move servos, read sensor, transmit data."""
    global distance, stop_scan, paused, pan_angle, tilt_angle, scan_complete_flag

    if stop_scan or scan_complete_flag:
        return
    if paused:
        return

    pan_step = pan_inc if pan_start <= pan_end else -pan_inc
    tilt_step = tilt_inc if tilt_start <= tilt_end else -tilt_inc

    if pan_angle is None:
        pan_angle = pan_start
    if tilt_angle is None:
        tilt_angle = tilt_start

    pan_servo.write(pan_angle)
    tilt_servo.write(tilt_angle)
    time.sleep_ms(delay_ms)

    # Read sensor
    if sensor_type == 'ultra':
        distance = ultra()
        if distance is None:
            distance = max_distance
    elif sensor_type == 'tof':
        try:
            distance = sensor.read()
            if distance is None:
                distance = max_distance
        except OSError:
            distance = max_distance
    else:
        return

    # Transmit valid data point over BLE
    if min_distance <= distance <= max_distance:
        data = f"pan={pan_angle},tilt={tilt_angle},distance={distance}"
        sp.send(data.encode('utf-8'))
        time.sleep_ms(50)

    # Advance scan position (tilt inner loop, pan outer loop)
    if tilt_start <= tilt_end:
        if tilt_angle + tilt_step <= tilt_end:
            tilt_angle += tilt_step
        else:
            tilt_angle = tilt_start
            if pan_start <= pan_end:
                if pan_angle + pan_step <= pan_end:
                    pan_angle += pan_step
                else:
                    scan_complete_flag = True
            else:
                if pan_angle + pan_step >= pan_end:
                    pan_angle += pan_step
                else:
                    scan_complete_flag = True
    else:
        if tilt_angle + tilt_step >= tilt_end:
            tilt_angle += tilt_step
        else:
            tilt_angle = tilt_start
            if pan_start <= pan_end:
                if pan_angle + pan_step <= pan_end:
                    pan_angle += pan_step
                else:
                    scan_complete_flag = True
            else:
                if pan_angle + pan_step >= pan_end:
                    pan_angle += pan_step
                else:
                    scan_complete_flag = True


# ─────────────────────────────────────────────
# BLE command handler
# ─────────────────────────────────────────────

def process_command(command):
    """Parse and execute a command string received over BLE."""
    global sensor_type, servo_active, stop_scan, paused, pointer, \
            sensor, pan_start, pan_end, tilt_start, tilt_end, \
            pan_inc, tilt_inc, min_distance, max_distance, \
            pan_angle, tilt_angle, scan_complete_flag

    command = command.strip()
    try:
        if command == "led_on":
            led.on()
        elif command == "led_off":
            led.off()
        elif command.startswith("min_distance="):
            min_distance = int(command.split('=')[1])
        elif command.startswith("max_distance="):
            max_distance = int(command.split('=')[1])
        elif command == "pointer_on":
            pointer.on()
            fl_led.on()
        elif command == "pointer_off":
            pointer.off()
            fl_led.off()
        elif command == "pause":
            paused = True
        elif command == "resume":
            paused = False
        elif command == "stop":
            stop_scan = True
        elif command == "sensor=ultra":
            sensor_type = 'ultra'
        elif command == "sensor=tof":
            sensor_type = 'tof'
            tof()
        elif command.startswith("pan_start="):
            val = int(command.split('=')[1])
            if PAN_ANGLE_MIN <= val <= PAN_ANGLE_MAX:
                pan_start = val
                pan_angle = None
            else:
                sp.send(f"error: invalid pan_start {val}".encode('utf-8'))
        elif command.startswith("pan_end="):
            val = int(command.split('=')[1])
            if PAN_ANGLE_MIN <= val <= PAN_ANGLE_MAX:
                pan_end = val
            else:
                sp.send(f"error: invalid pan_end {val}".encode('utf-8'))
        elif command.startswith("tilt_start="):
            val = int(command.split('=')[1])
            if TILT_ANGLE_MIN <= val <= TILT_ANGLE_MAX:
                tilt_start = val
                tilt_angle = None
            else:
                sp.send(f"error: invalid tilt_start {val}".encode('utf-8'))
        elif command.startswith("tilt_end="):
            val = int(command.split('=')[1])
            if TILT_ANGLE_MIN <= val <= TILT_ANGLE_MAX:
                tilt_end = val
            else:
                sp.send(f"error: invalid tilt_end {val}".encode('utf-8'))
        elif command.startswith("move_pan="):
            val = int(command.split('=')[1])
            if PAN_ANGLE_MIN <= val <= PAN_ANGLE_MAX:
                pan_servo.write(val)
            else:
                sp.send(f"error: invalid pan angle {val}".encode('utf-8'))
        elif command.startswith("move_tilt="):
            val = int(command.split('=')[1])
            if TILT_ANGLE_MIN <= val <= TILT_ANGLE_MAX:
                tilt_servo.write(val)
            else:
                sp.send(f"error: invalid tilt angle {val}".encode('utf-8'))
        elif command == "start":
            if all(v is not None for v in [pan_start, pan_end, tilt_start, tilt_end]):
                servo_active = True
                stop_scan = False
                paused = False
                pan_angle = None
                tilt_angle = None
                scan_complete_flag = False
                pointer.on()
                fl_led.on()
            else:
                sp.send("error: Angles not set. Cannot start scan.".encode('utf-8'))
    except Exception:
        pass


def on_rx_write(data):
    """BLE write callback — decode and process incoming command."""
    command = data.decode("utf-8").strip()
    process_command(command)


# Register BLE write callback
sp.on_write(on_rx_write)

# ─────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────

prev_connected = False

while True:
    if sp.is_connected():
        if not prev_connected:
            led.on()
            prev_connected = True
            move_servos_to_start_positions()
        if servo_active and not stop_scan and not scan_complete_flag:
            scan_step()
        elif scan_complete_flag:
            servo_active = False
            pointer.off()
            fl_led.off()
            sp.send('scan_complete'.encode('utf-8'))
            scan_complete_flag = False
        utime.sleep_ms(50)
    else:
        if prev_connected:
            led.off()
            prev_connected = False
        utime.sleep_ms(100)
