# 3D Scanner — Low-Cost Spatial Data Acquisition System

**Daniel Reynolds — REYN Consultancy / La Trobe University, 2024**

A multidisciplinary mechatronic project exploring low-cost 3D scanning using ultrasonic and Time-of-Flight sensors — deliberately built without Arduino or off-the-shelf breakout libraries. The goal was to understand the hardware, not abstract it away.

---

## What This Is

A complete scanning system built from scratch: custom mechanical assembly, Raspberry Pi Pico W firmware in MicroPython, wireless BLE communication, and a Python desktop GUI for real-time 3D point cloud visualisation.

The system sweeps a sensor through a configurable pan/tilt range and transmits live distance data over BLE to the host. The host converts spherical coordinates to Cartesian, renders a live 3D point cloud, and exports scan data to CSV.

---

## Why It Was Built This Way

Most sensor projects use Arduino with pre-built abstraction layers. This project instead used a **Raspberry Pi Pico W with MicroPython**, implementing BLE GATT services directly on the Pico W, integrating sensors and servo control at firmware level, and designing a custom carrier PCB to eliminate the development board entirely.

Mechanically, the scanner uses a **geared pan mechanism with ball-bearing-supported rotation** rather than direct servo drive — reducing mechanical error, distributing servo loads through the gear train, and improving scan repeatability. A tube attachment focuses the ultrasonic beam to mitigate cone-angle spread.

This was a deliberate engineering decision at every level: understand the system, don't just make it work.

---

## System Architecture

```
┌─────────────────────────────────────┐
│         Raspberry Pi Pico W         │
│  MicroPython firmware (main.py)     │
│                                     │
│  ┌─────────┐  ┌──────────────────┐  │
│  │HC-SR04  │  │VL53L0X ToF (I2C) │  │
│  │Ultrasonic│  │(selectable)      │  │
│  └─────────┘  └──────────────────┘  │
│                                     │
│  ┌──────────┐  ┌───────────────┐   │
│  │Pan Servo │  │Tilt Servo     │   │
│  │(geared)  │  │(direct)       │   │
│  └──────────┘  └───────────────┘   │
│                                     │
│  BLE Nordic UART Service (NUS)      │
└──────────────┬──────────────────────┘
               │ Bluetooth Low Energy
               │ JSON-style command/data packets
               ▼
┌─────────────────────────────────────┐
│     Python Host GUI (PyQt5)         │
│  GUI_Matplotlib_Final_ver.py        │
│                                     │
│  BleakClient (async BLE)            │
│  Spherical → Cartesian conversion   │
│  4-position coordinate transforms   │
│  Matplotlib 3D point cloud          │
│  Delaunay / ConvexHull rendering    │
│  CSV export (raw + processed)       │
└─────────────────────────────────────┘
```

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Raspberry Pi Pico W |
| ToF Sensor | VL53L0X (I2C, pins 12/13) |
| Ultrasonic | HC-SR04 (pins 14/15) |
| Pan servo | SG90 via geared mechanism |
| Tilt servo | FS5113M direct |
| Structure | 3D resin-printed parts |
| Bearings | Steel sealed ball bearings |
| Gears | ISO spur gears, 0.8M 20T/27T |
| PCB | Custom Pico carrier board |
| Laser pointer | Alignment aid (pin 17) |

**Assembly drawing:** [`docs/scanner_assm_ver4_REVA.pdf`](docs/scanner_assm_ver4_REVA.pdf)  
Full A1 GA with plan, elevation, section cuts, exploded isometric, and parts list.

---

## Repository Structure

```
3d-scanner/
├── pico/                          # Runs on Raspberry Pi Pico W
│   ├── main.py                    # Main firmware — scan state machine, BLE, sensors
│   ├── ble_simple_peripheral.py   # BLE GATT peripheral (Nordic UART Service)
│   └── ble_advertising.py         # GAP advertising payload builder
│
├── host/                          # Runs on PC / Mac
│   └── GUI_Matplotlib_Final_ver.py # PyQt5 GUI — BLE client, visualisation, CSV
│
└── docs/
    ├── scanner_assm_ver4_REVA.pdf  # Engineering assembly drawing
    └── technical_report.pdf        # Full technical report
```

---

## BLE Protocol

Communication uses the **Nordic UART Service (NUS)** — a well-established BLE UART profile.

### Commands (PC → Pico)

| Command | Description |
|---------|-------------|
| `sensor=ultra` / `sensor=tof` | Select active sensor |
| `pan_start=N` / `pan_end=N` | Set pan range (0–180°) |
| `tilt_start=N` / `tilt_end=N` | Set tilt range (0–180°) |
| `move_pan=N` / `move_tilt=N` | Manual servo positioning |
| `min_distance=N` / `max_distance=N` | Filter thresholds (mm) |
| `start` | Begin scan sequence |
| `stop` | Halt scan |
| `pause` / `resume` | Suspend / continue scan |
| `pointer_on` / `pointer_off` | Laser pointer toggle |

### Data (Pico → PC)

| Message | Description |
|---------|-------------|
| `pan=N,tilt=N,distance=N` | Scan data point |
| `scan_complete` | End of scan notification |
| `error: <message>` | Error response |

---

## Software Stack

**Pico (MicroPython):**
- `bluetooth` — BLE GATT server, Nordic UART Service
- `vl53l0x` — VL53L0X ToF driver
- `servo` — PWM servo control

**Host (Python 3):**
```
pip install PyQt5 matplotlib numpy scipy bleak
```

| Library | Use |
|---------|-----|
| `bleak` | Async BLE client |
| `PyQt5` | GUI framework |
| `matplotlib` | 3D point cloud visualisation |
| `numpy` | Coordinate transforms, filtering |
| `scipy` | Delaunay triangulation, ConvexHull |

---

## Running It

**On the Pico:**
1. Flash MicroPython to Pico W
2. Copy `pico/main.py`, `pico/ble_simple_peripheral.py`, `pico/ble_advertising.py` to Pico filesystem
3. Also copy a `servo.py` PWM servo library (e.g. `micropython-servo`)
4. Power on — Pico advertises as `DR_PICOW`

**On the host:**
```bash
pip install PyQt5 matplotlib numpy scipy bleak
python host/GUI_Matplotlib_Final_ver.py
```
1. Click **Connect** — scans for `DR_PICOW`
2. Set scan parameters in **Servo Angles** tab
3. Select sensor type, set position (1–4 corners)
4. Click **Scan** to start

---

## Multi-Position Scanning

The system supports **4 scanning positions** (room corners) with automatic coordinate transforms. Each position applies a rotation + translation so data from all four positions stitches into a single coordinate space.

| Position | Corner | Rotation |
|----------|--------|---------|
| 1 | Front Left | 0° |
| 2 | Front Right | 90° |
| 3 | Rear Left | −90° |
| 4 | Rear Right | 180° |

---

## Key Engineering Decisions

**Geared pan mechanism** — servo torque transferred through spur gears to a ball-bearing-supported rotating platform. Reduces backlash transmitted to sensor, improves repeatability. Backlash compensation routine in firmware overshoots start position and returns to ensure consistent gear tooth engagement.

**Dual sensor support** — ultrasonic (HC-SR04) and ToF (VL53L0X) selectable at runtime over BLE. Tube attachment fitted to ultrasonic sensor to focus beam and reduce cone-angle measurement error.

**Custom carrier PCB** — Pico W operated without development board. P-channel MOSFET protection circuit (from Pico datasheet) designed to protect VBUS during operation.

**Asynchronous BLE** — host uses `asyncio` + `bleak` in a dedicated thread. Motor control and BLE transmit run asynchronously on the Pico to prevent interference between scan sequencing and data transmission.

---

## Demonstrated Capabilities

- MicroPython BLE GATT peripheral implementation from scratch
- Dual sensor integration with runtime switching
- Pan/tilt servo control with backlash compensation
- Spherical-to-Cartesian coordinate conversion
- Asynchronous BLE communication (both ends)
- Real-time BLE GUI control and live visualisation
- PyQt5 GUI with embedded Matplotlib 3D canvas
- Multi-position scan merging via rotation matrices
- Outlier removal (Z-score), spatial averaging, Delaunay/ConvexHull rendering
- Custom mechanical design: gears, bearings, 3D-printed structure, custom PCB

---

## Demo

📹 **Full system demonstration and project presentation**  
https://youtu.be/Q07UHdNxgic?si=WPGpGwHWOMIRFbne

⚡ **Quick BLE GUI ↔ Pico W control demonstration**  
https://youtube.com/shorts/xAAtb4II7n4?si=r3o97eI5b7I6jdhX

---

## Acknowledgements

Developed as part of engineering coursework at La Trobe University.  
Mechanical design and firmware by Daniel Reynolds — REYN Consultancy.

---

## Contact

📧 Daniel@reyn.com.au  
🔗 [reyn.com.au](https://reyn.com.au)  
💼 [github.com/ReynDaniel](https://github.com/ReynDaniel)
