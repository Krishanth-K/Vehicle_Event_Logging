# Vehicle Event Logging and Response System
### Revised Project Plan — STM32F401CCUx | UART-to-PC Emergency Response

> **Design philosophy:** The STM32F401CCUx reads data from vehicle-mounted sensors,
> detects abnormal conditions, logs timestamped events to an SD card, and transmits
> structured emergency alert messages over UART to a PC — simulating a real-world
> emergency notification pipeline. The system is **read-only** (sensor monitoring
> only); it does not actuate or control any vehicle component.

---

## 1. Problem Statement

A vehicle can experience abnormal conditions such as sudden impact, excessive
temperature, nearby obstacles, abnormal electrical conditions, and low-light
environments. Without continuous monitoring, these events may not be detected,
recorded, or reported in time.

The proposed system is an **STM32F401CCUx-based Vehicle Event Logging and Response
System** that:
- Continuously reads vehicle parameters from sensors
- Detects abnormal events by comparing values against defined thresholds
- Provides an immediate local response through LEDs and a buzzer
- Timestamps each event using an RTC
- Logs every event persistently to an SD card
- Transmits structured emergency alert messages to a PC via UART — simulating
  notification of emergency services

> **Lab demonstration note:** UART messages are received on a PC serial terminal
> (e.g. PuTTY / CoolTerm) via a USB-to-UART adapter, representing the emergency
> services notification channel.

---

## 2. Objectives / Deliverables

| Objective | Implementation / Deliverable |
|---|---|
| Temperature monitoring | LM35 + ADC; temperature value and over-temperature alert |
| Impact detection | MPU6050; sudden acceleration / tilt or possible impact event |
| Obstacle detection | IR sensor + HC-SR04; obstacle presence and distance warning |
| Light monitoring | LDR + ADC; low-light detection |
| Electrical monitoring | INA219; voltage / current warning |
| Speed monitoring | Hall sensor; RPM / speed calculation |
| Local response | LEDs and buzzer for normal / warning / emergency indication |
| Event logging | RTC timestamps events; SD card stores event history |
| PC communication | UART → USB-TTL → PC terminal; structured plain-text emergency alerts |
| Live display | OLED shows current sensor values and system status |
| Testing | Potentiometer used for controlled analog fault simulation |

---

## 3. Hardware Specifications

| Hardware | Have / Buy | Role / Specification | Embedded Concept |
|---|---|---|---|
| **STM32F401CCUx** (Black Pill) | Buy | Main controller — 84 MHz Cortex-M4, 256 KB Flash, 64 KB RAM; GPIO, ADC, I²C, SPI, USART, Timers, NVIC | Cortex-M4, all peripheral protocols |
| MPU6050 | Have | 3-axis accelerometer + gyroscope; impact, braking and tilt detection | I²C, interrupts, event detection |
| LM35 | Have | Analog temperature sensor for motor / vehicle temperature | ADC, threshold logic |
| LDR | Have | Light-intensity sensing for day / night or low-light detection | ADC, GPIO |
| IR sensor | Have | Digital obstacle or edge/pulse detection | GPIO, edge events |
| MAX30100 | Have | Optional driver health monitoring (heart rate / SpO₂) | I²C, data processing |
| Potentiometer | Have | Adjustable analog input for testing / fault simulation | ADC, testing |
| HC-SR04 | Buy | Accurate obstacle-distance measurement | GPIO + timer (echo pulse) |
| INA219 | Buy | Current and bus-voltage monitoring | I²C, arithmetic |
| Hall sensor + magnet | Buy | Wheel rotation and speed measurement | GPIO, timer, interrupt |
| I²C OLED (0.96") | Buy | Live display of values and system status | I²C, display interfacing |
| DS3231 RTC module | Buy | Timestamp vehicle events | I²C, RTC, timekeeping |
| Micro-SD module | Buy | Permanent event-history storage | SPI, memory, data logging |
| Micro-SD card | Buy | Store event logs | SPI, memory |
| Buzzer | Buy | Audible warning output | GPIO |
| LEDs + push buttons | Buy | Visual status indicators and manual input | GPIO |
| USB-to-UART adapter (CP2102 / CH340) | Buy | Connect STM32 USART2 to PC COM port | UART, serial communication |
| LM2596 buck converter / USB power | Buy | Regulated 3.3 V / 5 V prototype power | Power management |
| Breadboard, jumper wires, passives | Buy | Prototype assembly and interfacing | Hardware interfacing |

---

## 4. Embedded System Concepts Mapping

| Concept | Where Used | What It Demonstrates |
|---|---|---|
| Cortex-M4 architecture | STM32F401CCUx main controller | Dedicated real-time embedded processing on an ARM MCU |
| GPIO | IR, LEDs, buzzer, buttons | Digital input / output control |
| ADC | LM35, LDR, potentiometer | Analog signal → digital value conversion |
| I²C | MPU6050, MAX30100, INA219, OLED, RTC | Multi-device two-wire digital sensor / peripheral bus |
| **UART** | **USART2 → USB-TTL → PC** | **Serial communication; transmit emergency alert messages to PC at 115200 baud** |
| SPI | SD card | Fast synchronous serial interface for data storage |
| Timers | Hall sensor, HC-SR04 echo timing, periodic sampling | Time measurement, pulse counting, and time-base generation |
| Interrupts / NVIC / ISR | MPU6050 INT, IR edge, Hall pulse, emergency events | Immediate, deterministic event response |
| SysTick / periodic timing | Sensor sampling, display refresh, UART telemetry | Real-time scheduling at fixed intervals |
| RTC | DS3231 | Real-world date / time for event timestamps |
| Registers / bit operations | Peripheral configuration | GPIO / ADC / timer / UART setup at register level |
| Memory | RAM + Flash + SD card | Program in Flash, runtime data in RAM, persistent logs on SD |
| Testing / debugging | Potentiometer + controlled fault conditions | Fault injection and threshold validation |
| Performance measurement | Timers / software | Event-to-response timing and sampling-rate measurement |

---

## 5. System Flow — 10-Stage Event Pipeline

| Step | Stage | Example | STM32 Hardware / Peripheral | Output | Embedded Concept |
|---|---|---|---|---|---|
| 1 | **Sense** | LM35 reads temperature | ADC | Temperature value | Analog acquisition |
| 2 | **Acquire** | MPU6050 provides acceleration | I²C | Acceleration data | Digital sensor interfacing |
| 3 | **Measure** | Hall sensor pulse / HC-SR04 echo | Timer + GPIO | RPM / distance | Time measurement |
| 4 | **Detect** | Temperature or acceleration exceeds threshold | CPU + logic | Event flag | Arithmetic / logical processing |
| 5 | **Interrupt** | Critical event occurs | NVIC + ISR | Immediate event handling | Interrupt-driven response |
| 6 | **Respond** | Activate buzzer + LEDs | GPIO | Local alarm (audible + visual) | Real-time GPIO control |
| 7 | **Timestamp** | Read date / time from RTC | I²C + RTC (DS3231) | Timestamp string | Event traceability |
| 8 | **Log** | Write event record to SD card | SPI + SD | Stored CSV event record | Persistent data logging |
| 9 | **Communicate** | Send alert to PC via UART | USART2 → USB-TTL → PC | Structured plain-text emergency message | Serial communication |
| 10 | **Display** | Show live values and status | I²C OLED | Local dashboard | Human-machine interface |

---

## 6. UART Emergency Message Protocol

Emergency and status messages are transmitted over **USART2 at 115200 baud, 8N1**
to a PC serial terminal via a USB-to-UART adapter.

### Message Format

```
[LEVEL] EVENT_TYPE | field1=value | field2=value | time=HH:MM:SS
```

| Level | Meaning | Example trigger |
|---|---|---|
| `[INFO]` | Normal periodic telemetry | Every 1 s sensor reading |
| `[WARNING]` | Threshold crossed — attention needed | Temp > 70 C, obstacle < 30 cm |
| `[EMERGENCY]` | Critical condition — simulates emergency services notification | Impact detected, voltage < 3.5 V |

### Example Messages

```
[INFO]      TELEMETRY  | temp=45C  | ax=0.01g  | dist=120cm | rpm=960 | time=14:22:00
[WARNING]   OVER_TEMP  | temp=82C  |           |            |         | time=14:22:05
[WARNING]   OBSTACLE   | dist=18cm |           |            |         | time=14:22:10
[INFO]      SPEED      | rpm=1240  | kmh=45    |            |         | time=14:22:15
[EMERGENCY] IMPACT     | ax=3.21g  | ay=0.12g  | az=0.05g   |         | time=14:22:18
[EMERGENCY] LOW_VOLT   | volt=3.8V | curr=2.1A |            |         | time=14:22:20
[WARNING]   LOW_LIGHT  | ldr=120   |           |            |         | time=14:22:25
```

> **Lab demo interpretation:** `[EMERGENCY]` messages represent the system
> notifying emergency services. The PC terminal displays the alert in real time,
> demonstrating the full pipeline from sensor reading to emergency notification.

---

## 7. Sensor and Purpose Map

| Component | Have / Buy | Physical Quantity | Signal / Interface | Purpose in Vehicle | Event / Output | Why It Matters |
|---|---|---|---|---|---|---|
| MPU6050 | Have | Acceleration + angular motion | I²C | Detect sudden movement, braking, tilt, possible impact | `[EMERGENCY] IMPACT` | Provides a real-world event source for interrupt-driven embedded processing |
| LM35 | Have | Temperature | Analog → ADC | Monitor vehicle / motor temperature | `[WARNING] OVER_TEMP` | Demonstrates analog acquisition and threshold decision |
| LDR | Have | Light intensity | Analog → ADC | Detect low-light condition | `[WARNING] LOW_LIGHT` | Demonstrates ADC and automatic control logic |
| IR sensor | Have | Object / edge presence | GPIO digital | Detect nearby obstacle or wheel pulse | `[WARNING] OBSTACLE` | Demonstrates GPIO and edge-triggered events |
| MAX30100 | Have | Pulse / SpO2 | I²C | Optional driver health monitoring | `[WARNING] HEALTH_ALERT` | Demonstrates additional I²C sensor and data processing |
| Potentiometer | Have | Adjustable analog voltage | ADC | Simulate sensor value for controlled fault testing | Controlled fault injection | Demonstrates ADC testing methodology |
| HC-SR04 | Buy | Distance | GPIO + timer | Measure obstacle distance accurately | `[WARNING] OBSTACLE dist=Xcm` | Timer-based measurement (echo pulse duration) |
| INA219 | Buy | Current + bus voltage | I²C | Monitor electrical load and supply condition | `[EMERGENCY] LOW_VOLT` | Adds electrical health monitoring |
| Hall sensor | Buy | Wheel rotation pulses | GPIO + timer | Calculate wheel RPM / speed | `[WARNING] OVERSPEED` | Strong timer + interrupt application |
| USB-to-UART (CP2102/CH340) | Buy | Serial UART | UART | Bridge STM32 USART2 to PC COM port | Emergency alert on PC terminal | Demonstrates serial communication peripheral |

---

## 8. Development Stages

| Stage | Build / Demonstrate | Concepts |
|---|---|---|
| Stage 1 — Core sensing | STM32F401CCUx + LM35 + MPU6050 + LDR + IR | ADC, GPIO, I²C |
| Stage 2 — Event response | Buzzer + LEDs + Hall sensor | Interrupts, timers, GPIO |
| Stage 3 — Logging | RTC (DS3231) + SD card | RTC, SPI, memory |
| Stage 4 — Communication | UART to PC terminal + OLED display | UART, I²C |
| Stage 5 — Testing | Potentiometer and controlled faults; verify UART alert messages on PC terminal | Testing / debugging, performance measurement |

> **Key statement for the lab:** The system demonstrates the complete embedded-system pipeline:
> **Sensor → Signal Acquisition → STM32 Processing → Event Detection →
> Interrupt-Driven Response → Timestamping → SD Logging → UART Emergency Alert to PC → OLED Display**

---

## 9. Purchase List — Updated Budget Estimate

| Item | Qty | Purpose | Priority | Est. Unit (INR) | Est. Total (INR) |
|---|---|---|---|---|---|
| STM32F401CCUx Black Pill board | 1 | Main controller | Essential | 300 | 300 |
| USB-to-UART adapter (CP2102 / CH340) | 1 | PC communication via UART | Essential | 80 | 80 |
| HC-SR04 ultrasonic sensor | 1 | Distance measurement | Essential | 200 | 200 |
| INA219 current / voltage sensor | 1 | Electrical monitoring | Essential | 120 | 120 |
| Hall-effect sensor + magnet | 1 | Wheel pulse / speed measurement | Essential | 100 | 100 |
| 0.96-inch I²C OLED | 1 | Local status display | Essential | 180 | 180 |
| DS3231 RTC module | 1 | Timestamp events | Essential | 200 | 200 |
| Micro-SD module | 1 | Event storage | Recommended | 100 | 100 |
| Micro-SD card | 1 | Store event logs | Recommended | 250 | 250 |
| Buzzer | 1 | Audible alert | Essential | 30 | 30 |
| LEDs + push buttons | 1 | Status indicators and input | Essential | 100 | 100 |
| LM2596 buck converter | 1 | Power regulation | Essential | 120 | 120 |
| Battery / suitable power supply | 1 | Prototype power | Essential | 500 | 500 |
| Breadboard | 1 | Prototype assembly | Essential | 150 | 150 |
| Jumper wires | 1 | Connections | Essential | 120 | 120 |
| Resistors / capacitors / connectors | 1 | Basic interfacing | Essential | 200 | 200 |
| **TOTAL ESTIMATE** | | | | | **~2,450** |

> **Cost saving vs. original plan:** Removed STM32F407 (1700), DC motor (200),
> L298N (125), chassis (700), Bluetooth (was "have", now unused).
> Replaced with STM32F401CCUx (300) + USB-TTL (80). **Net saving: ~2,345 INR.**

---

## 10. Changes from Original Plan — Diff Summary

| Section | Original | Revised |
|---|---|---|
| MCU | STM32F407VET6 (168 MHz, ~1700 INR) | **STM32F401CCUx Black Pill (84 MHz, ~300 INR)** |
| Communication module | Bluetooth (UART → wireless → phone) | **USB-to-UART adapter (UART → wired → PC)** |
| Emergency response | Motor slow/stop + wireless alert | **Structured UART plain-text message to PC only** |
| Motor + L298N | In scope (buy) | **Completely removed** |
| PWM | Used for motor control | **Removed** |
| System Flow Step 6 | Turn buzzer on / slow motor (GPIO + PWM) | **Activate buzzer + LEDs (GPIO only)** |
| System Flow Step 9 | UART + Bluetooth → remote alert | **USART2 → USB-TTL → PC terminal** |
| Dev Stage 2 | Buzzer + LEDs + motor + Hall | **Buzzer + LEDs + Hall (no motor)** |
| Dev Stage 4 | Bluetooth + OLED | **UART to PC + OLED** |
| SD card + RTC | In scope | **Still in scope** |
| HC-SR04 | In scope | **Still in scope** |
