# Nano Sentinel — Embedded Telemetry Stack

**Three-word descriptor:** Tiny Sensor Telemetry

Nano Sentinel is a microcontroller telemetry system that combines **C-based firmware** and **Python host tools** to demonstrate robust two-way communication between an embedded device and a computer. It implements a **binary framed serial protocol** with CRC validation, allowing structured commands, acknowledgments, and sensor data streaming.

---

## 📂 Project Structure

nano-sensor-v1/
├── fw/
│   ├── include/        # headers (frame.h, cmd_parser.h, etc.)
│   ├── src/            # firmware .c files (crc16.c, frame.c, cmd_parser.c, proto.c, main_avr.c, ...)
│   ├── boards/         # uart_avr.c + adc_avr.c hardware drivers
│   ├── build-avr/      # compiled outputs (ignored by git)
│   └── tools/          # host-side Python tools
│       ├── cli.py
│       ├── stream_monitor.py
│       ├── status_log.py
│       ├── wire_diag.py
│       └── sniff.py
├── tests/              # Ceedling unit tests
├── .gitignore
├── LICENSE
└── README.md

---

## ⚙️ System Architecture

 ┌───────────────────────────────┐
 │         Python Tools          │
 │  ├── cli.py (send commands)   │
 │  ├── stream_monitor.py (plot) │
 │  ├── status_log.py (CSV log)  │
 │  └── wire_diag.py (debug)     │
 └───────────────┬───────────────┘
                 │  Serial (115200 8N1)
 ┌───────────────┴───────────────┐
 │        AVR Firmware (C)       │
 │  ├── frame.c / frame.h        │ ← CRC + framing
 │  ├── cmd_parser.c / .h        │ ← command handling
 │  ├── proto.c / .h             │ ← decode / reply logic
 │  ├── uart_avr.c / .h          │ ← UART HAL
 │  ├── adc_avr.c / .h           │ ← sensor input (A0)
 │  └── main_avr.c               │ ← main loop & sampling
 └───────────────────────────────┘

---

## 🧠 Key Features

- **Binary protocol:**  
  `AA 55 | ver | type | len | payload | CRC16`

- **CRC-16/CCITT-FALSE validation** ensures data integrity.

- **Commands supported:**
  - `PING` → reply with `"PONG"`
  - `START` / `STOP` → control sampling
  - `SET_RATE [u16 ms]` → configure sample rate
  - `GET_STATUS` → reply with `[on, rate_hi, rate_lo]`

- **Sensor streaming:**  
  When active, the firmware periodically sends sensor samples as `FT_DATA` frames.

- **Cross-platform host tools:**  
  - `cli.py` — interactive control  
  - `stream_monitor.py` — live Matplotlib plot  
  - `status_log.py` — CSV logger  
  - `wire_diag.py` — raw frame inspector  

---

## 🧩 Build Instructions

### 🛠 Firmware (AVR / Arduino Nano or Uno)

Requirements:
- avr-gcc  
- avrdude  

Build and flash:

mkdir -p fw/build-avr
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -std=gnu11 -Wall -Wextra   -Ifw/include -Ifw/boards   fw/src/*.c fw/boards/*.c   -o fw/build-avr/nano_sentinel.elf
avr-objcopy -O ihex -R .eeprom fw/build-avr/nano_sentinel.elf fw/build-avr/nano_sentinel.hex

# Replace the serial path as needed
avrdude -c arduino -p m328p -P /dev/cu.usbserial-XXXX -b 115200 -D   -U flash:w:fw/build-avr/nano_sentinel.hex:i

---

### 🐍 Python Tools

Requirements:

python3 -m venv .venv
source .venv/bin/activate
pip install pyserial matplotlib

Usage examples:

python fw/tools/cli.py /dev/cu.usbserial-XXXX --ping
python fw/tools/cli.py /dev/cu.usbserial-XXXX --get-status
python fw/tools/cli.py /dev/cu.usbserial-XXXX --set-rate 200
python fw/tools/cli.py /dev/cu.usbserial-XXXX --start
python fw/tools/stream_monitor.py /dev/cu.usbserial-XXXX

---

## 🧪 Testing (Ceedling)

cd fw/tests
ceedling test:all

Unit tests validate CRC logic, frame parsing, and command replies.

---

## 💡 Learning Highlights

- Implemented a **custom serial protocol** with sync, header, payload, and CRC validation.
- Created **embedded command parsing logic** to manage system state.
- Integrated **Python-based telemetry tools** for data visualization and logging.
- Learned practical debugging across **firmware ↔ host serial boundaries**, including UART timing and DTR/RTS behavior.

---

## 🧰 Tech Stack

| Layer | Technologies |
|-------|---------------|
| Firmware | C, AVR-GCC, Atmega328P |
| Host Tools | Python, PySerial, Matplotlib |
| Testing | Ceedling, Unity |
| Protocol | Binary Framing + CRC16 |

---

## 📜 License

MIT © 2025 Reeve Fernandes

---

## 🧾 Project Summary

Nano Sentinel demonstrates how low-level firmware and high-level tools can cooperate to form a complete data pipeline — from hardware sensing to visual analysis — all within a compact and educational project.
