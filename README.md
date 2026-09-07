# obd2Reader

Arduino + Python setup for reading and clearing OBD-II DTCs on a BMW E87 (2010) using an Elegoo Uno R3 and MCP2515 CAN module.

## 1) Arduino sketch

File: `/home/runner/work/obd2Reader/obd2Reader/obd2_e87_mcp2515.ino`

### Required Arduino libraries
- `mcp2515` (common MCP2515 CAN library that provides `mcp2515.h`)

### Typical wiring (Uno R3 <-> MCP2515)
- D10 -> CS
- D13 -> SCK
- D11 -> SI (MOSI)
- D12 -> SO (MISO)
- D2  -> INT (optional for this sketch)
- 5V  -> VCC
- GND -> GND

OBD-II CAN lines:
- OBD pin 6 -> CAN-H
- OBD pin 14 -> CAN-L
- OBD pin 5 (or 4) -> GND

The sketch:
- Initializes CAN at 500 kbps.
- Reads and prints active DTCs at startup.
- Accepts serial commands:
  - `READ`  -> read current DTCs
  - `CLEAR` -> clear DTCs

Serial output protocol:
- `READY`
- `DTC <code>` lines (example `DTC P0301`)
- `END <count>`
- `CLEAR OK` or `CLEAR FAIL ...`
- `ERR ...` on errors

> Note: many MCP2515 boards use an 8 MHz crystal. If yours is 16 MHz, update bitrate config in the sketch.

## 2) Python client

File: `/home/runner/work/obd2Reader/obd2Reader/tools/obd2_serial_client.py`
Full DTC dictionary: `/home/runner/work/obd2Reader/obd2Reader/tools/dtc_descriptions.py`

### Requirements
- Python 3.8+
- `pyserial`

Install dependency:

```bash
pip install pyserial
```

### Usage

Read codes:

```bash
python /home/runner/work/obd2Reader/obd2Reader/tools/obd2_serial_client.py --port /dev/ttyUSB0
```

Read then optionally clear (with confirmation prompt):

```bash
python /home/runner/work/obd2Reader/obd2Reader/tools/obd2_serial_client.py --port /dev/ttyUSB0 --clear
```

Skip initial read and only allow clear flow:

```bash
python /home/runner/work/obd2Reader/obd2Reader/tools/obd2_serial_client.py --port /dev/ttyUSB0 --no-read --clear
```

The Python script uses a dedicated DTC dictionary file with a comprehensive OBD-II code list, and falls back to a generic category description only if a code is not present in that list.
