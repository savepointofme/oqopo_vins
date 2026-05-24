# tools/convert — Data Conversion Tools

Convert raw recorded data (ROS bags, ArduPilot logs) into the EuRoC/ASL format that the OpenVINS ROS-free runner expects.

---

## Scripts

### `extract_bag.py`
**Purpose:** Extract camera frames and IMU data from a ROS bag into EuRoC/ASL format.

**Input:** A ROS bag file (`.bag`) containing image and IMU topics.

**Output:** `mav0/cam0/data.csv`, `mav0/cam0/data/*.png`, `mav0/imu0/data.csv`

**Example:**
```bash
python tools/convert/extract_bag.py \
    --bag two.bag \
    --cam-topic /cam0/image_raw \
    --imu-topic /imu0/data \
    --output 20260509_fly2/mav0
```

**Pitfalls:**
- Requires ROS environment or `rosbag` Python package.
- Timestamp precision: camera timestamps must be in nanoseconds for OpenVINS. Verify `data.csv` format.
- Large bags take significant disk space when extracted (~14 GB for `two.bag`).

---

### `extract_gps.sh`
**Purpose:** Extract GPS CSV from an ArduPilot `.bin` flight log or from a `.log` text file.

**Input:** ArduPilot binary log (`.bin`) or text log with GPS message lines.

**Output:** `GPS.csv` with columns: `TimeUS, Lat, Lng, Alt, ...`

**Example:**
```bash
bash tools/convert/extract_gps.sh \
    /path/to/ardulog.bin \
    output_GPS.csv
```

**Pitfalls:**
- The canonical raw GPS files are already in `D:\vscode_dir\20260509_fly{N}_gps\GPS.csv`.
  Do not re-extract unless the raw log files have been replaced.
- `TimeUS` is microseconds since ArduPilot boot — not an absolute timestamp.
  Use `align/align_gps_traj_timestamps.py` to convert to camera time.

---

## Canonical Inputs

The extracted datasets already exist. You do not need to re-extract unless starting from a new recording.

| Flight | Source bag | Extracted dataset |
|--------|-----------|-------------------|
| fly2 | `two.bag` (~7 GB) | `20260509_fly2/mav0/` (~14 GB) |
| fly4 | `four.bag` (~10 GB) | `20260509_fly4/mav0/` (~19 GB) |
| fly1 | unknown bag | `20260509_fly1/mav0/` (~25 GB) |
| fly3 | unknown bag | `20260509_fly3/mav0/` (~16 GB) |

---

## Derived Results Are Not Ground Truth

Files produced by these scripts (images, IMU CSV) are sensor recordings.
GPS files produced by `extract_gps.sh` are raw ArduPilot logs, not aligned to camera time.
Always run `tools/align/align_gps_traj_timestamps.py` after GPS extraction.
