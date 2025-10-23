# COMPSYS-704-part2 Group 19
Teresa Zhang and Jack Wang

## What it measures
- **Accelerometer (mg):** `ACC_Value.x`, `ACC_Value.y`, `ACC_Value.z`
- **Magnetometer (mg):** `MAG_Value.x`, `MAG_Value.y`, `MAG_Value.z`
- **Computed values:**
  - **Heading (0–360°):** magnetometer-based yaw (relative heading is sent over BLE)
  - **Step Count:** simple Z-axis threshold with hysteresis


## Run with **STM32CubeIDE**
1. **Import the project**
   - *File ▸ Import ▸ Existing Projects into Workspace* → select the folder containing `main.c`.

2. **Build**
   - Choose Debug/Release and click **Build** (hammer icon).

3. **Flash**
   - Connect device to the **computer** and **STM32CubeIDE**.
   - Click **Run** (play) to program and start.

4. **View on phone**
   - Install **ST BLE Sensor** (iOS/Android).
   - Power the board, scan, and connect to the device.
   - Open live charts to see **Accelerometer**, **Magnetometer**, **Steps**, and **Heading**.

---

## Notes
- **Quick calibration:** after startup, slowly rotate the board in different orientations for a few seconds so min/max calibration settles; heading stabilizes afterwards.
- **Update rate:** loop runs about **100 ms** (adjustable in timer init if needed).
- **Units:** accelerometer & magnetometer values are in **mg** (milli-g / milli-gauss).