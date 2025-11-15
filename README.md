# Sensor Fusion Project

EECE 5554 Final Project - Multi-Sensor Localization

## Team
- Person 1: Sarthak (Complementary Filter)
- Person 2: Mohit (Extended Kalman Filter)

## What We're Building
Combine GPS + IMU + Camera to track position indoors and outdoors

## Current Status
- [x] Repo created
- [ ] Drivers ready
- [ ] Data collected
- [ ] Algorithms implemented
- [ ] Report written

## Structure
```
sensor_fusion/
├── drivers/          # Code to read sensors
├── data/             # Recorded data will go here
└── scripts/          # Helper scripts
```

## How to Use (Monday)

### 1. Connect Hardware
- GPS → USB port
- IMU → USB port  
- Camera → USB port

### 2. Run Data Collection
```bash
cd sensor_fusion
./collect_data.sh
```

### 3. Walk Path
- Start outside
- Walk into building
- Walk back outside
- Press Ctrl+C when done

Data saved in `data/` folder!

## Next Steps
- Monday: Collect data
- Tuesday-Friday: Build fusion algorithms
- Weekend: Analysis and report
