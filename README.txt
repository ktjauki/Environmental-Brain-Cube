Environmental Brain Cube (EBC)

Overview
The Environmental Brain Cube (EBC) is a compact ESP32-based embedded system designed to monitor, analyze, and evaluate environmental conditions in real time.

The system integrates multiple sensors to measure temperature, humidity, pressure, motion, light, and power, while logging data for post-processing in MATLAB. It is designed to determine environment classification and long-term suitability, with applications in space exploration and autonomous systems.

The project demonstrates a complete pipeline:
data acquisition → processing → evaluation → prediction → decision-making

Full Report:
See Project Report folder or GitHub repository.


Key Features
- Multi-sensor environmental monitoring
- Real-time OLED display + serial output
- SD card logging (CSV + summaries)
- Threshold-based warning system (LEDs + buzzer)
- Environment classification:
  - Normal
  - Warm-Dry
  - Warm-Humid
  - Cold
- MATLAB-based:
  - Data visualization
  - Trend analysis
  - Long-term prediction
  - Suitability evaluation


Hardware System

Core Components
- ESP32 WROVER-E (main controller)
- BME280 – air temperature, humidity, pressure
- DS18B20 – ground temperature
- MPU6050 – acceleration & gyroscope
- Photoresistor – light sensing
- OLED (SSD1306) – display
- SD Card Module – data logging
- 5V Solar Panel – power input sensing
- 9V Battery – power source


System Architecture

1. Data Acquisition
   Sensors collect environmental data continuously

2. Processing
   ESP32 converts raw data into usable values
   Invalid data is filtered

3. Evaluation
   Values compared to predefined thresholds
   Environment matched to closest profile

4. Output
   OLED + Serial Monitor display
   SD card logging
   LED/Buzzer alerts

5. Post-Processing
   MATLAB used for analysis, visualization, and predictions


Repository Structure

Environmental-Brain-Cube/
|
|-- Arduino Code/          # ESP32 programs (FREE run, MODES classification, SUITABILITY analysis)
|-- MATLAB Scripts/        # MATLAB scripts for data processing, plotting, and prediction
|-- MATLAB Outputs/        # Generated graphs, figures, and analysis results
|-- CSV Logged Data Files/ # Raw sensor data collected from test runs (CSV format)
|-- CAD Files/             # 3D models and design files for the enclosure
|-- Media/                 # Images of setup + Google Drive link to videos (videos stored externally due to large size)
|-- Project Report/        # Full engineering report (PDF)
|-- README.txt


Experimental Setup

Experiments were conducted using a controlled simulation box to replicate different environments:

- Normal – ambient conditions
- Warm-Dry – heated air, low humidity
- Warm-Humid – heated water, high humidity
- Cold – cooled environment

All conditions were stabilized before testing to ensure consistent data collection.


Data & Analysis

Data Collection
- Air temperature
- Ground temperature
- Humidity
- Pressure
- Light level
- Motion (acceleration)
- Battery voltage
- Solar voltage

MATLAB Processing
- Time-series plots
- Stability vs drift analysis
- Trend-based predictions
- Time-to-limit estimation
- Environment classification validation


Results Summary
- System successfully classified environments based on sensor data
- Stable environments showed consistent readings over time
- Trend analysis identified potential long-term instability
- Power analysis indicated battery depletion trends over extended use


Key Engineering Concepts
- Embedded systems design (ESP32)
- Sensor integration (I2C, analog, digital)
- Data logging and processing
- Environmental modeling
- Trend-based prediction systems
- Power monitoring (battery vs solar)
- Experimental validation


Applications
- Space exploration systems
- Planetary environmental monitoring
- Autonomous robotics
- Remote sensing platforms


Limitations
- Simplified simulation environment
- No direct current sensing (power estimated from voltage)
- Sensor accuracy constraints
- Prediction models are trend-based (not physics-based)


Future Work
- Add current sensing for accurate power analysis
- Implement wireless data transmission
- Improve prediction models (nonlinear / ML)
- Test in real-world outdoor environments
- Integrate into mobile robotic systems


Author
Kevin Jauki
Mechanical Engineering
George Mason University
