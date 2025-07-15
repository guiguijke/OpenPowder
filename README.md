# OpenPowder

![OpenPowder Logo](https://via.placeholder.com/150?text=OpenPowder) <!-- Replace with actual logo if available -->

Welcome to **OpenPowder** -- an open-source project for precise and efficient powder dosing using Arduino! Whether you're in pharmaceuticals, chemistry, or DIY crafting, OpenPowder automates the dispensing of fine powders with high accuracy and speed. Say goodbye to manual scoops and hello to reliable, repeatable dosing.

This project leverages a stepper motor for controlled dispensing, a high-precision load cell for real-time weight monitoring, and an intuitive LCD interface. It's designed to handle small quantities (up to 3g) with minimal overshoot, making it ideal for sensitive applications.

## Features

- **Adaptive Dosing Modes**: Fast initial dosing followed by progressively slower bursts for precision without sacrificing speed.

- **Calibration System**: Ensures grams-per-step accuracy through three successive measurements, optimizing for your specific setup.

- **User-Friendly Controls**: Buttons for starting cycles, taring, and auto mode; long-press for calibration.

- **LCD Display**: Real-time feedback on status, speed, setpoint, and current weight, with a flashing prompt for calibration if needed.

- **Open-Source**: Fully customizable code for Arduino enthusiasts.

## Hardware Requirements

To build and run OpenPowder, you'll need:

- **Arduino Due**: The core microcontroller (chosen for its speed and compatibility with high-precision libraries).

- **Stepper Motor Shield**: A compatible shield (e.g., based on FastAccelStepper library) to drive the stepper motor for powder dispensing.

- **LCD Screen**: An I2C-compatible LCD (e.g., 16x2 with address 0x27) for displaying status and measurements.

- **Load Cell (30g capacity)**: A high-resolution load cell for accurate weight sensing (e.g., HX711-compatible strain gauge).

- **HX711 ADC Module**: Amplifies and converts the load cell signal for precise digital readings.

- Additional components: Potentiometers for setpoint adjustment (connected to A0 and A1), buttons/switches for controls (pins 8-10), and wiring for the stepper (pins 2, 4, 6).

Assemble the hardware as per standard Arduino wiring diagrams for these components. Ensure the load cell is mounted securely to minimize vibrations.

## Installation

1\. **Clone the Repository**:

   ```bash

   git clone https://github.com/yourusername/OpenPowder.git

   cd OpenPowder

   ```

2\. **Set Up Development Environment**:

   - Use **VSCode with PlatformIO (PIO)** for seamless Arduino development:

     - Install VSCode from [code.visualstudio.com](https://code.visualstudio.com/).

     - Add the PlatformIO extension via the Extensions marketplace (search for "PlatformIO IDE").

     - Open the project folder in VSCode -- PIO will auto-detect the Arduino project.

     - Configure `platformio.ini` if needed (example below):

       ```ini

       [env:due]

       platform = atmelsam

       board = due

       framework = arduino

       ```

     - Build and upload: Use the PIO toolbar (bottom of VSCode) to build (`✓` icon) and upload (`→` icon) to your Arduino Due.

3\. **Libraries**:

   - The code depends on: `LiquidCrystal_I2C`, `HX711_ADC`, `FastAccelStepper`.

   - PIO will handle installation automatically during build, or install via PIO's library manager:

     ```bash

     pio lib install "LiquidCrystal_I2C" "HX711_ADC" "FastAccelStepper"

     ```

## Usage

1\. **Power On**: Connect your Arduino Due to power/USB. The LCD will show "Setup Done" after initialization.

2\. **Setpoint Adjustment**: Use potentiometers on A0 (integer part) and A1 (decimal part) to set the target weight (0-3g).

3\. **Controls**:

   - **Start Button (Pin 9)**: Press to begin dosing.

   - **Tare Button (Pin 10)**: Short press to tare (zero) the scale; long press (1s) to start calibration.

   - **Auto Mode Switch (Pin 8)**: Enable for automatic dosing when weight drops below -0.5g.

4\. **Dosing Process**:

   - The system tares, then doses in phases: Fast (up to 80-95% based on setpoint), Medium Slow, Slow, and Very Slow bursts.

   - Real-time LCD updates show status (e.g., "Fast", "Slow"), speed percentage (00-99), target ("T:"), and current weight ("W:").

   - If calibration isn't done, "Calib Need" flashes in waiting mode.

5\. **Monitoring**: Serial output (115200 baud) logs weights, states, and debug info for troubleshooting.

## Calibration

Calibration is critical for achieving the fastest and most precise powder dosing. It calculates `grams_per_step` -- the amount of powder dispensed per stepper motor step -- tailored to your hardware (e.g., motor resolution, powder density, dispensing mechanism).

### Why Calibrate?

- **Precision**: Accounts for mechanical variations, ensuring bursts don't overshoot.

- **Speed**: Accurate grams-per-step allows aggressive fast dosing without excess, followed by fine-tuned slow phases.

- **Repeatability**: Averages three 400-step measurements to minimize noise and errors.

### How to Calibrate

1\. **Trigger**: Long-press the Tare button (Pin 10) for 1 second.

2\. **Process**:

   - The system tares the scale.

   - Executes three bursts of 400 steps each at max speed, stabilizing after each.

   - Measures weight change per burst and averages the results to update `grams_per_step`.

3\. **LCD Feedback**: Shows "Tare Calib" during taring, then "Calib" while running.

4\. **Completion**: Serial logs each burst and the final average (e.g., "Calibration average grams/step = 0.000123"). The system resets to waiting.

5\. **Reminder**: If not calibrated, the LCD flashes "Calib Need" in waiting mode. Dosing uses a default (0.0001g/step), but accuracy may suffer.

Run calibration whenever hardware changes (e.g., new powder type or motor adjustments) for optimal performance.

## Weighing Logic

OpenPowder's dosing algorithm balances speed and precision, ideal for powders where even 0.005g matters:

1\. **Taring**: Zeros the scale for an accurate starting point.

2\. **Fast Phase**: Dispenses up to 80-95% of target (adaptive: higher % for larger setpoints >0.5g, halved speed for <0.5g to prevent overshoot). Uses estimated steps based on calibrated grams-per-step.

3\. **Stabilization**: Brief pause (300-500ms) after each phase to account for powder settling and measurement lag.

4\. **Medium Slow Bursts**: 5-20 steps at 1/3 max speed, targeting 97% of setpoint.

5\. **Slow Bursts**: 3-10 steps at 1/10 max speed, to 99.5%.

6\. **Very Slow Bursts**: 1-5 steps at 1/20 max speed, until within 0.005g tolerance.

7\. **Adaptive Intelligence**: Bursts are calculated dynamically (80% of remaining grams), with min/max limits to avoid tiny/infinite loops. Retries after stabilization if under target.

This multi-stage approach ensures rapid bulk dispensing followed by precise fine-tuning, minimizing time while avoiding overshoot -- especially critical for small doses (e.g., 0.3g) or larger ones (e.g., 2.7g).

## Contributing

We'd love your contributions! Fork the repo, make changes, and submit a pull request. Ideas for improving the algorithm, UI, or hardware integrations are welcome. Join our community on Discord to discuss: [https://discord.gg/vSHgKag9AD](https://discord.gg/vSHgKag9AD).

## License

MIT License -- feel free to use, modify, and distribute.

Happy dosing with OpenPowder! Share your builds and feedback on GitHub or our [Discord server](https://discord.gg/vSHgKag9AD). 🚀