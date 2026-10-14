# Micromouse

[![Firmware](https://github.com/RakibulIslam1/micromouse/actions/workflows/build.yml/badge.svg)](https://github.com/RakibulIslam1/micromouse/actions/workflows/build.yml)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-253858)
![License](https://img.shields.io/badge/license-MIT-green)

**ESP32-S3 firmware for an encoder driven micromouse.**

Micromouse brings together magnetic wheel odometry, infrared wall sensing, flood-fill exploration, persistent maze storage, and selectable path runs in one ESP32-S3 project. A small OLED and three buttons provide the operating interface, so normal setup and maze runs work without a laptop.

This repository contains firmware for the hardware profile described below. It is distributed under the MIT license and can be built and modified with PlatformIO.

## Release profile

| Property | Community firmware |
| --- | --- |
| Controller | ESP32-S3, Arduino framework |
| Maze | Fixed 13 × 13 cells |
| Cell pitch | Fixed 180 mm |
| Starting position | Bottom-left cell, facing into the maze |
| Default goal | Center cell `(6, 6)` |
| Optional goal | Single cell or a 2 × 2 region set through the menu |
| Wheel sensing | Two AS5600 encoders, one on each I²C bus |
| Navigation sensing | Left, front, and right IR channels; five channels sampled |
| User interface | 128 × 64 SSD1306 OLED and three buttons |
| Storage | 1,024-byte EEPROM emulation region |
| Host connection | Native USB serial at 115200 |

Turns use relative encoder feedback. Straight movement retains wall tracking and departure-heading feedback. This release has no heading correction step after each cell, rear wall alignment maneuver, maze-size selector, starting-corner selector, or interactive encoder magnet alignment screen. Mechanical encoder alignment is part of assembling the robot. Wi-Fi telemetry and advanced serial tuning are outside this release.

## Hardware

Use an ESP32-S3 board exposing the GPIOs below, two independently driven DC motors, an L293D bridge, two AS5600 magnetic encoders, five analog IR receivers with switched emitters, an SSD1306 display, three buttons, and a buzzer. The bridge enable inputs are tied high; PWM is applied to its four direction inputs.

| Connection | GPIO / bus |
| --- | --- |
| Left motor forward / reverse | 16 / 35 |
| Right motor forward / reverse | 15 / 38 |
| IR receiver L / LD / F / RD / R | 6 / 5 / 4 / 11 / 10 |
| IR emitter L / LD / F / RD / R | 7 / 8 / 8 / 7 / 1 |
| OLED and left encoder SDA / SCL | 3 / 2, `Wire` |
| Right encoder SDA / SCL | 13 / 12, `Wire1` |
| Left / right / center button | 33 / 37 / 18 |
| Buzzer | 9 |

The display address is `0x3C`; both encoders use `0x36`, which is why the encoders sit on separate buses. Both buses run at 400 kHz. Buttons connect their GPIO to ground when pressed and use internal pull-ups. IR emitter GPIOs 7 and 8 are shared by channel pairs; preserve that wiring when reproducing the sensor board.

The firmware expects positive wheel distance when the robot moves forward. The right encoder has the opposite mechanical orientation, and its count direction is inverted in software. Defaults assume a 35.5 mm wheel diameter and a 63 mm wheelbase. Measure the assembled chassis before changing these settings.

Supply the motors through their driver and use a common ground with the controller. Keep signals reaching the ESP32 within its GPIO voltage limits. Confirm motor direction with the wheels lifted before attempting a maze run.

## Build and flash

Install PlatformIO Core or the PlatformIO extension for VS Code. The configuration pins Espressif32 `6.13.0`, Adafruit SSD1306 `2.5.17`, and Adafruit GFX `1.12.6`; other required Arduino libraries are resolved by PlatformIO.

```bash
git clone https://github.com/RakibulIslam1/micromouse.git
cd micromouse
python -m pip install platformio==6.2.0
python -m platformio run -e micromouse
python -m platformio run -e micromouse -t upload
python -m platformio device monitor -b 115200
```

If multiple devices are attached, pass `--upload-port` to the upload command and `--port` to the monitor command. The board profile is `esp32-s3-devkitc-1`, with flash constrained to 4 MB and native USB enabled. The generated image appears at `.pio/build/micromouse/firmware.bin`.

The physical robot must expose the listed pins even if its board name differs from the build profile. This repository contains firmware and a wiring reference; it does not include PCB fabrication files, a chassis model, or a complete bill of materials.

## Operating the mouse

At power-on, the firmware initializes sensors and encoders, checks encoder response, and opens the home screen. Encoder warnings identify a hardware problem that should be corrected before motion. The home screen displays wheel distances, drive speed, and sensor bars.

| Home control | Short press | Hold longer than 300 ms |
| --- | --- | --- |
| Left | Left-hand wall rule | Right-hand wall rule |
| Center | Stored route at moderate speed | Flood-fill search |
| Right | Main menu | Stored route speed selection |

In the main menu, short left/right presses browse entries. Hold right to select; hold left to return. The menu provides Settings, Analog Read, Dist. Read, Motor Check, Wall Follow, Debug Toggle, Flood Fill, Run Path, Clear Maze, and Fast Run.

Search explores toward the configured goal, returns to the starting area, checks the candidate route, and saves the map. It stops after saving. Select a run separately once the robot is positioned correctly. Stored-route modes require confirmed open edges along the route. Clear the saved maze when the physical track changes.

Place the robot in the bottom-left starting cell facing north into the track. Search and hand-rule modes include a 50 mm initial entry movement; position the robot to account for that offset. Stored-route runs begin at the starting cell's center and expect the same north-facing orientation. There is no automatic backing maneuver to establish either position.

Press either side button to stop during motion. Fast Run performs a segmented outward run, returns, and then uses the higher-speed arc mode; begin with normal Run Path until the chassis is tuned.

## Setup parameters

Settings contains eighteen controls: drive speed, motor balance, turn speed, movement and turn braking, wall Kp/Kd, wheelbase, wall setpoint, straight Kp, turn deceleration, wheel diameter, three wall thresholds, and the goal mode and coordinates.

In the settings list, short left/right presses browse and hold right opens an item. In the editor, short left increases and short right decreases the value; hold right saves, and hold left discards. Saved values are applied immediately and restored after reboot.

| Default when EEPROM has no valid value | Value |
| --- | --- |
| Drive / turn PWM | 150 / 100 |
| Movement / turn brake | 50 / 40 ms |
| Wall Kp / Kd | 20 / 50 |
| Straight Kp | 30 |
| Turn deceleration zone | 30 degrees |
| Side / front wall thresholds | 1000 / 220 ADC units |
| Wall tracking setpoint | 205 ADC units |

Use Analog Read to measure each sensor against the actual track walls. A wall is detected when its raw reading falls below its threshold. Thresholds depend on receiver circuits, ambient light, emitter placement, and wall reflectivity. The Dist. Read screen uses inherited conversion coefficients and should be treated as an estimate until measured against the installed sensors; navigation uses raw wall readings.

Tune wheel geometry and motor balance before increasing speed. Motor Check starts at zero PWM; short presses change its target and holding right applies it. Holding left stops and returns. Keep the wheels lifted for this check. Displayed tuning values are chassis settings, not measured performance guarantees.

## Technical overview

The motion layer integrates absolute wheel-angle samples into signed travel and differential heading. It handles encoder rollover, filters implausible samples, limits steering authority, and uses bounded movement and turn loops. Infrared observations update a compact directional wall map, and flood-fill distances guide the next step toward the active goal.

Stored routes can be replayed one cell at a time or with consecutive straight cells grouped into a segment. The fast mode also offers differential arc turns. Maze data and route data are validated when loaded from EEPROM. Motor output stops on detected motion failures or user cancellation.

USB provides a small idle-screen console: `?` shows help, `p` prints status, `z` resets distance counters, and `!` stops the motors. These commands are serviced on the home screen; use the side buttons during a run. Debug Toggle enables additional serial diagnostics.

## Validation

The community firmware is compiled for the ESP32-S3 target before publication. Native checks exercise relative turn selection, flood-fill distances against independent breadth-first searches, route verification, EEPROM round trips and corrupted records, and failed-movement position handling. GitHub Actions builds the firmware on pushes and pull requests.

Software checks do not establish physical turn accuracy, sensor calibration, battery behavior, or competition times. This release still needs validation on the assembled robot and its intended track. No competition ranking or timing result is claimed.

## License and contributions

The project source is available under the [MIT license](LICENSE). PlatformIO downloads third-party libraries separately; those libraries retain their own licenses. Build caches and third-party reference projects are not included in this repository.

For a useful issue or pull request, include the board, motor driver, wheel dimensions, relevant settings, observed behavior, and the build output. Keep proposed changes compatible with the published hardware profile and fixed maze geometry.
