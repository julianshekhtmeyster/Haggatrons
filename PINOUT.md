# Robot pinout (W11 + TB6612FNG + REV 2m Distance Sensor)

Every robot is wired the same way. `firmware/robot/robot.ino` uses exactly these pins.

```
                    W11 ESP32-S3                               TB6612FNG
               ┌──────────────────┐                      ┌────────────────┐
               │ IO1  ────────────┼──────────────────────┤ AIN1           │
               │ IO2  ────────────┼──────────────────────┤ AIN2       AO1 ├──┐ left N20
               │ IO3  ────────────┼──────────────────────┤ PWMA       AO2 ├──┘
               │ IO6  ────────────┼──────────────────────┤ PWMB           │
               │ IO7  ────────────┼──────────────────────┤ BIN1       BO1 ├──┐ right N20
               │ IO8  ────────────┼──────────────────────┤ BIN2       BO2 ├──┘
               │ IO9  ────────────┼──────────────────────┤ STBY           │
               │ 3V3  ────────────┼──────────┬───────────┤ VCC            │
               │ GND  ────────────┼────────┬─┼───────────┤ GND            │
               │ BAT+ ───(LiPo +)─┼────────┼─┼───────────┤ VM             │
               │                  │        │ │           └────────────────┘
               │ IO5 (SDA) ───────┼──┐     │ │           REV 2m Distance Sensor
               │ IO4 (SCL) ───────┼─┐│     │ │           ┌────────────────┐
               │  (onboard QMI8658│ │└─────┼─┼───────────┤ white  SDA     │
               │   IMU on IO5/IO4)│ └──────┼─┼───────────┤ blue   SCL     │
               └──────────────────┘        │ └───────────┤ red    3.3 V   │
                                           └─────────────┤ black  GND     │
                                                         └────────────────┘
```

| W11 pin | Connects to | Purpose |
| --- | --- | --- |
| IO1 | TB6612 AIN1 | Left motor direction |
| IO2 | TB6612 AIN2 | Left motor direction |
| IO3 | TB6612 PWMA | Left motor speed (20 kHz PWM) |
| IO6 | TB6612 PWMB | Right motor speed (20 kHz PWM) |
| IO7 | TB6612 BIN1 | Right motor direction |
| IO8 | TB6612 BIN2 | Right motor direction |
| IO9 | TB6612 STBY | Driver enable; firmware holds it LOW except during an approved move |
| IO5 | REV white (SDA) | Shared I²C bus with the onboard IMU |
| IO4 | REV blue (SCL) | Shared I²C bus with the onboard IMU |
| 3V3 | TB6612 VCC, REV red | Logic power |
| GND | TB6612 GND, REV black | Common ground (battery, driver, sensor) |
| BAT+ (LiPo) | TB6612 VM | Motor power |
| — | AO1/AO2, BO1/BO2 | Left N20, right N20 |

## Changes from the first wiring

Four wires move and one is added:

- **BIN1 IO4 → IO7, BIN2 IO5 → IO8.** IO4/IO5 are the onboard QMI8658 IMU's I²C lines, so motor signals there would corrupt the IMU, and IMU traffic would toggle the right motor's direction.
- **REV sensor IO7/IO8 → IO5 (white) / IO4 (blue).** The ESP32-S3 has two I²C controllers: the camera uses one, and the IMU the other. The REV sensor (address 0x29) therefore shares the IMU's bus; the IMU's address is different, so they don't clash.
- **New: STBY → IO9.** This gives the firmware a hardware kill: the driver is disabled at boot and between moves. Leaving STBY unconnected disables the driver entirely.

All motor pins sit on the W11's IO1–IO9 header. None is a camera pin or USB pin, and GPIO21 (possible SD card select) is avoided. The TB6612's internal pull-downs keep the motors off while the ESP32 boots.

## Before driving on the floor

1. Wheels off the floor. Arm the fleet in the app and jog each robot. If a wheel spins backwards, set `motor_flags` in `config/fleet.json` (1 = invert left, 2 = invert right, 4 = swap sides) and re-provision.
2. Turn left with the jog button (⟲). If the reported yaw is negative, set `yaw_sign` to -1 (or change `yaw_axis` if the board is mounted on its side) and re-provision.
3. Check each N20's stall current against the TB6612's rating at your LiPo voltage (1.2 A continuous per channel).
4. Measure `forward_mps_at_full_speed` (drive forward 1 s, measure, divide by the speed fraction) and update `config/fleet.json`.
