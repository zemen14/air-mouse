# 🖱️ AirMouse — Tilt-Controlled Bluetooth Mouse

**Wave your hand, move the cursor.** An ESP32-C3 reads an ICM-42670
accelerometer and turns tilt into mouse movement over Bluetooth LE.
It pairs with any computer or phone as a standard wireless mouse —
no drivers, no dongle, no app.

- **Tilt** the board to move the cursor (small tilt = fine control,
  big tilt = fast travel)
- **Hold a tilt** and the cursor accelerates — like a real mouse's
  pointer acceleration curve
- **Press the BOOT button** to click

## How it works

- The ICM-42670 IMU is read over I2C at 400 kHz (SDA GPIO 7,
  SCL GPIO 8), 100 times per second
- Acceleration tiers: tilt past the small threshold moves 5 px per
  report, past the large threshold 15 px — multiplied up to 3x the
  longer a tilt is held
- Movement is sent as standard BLE HID mouse reports
  (`esp_hidd_dev_input_set`), so any BLE host treats it as a mouse

## Hardware

- ESP32-C3 board
- ICM-42670-P accelerometer/gyro IMU (I2C `0x68`)
- BOOT button (GPIO 9) wired as the click input

## Build and flash

Requires ESP-IDF 6.1 with Bluetooth enabled.

```bash
idf.py set-target esp32c3
idf.py build flash monitor
```

Then pair with **AirMouse** from your computer's Bluetooth settings.

## Files

- `main/air_mouse_main.c` — IMU driver, tilt-to-cursor mapping with
  acceleration, click handling, and the BLE HID mouse device
- `main/esp_hid_gap.c/.h` — Bluetooth GAP setup helpers
- `extras/tilt_test.cpp` — the standalone IMU tilt detector this
  project grew out of (logs UP/DOWN/LEFT/RIGHT)

## Credit

The BLE HID device scaffolding is adapted from Espressif's ESP-IDF
`esp_hid_device` example. The IMU integration, tilt mapping, and
acceleration behavior are the original work in this repo.
