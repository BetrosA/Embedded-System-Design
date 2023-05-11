# Embedded-System-Design

# Display library esp32c3 idf

This is a simple example of using the Display library on the ESP32-C3 IDF. The code demonstrates how to initialize and control the display module, including writing text and drawing shapes.

## Requirements

To run this example, you need the following hardware:

- ESP32-C3 development board
- Display module lCD1604

## Setup

1. Connect the display module to the ESP32-C3 development board following the pinout specifications.
2. Connect the ESP32-C3 development board to your computer using a USB cable.
3. Install the ESP-IDF and the required toolchain on your computer.
4. Open a terminal and navigate to the project directory.
5. Build the project by running the command `idf.py build`.
6. Flash the firmware to the ESP32-C3 development board by running the command `idf.py -p PORT flash` (replace PORT with the name of the serial port).
7. Monitor the output by running the command `idf.py -p PORT monitor` (replace PORT with the name of the serial port).

## Usage

After flashing the firmware to the ESP32-C3 development board and starting the monitor, you should see the display module showing a blue screen with a white circle and some text.

The main loop of the program continuously updates the display, drawing a red circle that moves from left to right and bouncing off the edges of the screen. It also updates the text on the screen with the current time.

## Customization

You can modify the code to change the behavior of the display module. For example, you can change the colors, the size, and the position of the shapes, or add new elements to the screen.

You can also use the Display library to create your own projects that involve displaying information on a screen, such as weather stations, IoT dashboards, or user interfaces for embedded devices.


