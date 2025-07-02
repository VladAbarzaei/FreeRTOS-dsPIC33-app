# Servo Hatch Controller with FreeRTOS

A real-time embedded application for controlling a servo-driven hatch based on ambient temperature or analog input. The system runs on a dsPIC33FJ128MC802 microcontroller and uses FreeRTOS for task management and synchronization.

## Features

- Dual control modes: automatic (temperature-based) and manual (voltage-based)
- Real-time multitasking with FreeRTOS
- User interaction via UART, LED indicators, LCD display, and pushbutton
- PWM control for precise servo positioning
- Temperature sensing with DS18B20
- ADC input for manual mode

## Components

- **dsPIC33FJ128MC802**
- **FreeRTOS**
- **DS18B20** temperature sensor
- **ADC input** on RB3
- **LCD** with 4 lines
- **UART** interface for command input
- **LEDs** for status indication

## UART Commands

- `m` – Show current mode
- `c` – Toggle between auto/manual mode
- `t` – Print current temperature
- `h` – Show help menu

## FreeRTOS Tasks

| Task Name         | Function                                  | Frequency |
|-------------------|-------------------------------------------|-----------|
| `vAppStatusTask`  | Toggle status LED                         | 200 ms    |
| `vTempTask`       | Read temperature and apply PWM (auto)     | 2000 ms   |
| `vAdcTask`        | Read ADC and apply PWM (manual)           | 100 ms    |
| `vSerialMenuTask` | Handle UART commands and display info     | 100 ms    |

## Build & Run

1. Open the project in MPLAB X or your preferred IDE.
2. Compile and flash the firmware to the dsPIC33FJ128MC802 board.
3. Use a UART terminal (e.g., Tera Term) to send commands and view output.

## Notes

- The servo control signal is generated using PWM1H3 (RB10).
- The application uses semaphores to ensure mutual exclusivity between control modes.
- Perspective and behavior are validated using oscilloscope captures at different input values.

