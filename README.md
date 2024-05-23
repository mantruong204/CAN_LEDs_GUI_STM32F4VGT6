# LEDs CONTROL via CAN & Qt PYTHON GUI
## Hardware
- STM32F407VGT6 Discovery Board (x3)
- CAN Transceiver: CAN TJA1050
- LEDs
- Dimmer
- Switch
- Personal Computer
## Software diagram
![ảnh](https://github.com/mantruong204/CAN_LEDs_GUI_STM32F4VGT6/assets/155959855/51d2ec3b-a867-4a49-af23-691053b59ffb)
## Functionalities: 1 Master and 2 Slave, GUI (Qt Python)
- Slave 1 (S1): Dimmer, DIP Switch, Binary Switch
- Slave 2 (S2): RGB LED, 2 Steer signal LED
- Master: take LED cmds from S1 → S2, GUI interface
  - Dimmer: control color of RGB LED
  - DIP switch: Steer LEDs - right, left, backward
  - Binary Switch: ON/OFF RGB LED
- GUI: view current LEDs status and CAN frame, pick RGB Color, choose steer LEDs mode, send to Master
## Qt Python GUI
![ảnh](https://github.com/mantruong204/CAN_LEDs_GUI_STM32F4VGT6/assets/155959855/7f633985-3c6e-4ca5-a76a-7168ed78d9aa)
### UART Zone
![ảnh](https://github.com/mantruong204/CAN_LEDs_GUI_STM32F4VGT6/assets/155959855/73742d2f-c30a-40e7-bcbe-267de61a6b25)
### Slave 1 Zone
![ảnh](https://github.com/mantruong204/CAN_LEDs_GUI_STM32F4VGT6/assets/155959855/1a5a9157-df01-4c28-8d1c-d57eacdf85d5)
### Slave 2 and StatusBar Zone
![ảnh](https://github.com/mantruong204/CAN_LEDs_GUI_STM32F4VGT6/assets/155959855/bb33e67d-1cfb-4153-903b-72f7943b9664)
## Hardware
![ảnh](https://github.com/mantruong204/CAN_LEDs_GUI_STM32F4VGT6/assets/155959855/291e5968-f726-4be7-a5fa-9cf4b63b38a6)
