# Soccer Bot Project

A remotely controlled soccer-playing robot powered by motors and a NodeMCU microcontroller. The bot creates its own WiFi network, allowing a smartphone to connect and control the robot via HTTP requests sent from a custom-designed app.

---

## Features
- **WiFi Connectivity**: The NodeMCU creates its own WiFi network.
- **Remote Control**: The robot can be controlled using an app that sends HTTP requests.
- **Omnidirectional Movement**: The bot can move in desired directions based on user commands.
- **Custom App Integration**: A user-friendly app is designed to facilitate control.

---

## Components Used
1. **NodeMCU**: The microcontroller that handles WiFi communication and processes HTTP requests.
2. **Motors**: Provide movement capability to the bot.
3. **Motor Driver**: Interfaces the NodeMCU with the motors.
4. **Power Supply**: Supplies power to the motors and NodeMCU.
5. **Chassis**: Holds all components together.
6. **Wheels**: Enable movement on the playing field.

---

## Working Principle
1. **WiFi Setup**:
   - The NodeMCU creates its own WiFi hotspot.
   - The smartphone connects to this hotspot.

2. **HTTP Requests**:
   - The app sends HTTP requests (e.g., `http://192.168.4.1/forward`) to the NodeMCU.
   - The NodeMCU processes these requests and sends corresponding signals to the motor driver.

3. **Movement**:
   - The motor driver controls the motors to move the bot in the specified direction.

---

## Installation & Setup
### Hardware Assembly
1. Attach the motors to the chassis.
2. Connect the motor driver to the NodeMCU.
3. Wire the motors to the motor driver.
4. Connect the power supply to the NodeMCU and motor driver.
5. Secure all components to the chassis.

### Software Setup
1. Flash the NodeMCU with the provided firmware:
   - Use the Arduino IDE or a similar platform.
   - Install necessary libraries for NodeMCU (e.g., `ESP8266WiFi`).
2. Load the code to enable WiFi and HTTP handling.

### App Configuration
1. Download or build the control app (source code included).
2. Connect your smartphone to the NodeMCU’s WiFi.
3. Use the app to send movement commands.

---

## Usage
1. Power on the bot.
2. Connect your smartphone to the NodeMCU’s WiFi.
3. Open the app and send movement commands.
4. Enjoy controlling the soccer bot!

---

## HTTP Command Reference
| Command         | URL                     | Action                |
|-----------------|-------------------------|-----------------------|
| Move Forward    | `http://192.168.4.1/forward` | Moves the bot forward |
| Move Backward   | `http://192.168.4.1/backward`| Moves the bot backward|
| Turn Left       | `http://192.168.4.1/left`    | Turns the bot left    |
| Turn Right      | `http://192.168.4.1/right`   | Turns the bot right   |
| Stop            | `http://192.168.4.1/stop`    | Stops the bot         |

---


## Future Improvements
- Add a camera module for live streaming.
- Enable control via a web app for cross-platform compatibility.
- Implement obstacle detection and avoidance.
- Enhance app UI for better user experience.

---

## License
This project is licensed under the MIT License. See the `LICENSE` file for more information.

---

## Acknowledgments
- Inspiration from robotics and IoT projects.
