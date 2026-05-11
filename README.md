# Codebase for Tendon Transmission Testbed
The Tendon Transmission Testbed allows for comparison between different tendon transmission methods in the same environment, accounting for variables like wrap angle, bend radius, and added length. This software controls the actuation and data collection for this testbed. 

## How to Use:
**Wiring:** 
* **Load Cells:** Connect LC1 (SDA: pin 18, SCL: pin 19) and LC2 (SDA: pin 17, SCL: pin 16) to the Teensy on I2C Bus 0 and 1 respectively. 
* **Linear Encoder-Quadrature Receiver:** Connect pins 4, 8, 3, and 7 of the linear encoder DB9 connector to pins 32 (A), 31 (A&#772;), 9 (B), and 10 (B&#772;) of the quadrature receiver. Ensure 120&ohm; resistors are placed between A-A&#772; and B-B&#772;. Z and Z&#772; can be left hanging.
* **Linear Encoder Quadrature Receiver-Teensy:** Connect A0 and B0 to pins 7 and 9 respectively on the Teensy. NOTE: the quadrature receiver outputs 5V, which is greater than what the Teensy can accept. As such, A0 and B0 should be reduced (e.g., via voltage divider) before connecting to the Teensy. The fault pins can be left hanging. 

**Actuation:**
1. Connect wiring as specified above.
2. Flash 'main.cpp' onto Teensy
3. Open Serial Monitor
4. Tension tendon via the vented screw until a tension of 25.15 &plusmn; 0.15 N (or modified required starting tension value) is reached for 5 seconds (or modified holding time). 
5. Press the 'Start' button once the minimum tendon tension value is reached for the given period of time to begin the oscillation. The default is a linear ramp from 0 to 6 mm before an oscillation between 6 mm and 18 mm. 
6. Press the 'Stop' button to stop the test. 
7. Copy the values printed in the Serial Monitor into an Excel or Google sheet and format the CSV appropriately. 

## How It's Made:
**Hardware Used:** 
* Microcontroller: Teensy 4.1
* Motor Controller: ODrive Pro
* Rotary Encoder: Same Sky AMT212B-V 
* Linear Encoder: ATO ATO-IMLE-LMD
* Load Cell: Strain Gauge Load Cell (20 kg)

**Software:** 
* Teensy -- programmed with C++ 
* Teensy -- Computer: Serial 
* Teensy -- Load Cells/NAU7802: I2C 
* Teensy -- Linear Encoder: RS-422
* ODrive -- Rotary Encoder: RS-485 



