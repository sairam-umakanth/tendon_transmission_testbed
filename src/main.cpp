
#include <Arduino.h>
#include "ODriveCAN.h"
#include <Wire.h>
#include <Adafruit_NAU7802.h>

/* ------------------------ Load Cells ------------------------ */
// Create two NAU7802 objects
Adafruit_NAU7802 nau1;
Adafruit_NAU7802 nau2;

void setupNAU(Adafruit_NAU7802 &nau) {
  nau.setLDO(NAU7802_3V0);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_320SPS);

  nau.calibrate(NAU7802_CALMOD_INTERNAL);
  nau.calibrate(NAU7802_CALMOD_OFFSET);

  // Prime ADC
  for (int i = 0; i < 10; i++) {
    while (!nau.available());
    nau.read();
  }
}

/* ------------------------- ODrive CAN ------------------------- */
// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000
// ODrive node_id for odrv0
#define ODRV0_NODE_ID 5

#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

void onCanMessage(const CanMsg& msg); // forward declaration
struct ODriveStatus; // hack to prevent teensy compile error

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

/* ---------------------------- Setup ---------------------------- */

/* Button Configuration */
#define BUTTON_START 2
#define BUTTON_STOP 3
bool runMotor = false;

bool lastStart = HIGH;
bool lastStop = HIGH;
unsigned long debounce = 150;
unsigned long lastStartTime = 0, lastStopTime = 0;




void setup() {
  while(!Serial); // wait for serial port to open
  Serial.begin(115200);

  Serial.println("Initializing Load Cells...");
  // Start both I2C buses
  Wire.begin();         // Default I2C (pins 18=SDA0, 19=SCL0 on Teensy 4.0)
  Wire.setClock(400000);

  Wire1.begin();        // Second I2C (pins 17=SDA1, 16=SCL1 on Teensy 4.0)
  Wire1.setClock(400000);

  // Initialize first load cell on Wire
  if (!nau1.begin(&Wire)) {
    Serial.println("NAU7802 #1 not found!");
    while (1);
  }

  // Initialize second load cell on Wire1
  if (!nau2.begin(&Wire1)) {
    Serial.println("NAU7802 #2 not found!");
    while (1);
  }

  // Setup for both
  setupNAU(nau1);
  setupNAU(nau2);

  Serial.println("Both load cells ready");

  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1000)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrive ready. Press START button to begin motion.");
}
void loop() {
  pumpEvents(can_intf);
  unsigned long now = millis();

  // Read current button states
  bool startState = digitalRead(BUTTON_START);
  bool stopState = digitalRead(BUTTON_STOP);

  if (startState == LOW && lastStart == HIGH && (now - lastStartTime) > debounce) {
    runMotor = true;
    Serial.println("Starting torque control oscillation");
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(50);
    pumpEvents(can_intf);
    lastStartTime = now;
  }
  if (stopState == LOW && lastStop == HIGH && (now - lastStopTime) > debounce) {
    runMotor = false;
    Serial.println("Stopping - setting torque to 0");
    odrv0.setTorque(0.0f);
    delay(50);
    pumpEvents(can_intf);
    lastStopTime = now;
  }

  lastStart = startState;
  lastStop  = stopState;

   // ------------- MOTOR COMMAND -------------
  if (runMotor) {
    float t = now * 0.001f;
    float T = 2.0f;
    float phase = t * (TWO_PI / T);

    // Using torque command
    float commandedTorque = 0.5f * sinf(phase);

    // Clamp torque to ±0.5 Nm for safety
    const float MAX_TORQUE = 0.5f;
    if (commandedTorque > MAX_TORQUE) {
      commandedTorque = MAX_TORQUE;
    } else if (commandedTorque < -MAX_TORQUE) {
      commandedTorque = -MAX_TORQUE;
    }

    odrv0.setTorque(commandedTorque);
  }

  // ------------- FEEDBACK STREAMING -------------
  // Print commanded torque, position, and velocity
  static unsigned long lastPrintTime = 0;
  if (now - lastPrintTime >= 50) {  // Print every 50ms (20Hz)
    lastPrintTime = now;
    
    // Calculate current commanded torque
    float t = now * 0.001f;
    float T = 2.0f;
    float phase = t * (TWO_PI / T);
    float commandedTorque = runMotor ? (0.5f * sinf(phase)) : 0.0f;
    
    // Print commanded torque
    Serial.print("Cmd_Torque:");
    Serial.print(commandedTorque, 4);
    Serial.print(",");
    
    // Print position and velocity
    if (odrv0_user_data.received_feedback) {
      Serial.print("Pos:");
      Serial.print(odrv0_user_data.last_feedback.Pos_Estimate, 4);
      Serial.print(",");
      Serial.print("Vel:");
      Serial.println(odrv0_user_data.last_feedback.Vel_Estimate, 4);
    } else {
      Serial.println();
    }
  }
  

  // ------------- LOAD CELLS STREAMING -------------
  /*
  if (nau1.available()) {
    int32_t v1 = nau1.read();
    Serial.print("LC1:");
    Serial.println(v1);
  }

  if (nau2.available()) {
    int32_t v2 = nau2.read();
    Serial.print("LC2:");
    Serial.println(v2);
  }
    */
}