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
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN* odrives[] = {&odrv0};

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
bool isLogging = false;

bool lastStart = HIGH;
bool lastStop = HIGH;
unsigned long debounce = 150;
unsigned long lastStartTime = 0, lastStopTime = 0;

/* Position Limiting */
const float LINEAR_RANGE_MM = 30.0f;           // 30mm total linear travel (one-sided from start)
const float DRUM_RADIUS_MM = 3.0f;             // Drum radius in mm
const float DRUM_DIAMETER_MM = DRUM_RADIUS_MM * 2.0f;  // 6mm diameter
const float DRUM_CIRCUMFERENCE_MM = PI * DRUM_DIAMETER_MM;  // ~18.85mm

// Calculate how much linear motion per motor rotation
const float MM_PER_ROTATION = DRUM_CIRCUMFERENCE_MM;  // 1 rotation = full circumference of linear motion

// Calculate rotations needed for 30mm displacement from starting position
const float MAX_POSITION_ROTATIONS = LINEAR_RANGE_MM / MM_PER_ROTATION;  // ~1.592 rotations for full 30mm travel

float startPosition = 0.0f;  // Starting position at one end of rail
float centerPosition = 0.0f; // Center of oscillation (15mm from start)
unsigned long startTime = 0;  // Time when motor started (for phase calculation)

// Frequency sweep parameters
const float FREQ_START = 0.5f;   // Starting frequency in Hz
const float FREQ_END = 6.0f;    // Ending frequency in Hz
const float SWEEP_DURATION = 60.0f;  // Duration of sweep in seconds (adjust as needed)
const float RAMP_TIME = 0.5f;    // Time to ramp from 0 to -5mm

// Data logging variables
int32_t lastLC1_raw = 0;
int32_t lastLC2_raw = 0;

// Load cell calibration constants 
const float LC1_ZERO = -24.61f;
const float LC1_NCOUNT = 0.000094f; 
const float LC2_ZERO = -1705.74;
const float LC2_NCOUNT = 0.000096f; 

// Function to convert raw load cell reading to Newtons
float loadCellToNewtons(int32_t rawValue, float zero, float nCount) {
  return nCount * (rawValue - zero);
}

void setup() {
  while(!Serial);
  Serial.begin(115200);

  Serial.println("Initializing Load Cells...");
  // Start both I2C buses
  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
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

  // Register callbacks for heartbeat and encoder feedback
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true);
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
    while (true);
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

    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrive in closed loop control. Configuring position control mode...");
  
  // Set controller mode to position control
  odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  delay(100);
  
  // Pump events to process any responses
  for (int i = 0; i < 20; i++) {
    pumpEvents(can_intf);
    delay(10);
  }

  Serial.println("ODrive ready. Press START button to begin frequency sweep.");
  Serial.print("Sweep: ");
  Serial.print(FREQ_START);
  Serial.print(" Hz to ");
  Serial.print(FREQ_END);
  Serial.print(" Hz over ");
  Serial.print(SWEEP_DURATION);
  Serial.println(" seconds");
}

void loop() {
  pumpEvents(can_intf);
  unsigned long now = millis();

  // Read current button states
  bool startState = digitalRead(BUTTON_START);
  bool stopState = digitalRead(BUTTON_STOP);

  // START button - trigger on press (HIGH to LOW transition)
  if (startState == LOW && lastStart == HIGH && (now - lastStartTime) > debounce) {
    Serial.println("START button pressed!");
    
    // Ensure we're in closed loop control
    Serial.println("Entering closed loop control...");
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    
    // Wait for state to change and pump events
    unsigned long timeout = millis() + 1000;
    while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL && millis() < timeout) {
      pumpEvents(can_intf);
      delay(10);
    }
    
    if (odrv0_user_data.last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      Serial.println("Closed loop control active!");
    } else {
      Serial.println("ERROR: Failed to enter closed loop control!");
      lastStartTime = now;
      lastStart = startState;
      return;
    }
    
    // Get current position with manual request
    Serial.println("Reading starting position...");
    Get_Encoder_Estimates_msg_t feedback;
    if (odrv0.request(feedback, 500)) {
      // Set reference so CURRENT position is the starting point
      startPosition = feedback.Pos_Estimate;
      centerPosition = startPosition + (MAX_POSITION_ROTATIONS / 2.0f);
      odrv0_user_data.last_feedback = feedback;
      odrv0_user_data.received_feedback = true;
      
      // Print CSV header
      Serial.println("===== DATA START =====");
      Serial.println("Timestamp_ms,Position_mm,Velocity_rot_s,Commanded_Position_rot,LoadCell1_N,LoadCell2_N,CommandedTorque_Nm,MeasuredTorque_Nm,Frequency_Hz,Direction");
      
      isLogging = true;
      runMotor = true;
      startTime = now;  // Record when motor started
      
      Serial.print("Start position: ");
      Serial.print(startPosition, 4);
      Serial.print(" rot, Center position: ");
      Serial.print(centerPosition, 4);
      Serial.println(" rot");
      
      Serial.println("Starting frequency sweep NOW!");
    } else {
      Serial.println("ERROR: Could not read starting position!");
    }
    
    lastStartTime = now;
  }
  
  // STOP button - trigger on press (HIGH to LOW transition)
  if (stopState == LOW && lastStop == HIGH && (now - lastStopTime) > debounce) {
    runMotor = false;
    isLogging = false;
    Serial.println("===== DATA END =====");
    Serial.println("Stopping - holding current position");
    
    // Hold at current position
    if (odrv0_user_data.received_feedback) {
      odrv0.setPosition(odrv0_user_data.last_feedback.Pos_Estimate);
    }
    
    delay(50);
    pumpEvents(can_intf);
    lastStopTime = now;
  }

  // Save button states for next iteration
  lastStart = startState;
  lastStop = stopState;

  // ------------- MOTOR COMMAND & DATA LOGGING -------------
  if (runMotor) {
    // Manually request feedback every loop iteration
    Get_Encoder_Estimates_msg_t feedback;
    bool haveFeedback = odrv0.request(feedback, 100); // 100ms timeout
    
    float currentPos = 0.0f;
    float currentVel = 0.0f;
    
    if (haveFeedback) {
      currentPos = feedback.Pos_Estimate;
      currentVel = feedback.Vel_Estimate;
      // Update our stored feedback
      odrv0_user_data.last_feedback = feedback;
      odrv0_user_data.received_feedback = true;
    }
    
    // Calculate time since START button pressed in seconds
    float t = (now - startTime) * 0.001f;
    
    float commandedPosition;
    float currentFrequency = FREQ_START;  // Default for ramp phase

    if (t < RAMP_TIME) {
      // Ramp phase: smoothly go from 0mm to -5mm (tensioning)
      float rampProgress = t / RAMP_TIME;  // 0 to 1
      float rampPositionMM = -5.0f * rampProgress;  // 0 to -5mm
      commandedPosition = startPosition + (rampPositionMM / MM_PER_ROTATION);
    } else {
      // Frequency sweep phase
      float tSweep = t - RAMP_TIME;  // Time since sweep started
      
      // Check if sweep is complete
      if (tSweep > SWEEP_DURATION) {
        // Hold at center position after sweep completes
        runMotor = false;
        isLogging = false;
        Serial.println("===== SWEEP COMPLETE =====");
        Serial.println("Holding at center position");
        commandedPosition = startPosition + (-12.5f / MM_PER_ROTATION);  // Center of -5 to -20mm range
      } else {
      
      // Linear frequency sweep from FREQ_START to FREQ_END
      currentFrequency = FREQ_START + (FREQ_END - FREQ_START) * (tSweep / SWEEP_DURATION);
      
      // Calculate instantaneous phase by integrating frequency over time
      // phase(t) = 2π * ∫f(t)dt = 2π * [f_start*t + (f_end-f_start)*t²/(2*T)]
      float phase = TWO_PI * (FREQ_START * tSweep + 
                              (FREQ_END - FREQ_START) * tSweep * tSweep / (2.0f * SWEEP_DURATION));
      
      // Oscillation parameters (same as before)
      const float MIN_POSITION_MM = -20.0f;
      const float MAX_POSITION_MM = -5.0f;
      const float OSCILLATION_CENTER_MM = (MAX_POSITION_MM + MIN_POSITION_MM) / 2.0f;  
      const float OSCILLATION_RANGE_MM = MAX_POSITION_MM - MIN_POSITION_MM;  
      
      float amplitudeRot = (OSCILLATION_RANGE_MM / 2.0f) / MM_PER_ROTATION;
      float centerOffsetRot = OSCILLATION_CENTER_MM / MM_PER_ROTATION;
      
              commandedPosition = startPosition + centerOffsetRot + amplitudeRot * cosf(phase);
      }
    }
    
    // Send position command
    odrv0.setPosition(commandedPosition);
    
    // ------------- DATA LOGGING -------------
    if (isLogging && haveFeedback) {
      static unsigned long lastDataLog = 0;
      if (now - lastDataLog > 10) {  // Log every 10ms
        lastDataLog = now;
        
        float posFromStart = (currentPos - startPosition) * MM_PER_ROTATION;

        // Request torque data
        Get_Torques_msg_t torques;
        float commandedTorque = 0.0f;
        float measuredTorque = 0.0f;
        if (odrv0.request(torques, 100)) {
          commandedTorque = torques.Torque_Target;
          measuredTorque = torques.Torque_Estimate;
        }
        
        // Convert raw load cell values to Newtons
        float lc1_newtons = loadCellToNewtons(lastLC1_raw, LC1_ZERO, LC1_NCOUNT);
        float lc2_newtons = loadCellToNewtons(lastLC2_raw, LC2_ZERO, LC2_NCOUNT);
        
        // Determine direction
        static float lastPos = 0;
        static bool firstLog = true;
        String direction = "";
        if (!firstLog) {
          if (posFromStart > lastPos + 0.1) {
            direction = "MOVING_FORWARD";
          } else if (posFromStart < lastPos - 0.1) {
            direction = "MOVING_BACKWARD";
          } else {
            direction = "STATIONARY";
          }
        } else {
          direction = "STARTING";
          firstLog = false;
        }
        lastPos = posFromStart;
        
        // Log data in CSV format with frequency
        Serial.print(now);
        Serial.print(",");
        Serial.print(posFromStart, 4);
        Serial.print(",");
        Serial.print(currentVel, 4);
        Serial.print(",");
        Serial.print((commandedPosition - startPosition) * MM_PER_ROTATION, 4);
        Serial.print(",");
        Serial.print(lc1_newtons, 6);
        Serial.print(",");
        Serial.print(lc2_newtons, 6);
        Serial.print(",");
        Serial.print(commandedTorque, 6);
        Serial.print(",");
        Serial.print(measuredTorque, 6);
        Serial.print(",");
        Serial.print(currentFrequency, 4);  // Current frequency
        Serial.print(",");
        Serial.println(direction);
      }
    }
  }

  // ------------- LOAD CELLS -------------
  if (nau1.available()) {
    lastLC1_raw = nau1.read();
  }

  if (nau2.available()) {
    lastLC2_raw = nau2.read();
  }
}