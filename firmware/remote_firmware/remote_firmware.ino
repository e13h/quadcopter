#include <EEPROM.h>
#include <radio.h>
#include <SerLCD.h>

#include "quad_remote.h"  // Header file with pin definitions and setup
#include "transmission.h"

// CONSTANTS
const int THR_POS = 0;
const int YAW_POS = 8;
const int ROLL_POS = 16;
const int PIT_POS = 24;
const int COMP_FILTER_POS = 32;
const int PID_P_POS = 36;
const int PID_I_POS = 40;
const int PID_D_POS = 44;
const int AXIS_MIN = 0;
const int AXIS_MAX = 255;
const int SERIAL_BAUD = 9600;  // Baud rate for serial port

// Global Variables
int throttleRange[2];
int yawRange[2];
int rollRange[2];
int pitchRange[2];

response_pkt pkt;

bool calibrationActive = false;
bool quadcopterArmed = false;
bool tuningActive = false;
int currentTuningParam = -1;

int yaw = 0;
int throttle = 0;
int roll = 0;
int pitch = 0;
float complementaryFilterGain = 0.9;
float pid_p_gain = 0.0;
float pid_i_gain = 0.0;
float pid_d_gain = 0.0;

void btn1_pressed(bool);
void btn2_pressed(bool);
void knob_pressed(bool);
void btn_down_pressed(bool);
void btn_left_pressed(bool);
void btn_up_pressed(bool);
void btn_right_pressed(bool);
void knobs_update();
void set_gimbals();
void check_arm_status();
void print_gimbals();
void calibrateGimbals();
void print_range();
void display_tuning_param();
void begin_tuning(const int, const char*);

void setup() {
  Serial.begin(SERIAL_BAUD);  // Start up serial
  delay(100);
  Serial.println("Remote is online!");
  quad_remote_setup();
  rfBegin(RF_CHANNEL);
  Serial.print("Channel: ");
  Serial.println(RF_CHANNEL);

  btn1_cb = btn1_pressed;
  btn2_cb = btn2_pressed;
  knob1_btn_cb = knob_pressed;
  btn_down_cb = btn_down_pressed;
  btn_left_cb = btn_left_pressed;
  btn_up_cb = btn_up_pressed;
  btn_right_cb = btn_right_pressed;
  knobs_update_cb = knobs_update;
  lcd.setBacklight(0x000000FF);

  eeprom_load(THR_POS, throttleRange);
  eeprom_load(YAW_POS, yawRange);
  eeprom_load(ROLL_POS, rollRange);
  eeprom_load(PIT_POS, pitchRange);
  eeprom_load(COMP_FILTER_POS, complementaryFilterGain);
  eeprom_load(PID_P_POS, pid_p_gain);
  eeprom_load(PID_I_POS, pid_i_gain);
  eeprom_load(PID_D_POS, pid_d_gain);
}

void loop() {
  if (calibrationActive) {
    calibrateGimbals();
  } else {
    if (millis() % 10 == 0) {  // Read gimbal values every 10ms
      set_gimbals();
    }
    if (millis() % 80 == 0) {
      if (recieve_response(&pkt)) {
        quadcopterArmed = pkt.armed;
      }
    }
    check_arm_status();
    if (millis() % 50 == 0) {  // Send a packet every 50ms
      send_packet(throttle, yaw, roll, pitch, quadcopterArmed, complementaryFilterGain);
    }
    if (millis() % 100 == 0 && tuningActive) {
      display_tuning_param();
    }
  }
  if (millis() % 100 == 0) {
    print_gimbals();
  }
}

void calibrateGimbals() {
  print_range();

  int curThrottle = analogRead(PIN_THROTTLE);
  int curYaw = analogRead(PIN_YAW);
  int curRoll = analogRead(PIN_ROLL);
  int curPitch = analogRead(PIN_PITCH);

  // Set default Min range
  throttleRange[0] = curThrottle;
  yawRange[0] = curYaw;
  rollRange[0] = curRoll;
  pitchRange[0] = curPitch;

  // Set default Max range
  throttleRange[1] = curThrottle;
  yawRange[1] = curYaw;
  rollRange[1] = curRoll;
  pitchRange[1] = curPitch;
  delay(500);

  while (calibrationActive) {
    // Get new gimbal values
    curThrottle = analogRead(PIN_THROTTLE);
    curYaw = analogRead(PIN_YAW);
    curRoll = analogRead(PIN_ROLL);
    curPitch = analogRead(PIN_PITCH);

    // Update range if need be
    if (curThrottle < throttleRange[0]) {
      throttleRange[0] = curThrottle;
    }
    if (curThrottle > throttleRange[1]) {
      throttleRange[1] = curThrottle;
    }

    if (curYaw < yawRange[0]) {
      yawRange[0] = curYaw;
    }
    if (curYaw > yawRange[1]) {
      yawRange[1] = curYaw;
    }

    if (curRoll < rollRange[0]) {
      rollRange[0] = curRoll;
    }
    if (curRoll > rollRange[1]) {
      rollRange[1] = curRoll;
    }

    if (curPitch < pitchRange[0]) {
      pitchRange[0] = curPitch;
    }
    if (curPitch > pitchRange[1]) {
      pitchRange[1] = curPitch;
    }
  }

  // Display clalibration close

  // Update roll over values
  eeprom_store(THR_POS, throttleRange);

  eeprom_store(YAW_POS, yawRange);

  eeprom_store(ROLL_POS, rollRange);

  eeprom_store(PIT_POS, pitchRange);

  delay(1000);
  lcd.clear();

  // Debug
  print_range();
  return;
}

void print_range() {
  Serial.print("\nThrottle Min ");
  Serial.print(throttleRange[0]);
  Serial.print("\nThrottle Max ");
  Serial.print(throttleRange[1]);

  Serial.print("\nYaw Min ");
  Serial.print(yawRange[0]);
  Serial.print("\nYaw Max ");
  Serial.print(yawRange[1]);

  Serial.print("\nRoll Min ");
  Serial.print(rollRange[0]);
  Serial.print("\nRoll Max ");
  Serial.print(rollRange[1]);

  Serial.print("\nPitch Min ");
  Serial.print(pitchRange[0]);
  Serial.print("\nPitch Max ");
  Serial.print(pitchRange[1]);
}

void set_gimbals() {
  throttle = analogRead(PIN_THROTTLE);
  throttle =
      map(throttle, throttleRange[0], throttleRange[1], AXIS_MIN, AXIS_MAX);
  if (throttle <= 15) {
    throttle = 0;
  }
  yaw = analogRead(PIN_YAW);
  yaw = map(yaw, yawRange[0], yawRange[1], AXIS_MIN, AXIS_MAX);
  roll = analogRead(PIN_ROLL);
  roll = map(roll, rollRange[0], rollRange[1], AXIS_MIN, AXIS_MAX);
  pitch = analogRead(PIN_PITCH);
  pitch = map(pitch, pitchRange[0], pitchRange[1], AXIS_MIN, AXIS_MAX);
}

void print_gimbals() {
  if (quadcopterArmed) {
    Serial.print("A ");
  } else {
    Serial.print(". ");
  }
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);
}

void btn1_pressed(bool down) {
  if (down && !quadcopterArmed && !tuningActive && !calibrationActive) {
    calibrationActive = !calibrationActive;

    // Print calibrating message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calibrating");
  } else if (down && !quadcopterArmed && calibrationActive) {
    calibrationActive = !calibrationActive;

    // Print default message?
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Closing Calibration");
    delay(30);
  } else if (down && quadcopterArmed) {
    // Print message to disarm the quadcopter?
  }
}

void btn2_pressed(bool down) {
  if (down && quadcopterArmed) {
    quadcopterArmed = false;
    lcd.setBacklight(0x000000FF);
  }
}

void check_arm_status() {
  if (!quadcopterArmed && throttle == 0 && yaw >= 250 && roll >= 250 &&
      pitch >= 250) {
    quadcopterArmed = true;
    lcd.setBacklight(0x00FF0000);
  }
}

void knob_pressed(bool down) {
  if (down && tuningActive) {
    if (currentTuningParam == COMP_FILTER_POS) {
      eeprom_store(COMP_FILTER_POS, complementaryFilterGain);
    } else if (currentTuningParam == PID_P_POS) {
      eeprom_store(PID_P_POS, pid_p_gain);
    } else if (currentTuningParam == PID_I_POS) {
      eeprom_store(PID_I_POS, pid_i_gain);
    } else if (currentTuningParam == PID_D_POS) {
      eeprom_store(PID_D_POS, pid_d_gain);
    } else {
      return;
    }
    tuningActive = false;
    currentTuningParam = -1;
    lcd.clear();
    knob1.setCurrentPos(0);
  }
}

void btn_down_pressed(bool down) {
  if (down && !calibrationActive && !tuningActive) {
    begin_tuning(COMP_FILTER_POS, "Gain:");
  }
}

void btn_left_pressed(bool down) {
  if (down && !calibrationActive && !tuningActive) {
    begin_tuning(PID_P_POS, "P:");
  }
}

void btn_up_pressed(bool down) {
  if (down && !calibrationActive && !tuningActive) {
    begin_tuning(PID_I_POS, "I:");
  }
}

void btn_right_pressed(bool down) {
  if (down && !calibrationActive && !tuningActive) {
    begin_tuning(PID_D_POS, "D:");
  }
}

void begin_tuning(const int tuningParam, const char* msg) {
  if (tuningParam != COMP_FILTER_POS && tuningParam != PID_P_POS &&
      tuningParam != PID_I_POS && tuningParam != PID_D_POS) {
    return;
  }
  tuningActive = true;
  currentTuningParam = tuningParam;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  knob1.setCurrentPos(0);
}

void knobs_update() {
  if (tuningActive) {
    if (currentTuningParam == COMP_FILTER_POS) {
      // Increment/decrement the gain by 0.01, but don't go below 0 or above 1
      complementaryFilterGain = constrain(complementaryFilterGain + (knob1.getCurrentPos() / 100.0), 0.0, 1.0);
    } else if (currentTuningParam == PID_P_POS) {
      pid_p_gain = constrain(pid_p_gain + (knob1.getCurrentPos() / 20.0), 0.0, 4.0);
    } else if (currentTuningParam == PID_I_POS) {
      pid_i_gain = constrain(pid_i_gain + (knob1.getCurrentPos() / 20.0), 0.0, 4.0);
    } else if (currentTuningParam == PID_D_POS) {
      pid_d_gain = constrain(pid_d_gain + (knob1.getCurrentPos() / 20.0), 0.0, 4.0);
    }
    knob1.setCurrentPos(0);
  }
}

void display_tuning_param() {
  lcd.setCursor(0, 1);
  if (currentTuningParam == COMP_FILTER_POS) {
    lcd.print(complementaryFilterGain, 2);
    Serial.print("Gain: ");
    Serial.println(complementaryFilterGain);
  } else if (currentTuningParam == PID_P_POS) {
    lcd.print(pid_p_gain, 2);
    Serial.print("P: ");
    Serial.println(pid_p_gain);
  } else if (currentTuningParam == PID_I_POS) {
    lcd.print(pid_i_gain, 2);
    Serial.print("I: ");
    Serial.println(pid_i_gain);
  } else if (currentTuningParam == PID_D_POS) {
    lcd.print(pid_d_gain, 2);
    Serial.print("D: ");
    Serial.println(pid_d_gain);
  }
}
