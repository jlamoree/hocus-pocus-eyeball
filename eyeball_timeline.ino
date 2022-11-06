/*
 *  The DMX sheild (CTC-DRA-10-R2) library is from https://sourceforge.net/projects/dmxlibraryforar/ and is included using the Conceptinetics header.
 *
 *  Be sure to carefully set your servo positions so the movement is within real-world limits.
 */

#include <Wire.h>
#include <Conceptinetics.h>
#include <Adafruit_PWMServoDriver.h>

#define DMX_DEVICE_ADDRESS 512
#define DMX_DEVICE_CHANNELS 1
DMX_Slave dmx_device(DMX_DEVICE_CHANNELS);
byte dmx_value = 0;

#define SERVO_LR_INDEX 0
#define SERVO_LR_RIGHT 260
#define SERVO_LR_CENTER 290
#define SERVO_LR_LEFT 320
#define SERVO_UD_INDEX 1
#define SERVO_UD_DOWN 245
#define SERVO_UD_CENTER 290
#define SERVO_UD_UP 325
#define SERVO_EL_INDEX 2
#define SERVO_EL_OPEN 300
#define SERVO_EL_CLOSED 330
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

unsigned int servos[] = {SERVO_LR_INDEX, SERVO_UD_INDEX, SERVO_EL_INDEX};
unsigned int servo_position_desired[3];
float servo_position_smoothed[3];
float servo_position_last[3];

bool show_in_progress = false;
bool show_once = true;
unsigned long show_started = 0;
unsigned long playhead = 0;
unsigned long near = 10;

struct movement {
  byte servo;
  unsigned int position;
};
struct keyframe {
  unsigned long playhead;
  movement movements[4]; // Supports up to four movements in one keyframe
  byte movement_count;
};
keyframe keyframes[16]; // Size the keyframe array for the number of keyframes defined later
byte keyframe_count;
byte current_keyframe = 0; // Allows skipping over keyframes that have already passed

/**
* Arduino setup and config
*/
void setup() {
  // The program
  keyframes[0].playhead = 0;
  keyframes[0].movements[0] = {SERVO_EL_INDEX, SERVO_EL_OPEN};
  keyframes[0].movement_count = 1;
  keyframes[1].playhead = 3000;
  keyframes[1].movements[0] = {SERVO_LR_INDEX, SERVO_LR_LEFT};
  keyframes[1].movement_count = 1;
  keyframes[2].playhead = 6000;
  keyframes[2].movements[0] = {SERVO_LR_INDEX, SERVO_LR_RIGHT};
  keyframes[2].movement_count = 1;
  keyframes[3].playhead = 9000;
  keyframes[3].movements[0] = {SERVO_EL_INDEX, SERVO_EL_CLOSED};
  keyframes[3].movements[1] = {SERVO_LR_INDEX, SERVO_LR_CENTER};
  keyframes[3].movement_count = 2;
  keyframes[4].playhead = 9500;
  keyframes[4].movements[0] = {SERVO_EL_INDEX, SERVO_EL_OPEN};
  keyframes[4].movement_count = 1;
  keyframes[5].playhead = 10500;
  keyframes[5].movements[0] = {SERVO_UD_INDEX, SERVO_UD_UP};
  keyframes[5].movement_count = 1;
  keyframes[6].playhead = 11500;
  keyframes[6].movements[0] = {SERVO_UD_INDEX, SERVO_UD_DOWN};
  keyframes[6].movement_count = 1;
  keyframes[7].playhead = 13000;
  keyframes[7].movements[0] = {SERVO_UD_INDEX, SERVO_UD_CENTER};
  keyframes[7].movement_count = 1;
  keyframes[8].playhead = 14500;
  keyframes[8].movements[0] = {SERVO_EL_INDEX, SERVO_EL_CLOSED};
  keyframes[8].movement_count = 1;
  keyframes[9].playhead = 17000;
  keyframes[9].movement_count = 0;

  keyframe_count = 10;

  dmx_device.onReceiveComplete(dmxFrameReceived);
  dmx_device.enable();
  dmx_device.setStartAddress(DMX_DEVICE_ADDRESS);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  resetShow();
  updateServos();
}

/**
* Arduino main loop
*/
void loop() {
  if (show_once) {
    show_once = false;
    startShow();
  }
  if (show_in_progress) {
    evaluatePlayhead();
  }
  updateServos();
}


void evaluatePlayhead() {
  playhead = millis() - show_started;

  for (byte i = current_keyframe; i < keyframe_count; i++) {
    if (playhead > keyframes[i].playhead && playhead < keyframes[i].playhead + near) {
      for (byte j = 0; j < keyframes[i].movement_count; j++) {
        servo_position_desired[keyframes[i].movements[j].servo] = keyframes[i].movements[j].position;
      }
      current_keyframe = i;
      break;
    }
  }
  
  if (current_keyframe >= keyframe_count - 1) {
    show_in_progress = false;
    resetShow();
  }
}

void updateServos() {
  for (auto& index : servos) {
    if (servo_position_desired[index] * 0.01 > 0.5) {
      servo_position_smoothed[index] = (servo_position_desired[index] * 0.01) + (servo_position_last[index] * 0.99);
      servo_position_last[index] = servo_position_smoothed[index];
      pwm.setPWM(index, 0, servo_position_smoothed[index]);
    }
  }
}

void dmxFrameReceived() {
  dmx_value = dmx_device.getChannelValue(1);
  if (dmx_value > 0 && !show_in_progress) {
    startShow();
  }
}

void startShow() {
  show_started = millis();
  show_in_progress = true;
  playhead = 0;
  current_keyframe = 0;
}

void resetShow() {
  servo_position_desired[SERVO_LR_INDEX] = SERVO_LR_CENTER;
  servo_position_last[SERVO_LR_INDEX] = SERVO_LR_CENTER;
  servo_position_desired[SERVO_UD_INDEX] = SERVO_UD_CENTER;
  servo_position_last[SERVO_UD_INDEX] = SERVO_UD_CENTER;
  servo_position_desired[SERVO_EL_INDEX] = SERVO_EL_CLOSED;
  servo_position_last[SERVO_EL_INDEX] = SERVO_EL_CLOSED;
}
