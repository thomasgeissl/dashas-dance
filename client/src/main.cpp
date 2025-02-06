#include "config.h"
#include <esp_now.h>
#include <WiFi.h>
#include "esp_now_midi.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>
#include <Wire.h>


// run the print_mac firmware and adjust the mac address
uint8_t broadcastAddress[6] = { 0xCC, 0x8D, 0xA2, 0x8B, 0xfe, 0x88 };

esp_now_midi ESP_NOW_MIDI;

Adafruit_MPU6050 _mpu;
Adafruit_VL53L0X _lox = Adafruit_VL53L0X();

int touchValues[NUMBER_OF_TOUCHES];


bool _mpu6050Connected = false;
bool _vl53LoxConnected = false;
bool _bh1750Connected = false;

unsigned long _bh1750LastTime = 0;
unsigned long _mpu6050LastTime = 0;
unsigned long _vl53l0xLastTime = 0;

BH1750 _bh1750;


int _soloCC = -1;

VL53L0X_RangingMeasurementData_t _distanceMeasure;
sensors_event_t a, g, _temperature;




void customOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("Custom Callback - Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

void onNoteOn(byte channel, byte note, byte velocity) {
  Serial.printf("Note On - Channel: %d, Note: %d, Velocity: %d\n", channel, note, velocity);
}

void onNoteOff(byte channel, byte note, byte velocity) {
  Serial.printf("Note Off - Channel: %d, Note: %d, Velocity: %d\n", channel, note, velocity);
}

void onControlChange(byte channel, byte control, byte value) {
  Serial.printf("Control Change - Channel: %d, Control: %d, Value: %d\n", channel, control, value);
}

void onProgramChange(byte channel, byte program) {
  Serial.printf("Program Change - Channel: %d, Program: %d\n", channel, program);
}

void onPitchBend(byte channel, int value) {
  Serial.printf("Pitch Bend - Channel: %d, Value: %d\n", channel, value);
}
void onAfterTouch(byte channel, byte value) {
  Serial.printf("After Touch - Channel: %d, Value: %d\n", channel, value);
}
void onPolyAfterTouch(byte channel, byte note, byte value) {
  Serial.printf("Poly After Touch - Channel: %d, note: %d, Value: %d\n", channel, note, value);
}

bool shouldSendControlChangeMessage(int controller) {
  if (_soloCC <= 0) {
    return true;
  }
  if (_soloCC == controller) {
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  ESP_NOW_MIDI.setup(broadcastAddress, customOnDataSent);
  // ESP_NOW_MIDI.setup(broadcastAddress); //or get rid of the custom send function and use the default one

  // all of these midi handlers are optional, depends on the usecase, very often you just wanna send data and not receive
  // e.g. this can be used for calibration, or maybe you wanna connect an amp via i2s and render some sound
  ESP_NOW_MIDI.setHandleNoteOn(onNoteOn);
  ESP_NOW_MIDI.setHandleNoteOff(onNoteOff);
  ESP_NOW_MIDI.setHandleControlChange(onControlChange);
  ESP_NOW_MIDI.setHandleProgramChange(onProgramChange);
  ESP_NOW_MIDI.setHandlePitchBend(onPitchBend);
  ESP_NOW_MIDI.setHandleAfterTouchChannel(onAfterTouch);
  ESP_NOW_MIDI.setHandleAfterTouchPoly(onPolyAfterTouch);


  Serial.println("setting up sensors");
  Serial.print("setting up mpu 6050 ... ");
  _mpu6050Connected = _mpu.begin();
  if (_mpu6050Connected) {
    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.println("done");
  } else {
    Serial.println("error");
  }
  delay(1000);

  Serial.print("setting up vl53l0x ... ");
  _vl53LoxConnected = _lox.begin();
  if (_vl53LoxConnected) {
    Serial.println("done");
  } else {
    Serial.println("error");
  }
  delay(1000);


  Serial.print("setting up bh1750 ... ");
  _bh1750Connected = _bh1750.begin();
  if (_bh1750Connected) {
    Serial.println("done");
  } else {
    Serial.println("error");
  }
  delay(1000);
}

void loop() {
  auto time = millis();
  for (auto i = 0; i < NUMBER_OF_TOUCHES; i++) {
    touchValues[i] = touchRead(touchPins[i]);
  }

  // for(auto i = 0; i < NUMBER_OF_TOUCHES; i++){
  //   Serial.print(touchValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");

  if (_bh1750Connected && (time > _bh1750LastTime + RATE_BH1750)) {
    float lux = _bh1750.readLightLevel();
    lux = min((int)(lux), 1000);
    ESP_NOW_MIDI.sendControlChange(CC_BH1730, map(lux, 0, 1000, 0, 127), 1);
    _bh1750LastTime = time;
  }

  if (_mpu6050Connected && (time > _mpu6050LastTime + RATE_BH1750)) {
    _mpu.getEvent(&a, &g, &_temperature);
    // esp_err_t result =
    if (shouldSendControlChangeMessage(CC_MPU6050_ACCELERATION_X)) {
      ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ACCELERATION_X, map(a.acceleration.x + a.acceleration.y + a.acceleration.z, -30, 30, 0, 127), 1);
    }
    _mpu6050LastTime = time;



    // if (shouldSendControlChangeMessage(CC_MPU6050_ACCELERATION_X)) {
    //   ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ACCELERATION_X, map(a.acceleration.x, -10, 10, 0, 127), 1);
    // }
    // if (shouldSendControlChangeMessage(CC_MPU6050_ACCELERATION_Y)) {
    //   ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ACCELERATION_Y, map(a.acceleration.y, -10, 10, 0, 127), 1);
    // }
    // if (shouldSendControlChangeMessage(CC_MPU6050_ACCELERATION_Z)) {
    //   ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ACCELERATION_Z, map(a.acceleration.z, -10, 10, 0, 127), 1);
    // }
    // if (shouldSendControlChangeMessage(CC_MPU6050_ORIENTATION_X)) {
    //   ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ORIENTATION_X, map(g.orientation.x, -5000, 5000, 0, 127), 1);
    // }
    // if (shouldSendControlChangeMessage(CC_MPU6050_ORIENTATION_Y)) {
    //   ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ORIENTATION_Y, map(g.orientation.y, -5000, 5000, 0, 127), 1);
    // }
    // if (shouldSendControlChangeMessage(CC_MPU6050_ORIENTATION_Z)) {
    //   ESP_NOW_MIDI.sendControlChange(CC_MPU6050_ORIENTATION_Z, map(g.orientation.z, -5000, 5000, 0, 127), 1);
    // }
  }

  if (_vl53LoxConnected && (time > _vl53l0xLastTime + RATE_BH1750)) {
    _lox.rangingTest(&_distanceMeasure, false);  // pass in 'true' to get debug data printout!

    if (_distanceMeasure.RangeStatus != 4) {
      auto distance = _distanceMeasure.RangeMilliMeter;
      if (distance < 1000) {
        ESP_NOW_MIDI.sendControlChange(CC_VL53L0X, map(distance, 0, 1000, 127, 0), 1);
        _vl53l0xLastTime = time;
      }
    } else {
      // Serial.println(" out of range ");
    }
  }
}