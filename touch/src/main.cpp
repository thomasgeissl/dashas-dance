#include "config.h"
#include <esp_now.h>
#include <WiFi.h>
#include "esp_now_midi.h"

// run the print_mac firmware and adjust the mac address
uint8_t broadcastAddress[6] = {0xCC, 0x8D, 0xA2, 0x8B, 0xfe, 0x88};

esp_now_midi ESP_NOW_MIDI;

bool _touchValues[NUMBER_OF_TOUCHES];

void customOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("Custom Callback - Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

void onNoteOn(byte channel, byte note, byte velocity)
{
  Serial.printf("Note On - Channel: %d, Note: %d, Velocity: %d\n", channel, note, velocity);
}

void onNoteOff(byte channel, byte note, byte velocity)
{
  Serial.printf("Note Off - Channel: %d, Note: %d, Velocity: %d\n", channel, note, velocity);
}

void onControlChange(byte channel, byte control, byte value)
{
  Serial.printf("Control Change - Channel: %d, Control: %d, Value: %d\n", channel, control, value);
}

void onProgramChange(byte channel, byte program)
{
  Serial.printf("Program Change - Channel: %d, Program: %d\n", channel, program);
}

void onPitchBend(byte channel, int value)
{
  Serial.printf("Pitch Bend - Channel: %d, Value: %d\n", channel, value);
}
void onAfterTouch(byte channel, byte value)
{
  Serial.printf("After Touch - Channel: %d, Value: %d\n", channel, value);
}
void onPolyAfterTouch(byte channel, byte note, byte value)
{
  Serial.printf("Poly After Touch - Channel: %d, note: %d, Value: %d\n", channel, note, value);
}

void setup()
{
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


  for(auto i = 0; i < NUMBER_OF_TOUCHES; i++){
    _touchValues[i] = false;
  }
}

void loop()
{
  auto time = millis();
  bool touchValues[NUMBER_OF_TOUCHES];
  int rawTouchValues[NUMBER_OF_TOUCHES];
  for (auto i = 0; i < NUMBER_OF_TOUCHES; i++)
  {
    rawTouchValues[i] = touchRead(touchPins[i]);
    touchValues[i] =  rawTouchValues[i] > 40000;
  }

  for (auto i = 0; i < NUMBER_OF_TOUCHES; i++)
  {
    if(_touchValues[i] != touchValues[i]){
      if(touchValues[i]){
        ESP_NOW_MIDI.sendNoteOn(60+i, (int)(rawTouchValues[i], 0, 400000, 0, 127), 11);
        Serial.print("touched: "); Serial.println(i);
      }else{
        ESP_NOW_MIDI.sendNoteOff(60+i, 0, 11);
        Serial.print("untouched: "); Serial.println(i);
      }
    }
    _touchValues[i] = touchValues[i];
  }
}