#include <Arduino.h>
#include <Control-S.h>

#include "PinDefinitionsAndMore.h"
#define DECODE_SAMSUNG
#define NO_LED_FEEDBACK_CODE
#include <IRremote.hpp>  // include the library

// Power details
#define TV_POWER_PIN 5
#define STR_DE845_POWER_PIN 6
bool TV_power_on[2]; //index 0 = past, index 1 = now
bool STR_DE845_power_on[2]; //index 0 = past, index 1 = now

// TV Remote Details
#define TV_Remote_Proto SAMSUNG
const uint16_t TV_Remote_Address = 0x707;

// Sony STR-DE845 Details
const uint16_t Control_S_Address = 16;

// Mapping of TV Remote commands to Control-S commands
const uint8_t mapped_commands = 3;
const uint16_t command_mapping[mapped_commands][3] = {
  //2nd dim=0: TV remote command ID, Control-S command, Control-S repeats
  { 0x2, 21, 3},  // Power on/off
  { 0x7, 18, 1},  // Volume up
  { 0xB, 19, 1 }   // Volume down
};
#define Control_S_power_map_id 0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("### STARTING UP ###");

  Serial.println("### Setting up IR receiver ###");
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
  IrReceiver.begin(IR_RECEIVE_PIN);

  Serial.println("### Setting up Sony Control-S Sender ###");
  Serial.print(" Control_S_sendPin=");
  Serial.println(Control_S_sendPin);
  //pinModeFast(Control_S_sendPin, OUTPUT);
  pinMode(Control_S_sendPin,OUTPUT);  
  //digitalWriteFast(sendPin, LOW); //Default should be 0V
	digitalWrite(Control_S_sendPin,LOW);

  Serial.println("### Setting up TV and STR-DE845 power status pins ###");
  Serial.print(" TV_POWER_PIN=");
  Serial.println(TV_POWER_PIN);
  pinMode(TV_POWER_PIN,INPUT);
  TV_power_on[0]=digitalRead(TV_POWER_PIN);
  TV_power_on[1]=TV_power_on[0];
  Serial.print(" STR_DE845_POWER_PIN=");
  Serial.println(STR_DE845_POWER_PIN);
  pinMode(STR_DE845_power_on,INPUT);
  STR_DE845_power_on[0]=digitalRead(TV_POWER_PIN);
  STR_DE845_power_on[1]=STR_DE845_power_on[0];

  Serial.println("### Finished setup() ###");
};

void loop() {
  //IR Input check
  if (IrReceiver.decode()) {
    Serial.print("Recv IR code. ");
    if (IrReceiver.decodedIRData.protocol == TV_Remote_Proto) {
      Serial.print("Proto ok. ");
      if(IrReceiver.decodedIRData.address == TV_Remote_Address){
      Serial.print("Addr ok. ");
      for(uint8_t iCmd = 0; iCmd<=mapped_commands - 1; iCmd++)
        if(IrReceiver.decodedIRData.command == command_mapping[iCmd][0]){
          Serial.print("Cmd ok (");
          Serial.print(command_mapping[iCmd][0]);
          Serial.print("). Sending ");
          Serial.print(command_mapping[iCmd][1]);
          Serial.print(" as ");
          Control_S_send(Control_S_Address, command_mapping[iCmd][1],command_mapping[iCmd][2]);
        }
      }
    }
    Serial.println();
    IrReceiver.resume();
  }

  //TV or STR_DE845 state check
  TV_power_on[1]=digitalRead(TV_POWER_PIN);
  STR_DE845_power_on[1]=digitalRead(TV_POWER_PIN);
  if(TV_power_on[1]!=TV_power_on[0] && TV_power_on[1]==0){ //TV turned off
    if(STR_DE845_power_on[1]==1){ //STR-DE845 is still on
      Control_S_send(Control_S_Address, command_mapping[Control_S_power_map_id][1],command_mapping[Control_S_power_map_id][2]);
    }
  } else if(TV_power_on[1]!=TV_power_on[0] && TV_power_on[1]==1){ //TV turned on
    if(STR_DE845_power_on[1]==0){ //STR-DE845 is still off
      Control_S_send(Control_S_Address, command_mapping[Control_S_power_map_id][1],command_mapping[Control_S_power_map_id][2]);
    }
  }
};