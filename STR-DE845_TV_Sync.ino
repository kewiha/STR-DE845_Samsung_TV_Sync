#include <Arduino.h>
#include "Control-S.h"

#include "PinDefinitionsAndMore.h"
#define DECODE_SAMSUNG
#define NO_LED_FEEDBACK_CODE
#include <IRremote.hpp>  // include the library

unsigned int millis_var;

// Power details
#define TV_POWER_PIN 5
#define STR_DE845_POWER_PIN 6
bool TV_power_on[2]; //index 0 = past, index 1 = now
bool STR_DE845_power_on[2]; //index 0 = past, index 1 = now
#define wait_before_power_state_sync_ms 2000

// TV Remote Details
#define TV_Remote_Proto SAMSUNG
const uint16_t TV_Remote_Address = 0x007;

// Sony STR-DE845 Details
const uint16_t Control_S_Address = 16;
unsigned int last_Control_S_sent_time_ms = 0;

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
  pinMode(TV_POWER_PIN,INPUT); //External pulldown resistor needed. My TV cuts power to USB port shortly after power off, but the result is a floating input
  TV_power_on[0]=digitalRead(TV_POWER_PIN);
  TV_power_on[1]=TV_power_on[0];
  Serial.print(" STR_DE845_POWER_PIN=");
  Serial.println(STR_DE845_POWER_PIN);
  pinMode(STR_DE845_power_on,INPUT); //External pulldown resistor needed. Lower resistance (without exceeding safe current limits for Arduino) speeds up 5V -> 0V transition
  STR_DE845_power_on[0]=digitalRead(TV_POWER_PIN);
  STR_DE845_power_on[1]=STR_DE845_power_on[0];

  Serial.println("### Finished setup() ###");
};

void loop() {
  //IR Input check
  if (IrReceiver.decode()) {
    Serial.print("Recv IR code. Proto: ");
    Serial.print(IrReceiver.decodedIRData.protocol);
    if (IrReceiver.decodedIRData.protocol == TV_Remote_Proto) {
      Serial.print(" ok. Addr: ");
      Serial.print(IrReceiver.decodedIRData.address);
      if(IrReceiver.decodedIRData.address == TV_Remote_Address){
      Serial.print(" ok. Cmd: ");
      Serial.print(IrReceiver.decodedIRData.command);
      for(uint8_t iCmd = 0; iCmd<=mapped_commands - 1; iCmd++)
        if(IrReceiver.decodedIRData.command == command_mapping[iCmd][0]){
          Serial.print(" ok (");
          Serial.print(command_mapping[iCmd][0]);
          Serial.print("). Sending ");
          Serial.print(command_mapping[iCmd][1]);
          Serial.print(" as ");
          Control_S_send(Control_S_Address, command_mapping[iCmd][1],command_mapping[iCmd][2]);
          last_Control_S_sent_time_ms = millis();
        }
      }
    }
    Serial.println();
    IrReceiver.resume();
  }

  //TV or STR_DE845 sync check (e.g. in case TV turns off automatically, need to turn off STR-DE845 so devices are both on or both off)
  millis_var = millis();
  if (millis_var > last_Control_S_sent_time_ms) { //Ignore state checks when millis() just rolled over to 0
    if ((millis_var - last_Control_S_sent_time_ms) > wait_before_power_state_sync_ms ) { //Don't act on TV_POWER_PIN or STR_DE845_POWER_PIN right away in case STR-DE845 is already acting on a Control S signal
      TV_power_on[1]=digitalRead(TV_POWER_PIN);
      STR_DE845_power_on[1]=digitalRead(STR_DE845_POWER_PIN);
      if(TV_power_on[1]!=TV_power_on[0] && TV_power_on[1]==0){ //TV turned off
        Serial.print("TV turned off.");
        if(STR_DE845_power_on[1]==1){ //STR-DE845 is still on
          Serial.println(" STR-DE845 is still on, turning off.");
          Control_S_send(Control_S_Address, command_mapping[Control_S_power_map_id][1],command_mapping[Control_S_power_map_id][2]);
          STR_DE845_power_on[1]=!STR_DE845_power_on[1];
          last_Control_S_sent_time_ms = millis();
        } else {
          Serial.println(" STR-DE845 is off, nothing to do.");
          last_Control_S_sent_time_ms = millis(); //Dirty hack: no Control S signal sent
        }
        TV_power_on[0]=TV_power_on[1];
      } else if(TV_power_on[1]!=TV_power_on[0] && TV_power_on[1]==1){ //TV turned on
        Serial.print("TV turned on");
        if(STR_DE845_power_on[1]==0){ //STR-DE845 is still off
          Serial.println(" STR-DE845 is still off, turning on.");
          Control_S_send(Control_S_Address, command_mapping[Control_S_power_map_id][1],command_mapping[Control_S_power_map_id][2]);
          STR_DE845_power_on[1]=!STR_DE845_power_on[1];
          last_Control_S_sent_time_ms = millis();
        } else {
          Serial.println(" STR-DE845 is on, nothing to do.");
          last_Control_S_sent_time_ms = millis(); //Dirty hack: no Control S signal sent
        }
        TV_power_on[0]=TV_power_on[1];
      }
    }
  }
};
