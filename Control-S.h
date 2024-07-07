//Provides ability to SEND Sony Control-S (i.e., the SIRC protocol but with TTL logic and no carrier PWM)
	//READ is not implemented
//Code taken from https://github.com/Arduino-IRremote with minor modification outside of simplifications
	//It may be possible and more practical to use the IRremote library instead of this one, but some settings will require changes to produce the same output waveform

//#include <digitalWriteFast.h>

#ifndef Control_S_sendPin
#define Control_S_sendPin 4
#endif

#define SONY_ADDRESS_BITS       5
#define SONY_COMMAND_BITS       7
#define SONY_UNIT               600 // 24 periods of 40kHz

#define SONY_HEADER_MARK        (4 * SONY_UNIT) // 2400
#define SONY_ONE_MARK           (2 * SONY_UNIT) // 1200
#define SONY_ZERO_MARK          SONY_UNIT
#define SONY_SPACE              SONY_UNIT

#define SONY_REPEAT_PERIOD          45000 // Commands are repeated every 45 ms (measured from start to start) for as long as the key on the remote control is held down.

/**
 * Custom delay function that circumvents Arduino's delayMicroseconds 16 bit limit
 * and is (mostly) not extended by the duration of interrupt codes like the millis() interrupt
 */
void Control_S_customDelayMicroseconds(uint16_t aMicroseconds) {
#if defined(ESP32) || defined(ESP8266)
    // from https://github.com/crankyoldgit/IRremoteESP8266/blob/00b27cc7ea2e7ac1e48e91740723c805a38728e0/src/IRsend.cpp#L123
    // Invoke a delay(), where possible, to avoid triggering the WDT.
    // see https://github.com/Arduino-IRremote/Arduino-IRremote/issues/1114 for the reason of checking for > 16383)
    // delayMicroseconds() is only accurate to 16383 us. Ref: https://www.arduino.cc/en/Reference/delayMicroseconds
    if (aMicroseconds > 16383) {
        delay(aMicroseconds / 1000UL);  // Delay for as many whole milliseconds as we can.
        // Delay the remaining sub-millisecond.
        delayMicroseconds(static_cast<uint16_t>(aMicroseconds % 1000UL));
    } else {
        delayMicroseconds(aMicroseconds);
    }
#else

#  if defined(__AVR__)
    unsigned long start = micros() - (64 / clockCyclesPerMicrosecond()); // - (64 / clockCyclesPerMicrosecond()) for reduced resolution and additional overhead
#  else
    unsigned long start = micros();
#  endif
// overflow invariant comparison :-)
    while (micros() - start < aMicroseconds) {
    }
#endif
}

void Control_S_mark(uint16_t aMarkMicros) {
	//digitalWriteFast(Control_S_sendPin, HIGH); // Set output to active low.
	digitalWrite(Control_S_sendPin,HIGH);
    Control_S_customDelayMicroseconds(aMarkMicros);
	//digitalWriteFast(Control_S_sendPin, LOW); // Set output to active low.
	digitalWrite(Control_S_sendPin,LOW);
}

void Control_S_space(uint16_t aSpaceMicros) {
    Control_S_customDelayMicroseconds(aSpaceMicros);
}


uint16_t Control_S_get_data_bits(uint16_t aAddress, uint8_t aCommand, uint8_t aNumberOfBits = 12) {
	uint16_t Control_S_data_bits = 0;
	for(uint_fast8_t iBit = 0; iBit<=aNumberOfBits-1;iBit++){
		if(iBit < 7){ //Command
			bitWrite(Control_S_data_bits,aNumberOfBits-1-iBit,bitRead(aCommand,iBit));
		} else { //Address
			bitWrite(Control_S_data_bits,aNumberOfBits-1-iBit,bitRead(aAddress,iBit-SONY_COMMAND_BITS));
		}
	}
	return Control_S_data_bits;
}

void Control_S_send(uint16_t aAddress, uint8_t aCommand, int_fast8_t aNumberOfRepeats, uint8_t aNumberOfBits = 12) {
	uint_fast8_t tNumberOfCommands = aNumberOfRepeats + 1;
	uint_fast16_t tDataBits = 0;
	tDataBits=Control_S_get_data_bits(aAddress,aCommand,aNumberOfBits);
	Serial.print(tDataBits, BIN);
	while (tNumberOfCommands > 0) {
        unsigned long tStartOfFrameMillis = millis();
        /*
        * Send Header
        */
        Control_S_mark(SONY_HEADER_MARK);
        Control_S_space(SONY_SPACE);

		/*
		* Send Command then Address bits
		*/
		for(uint_fast8_t iBit = 0; iBit<=aNumberOfBits-1;iBit++){
			if(bitRead(tDataBits,aNumberOfBits-iBit-1) == 1){
				Control_S_mark(SONY_ONE_MARK);
				Control_S_space(SONY_SPACE);
			} else if(bitRead(tDataBits,aNumberOfBits-iBit-1) == 0){
				Control_S_mark(SONY_ZERO_MARK);
				Control_S_space(SONY_SPACE);
			}	
		}
        /*
        * Wait between repeats
        */
        tNumberOfCommands--; // skip last delay!
        if (tNumberOfCommands > 0) {
            /*
             * Check and fallback for wrong RepeatPeriodMillis parameter. I.e the repeat period must be greater than each frame duration.
             */
            auto tFrameDurationMillis = millis() - tStartOfFrameMillis;
            if (SONY_REPEAT_PERIOD > tFrameDurationMillis) {
                Control_S_customDelayMicroseconds(SONY_REPEAT_PERIOD - tFrameDurationMillis);
            }
        }
    }	
}






