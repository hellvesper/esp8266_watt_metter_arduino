// I2C device class (I2Cdev) demonstration Arduino sketch for ADS1115 class
// Example of reading two differential inputs of the ADS1115 and showing the value in mV
// 06 May 2013 by Frederick Farzanegan (frederick1@farzanegan.org)
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2013-05-13 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include <Arduino.h>
#include "ADS1115.h"

#define HOUR_MILLIS 3.6e6
#define MEASURE_INTERVAL 500 //millis

ADS1115 adc0(ADS1115_DEFAULT_ADDRESS); 

void setup() {                
    Wire.begin();  // join I2C bus
    Serial.begin(19200); // initialize serial communication 
    Serial.println("Initializing I2C devices..."); 
    adc0.initialize(); // initialize ADS1115 16 bit A/D chip
    
    Serial.println("Testing device connections...");
    Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
      
    // To get output from this method, you'll need to turn on the 
    //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
    adc0.showConfigRegister();
    
    // We're going to do continuous sampling
    adc0.setMode(ADS1115_MODE_CONTINUOUS);
}

void loop() {
	static float wattsConsumed = 0;
	static float ampsConsumed = 0;
	static uint64_t ampsConsumedADC, wattsConsumedADC = 0;
    // Sensor is on P0/N1 (pins 4/5)
    // Set the gain (PGA) +/- 2.048v
    adc0.setGain(ADS1115_PGA_2P048);

    // Get the number of counts of the accumulator
    Serial.print("ADC1 = ");
    
    // The below method sets the mux and gets a reading.
    int sensorOneCounts=adc0.getConversionP0N1();  // counts up to 16-bits
	ampsConsumedADC += sensorOneCounts;
    Serial.print(sensorOneCounts);

    // To turn the counts into a voltage, we can use
    Serial.print(" | Voltage = ");
    Serial.print(sensorOneCounts*adc0.getMvPerCount(), 4);
    Serial.print("mV");
	Serial.print(" | Amp = ");
	float loadCurrent = sensorOneCounts * adc0.getMvPerCount() / 500; // 500mOhms is shunt resistor value
	Serial.print(loadCurrent,4);
	Serial.print("A");

    // Manually set the MUX  // could have used the getConversionP* above
    adc0.setMultiplexer(ADS1115_MUX_P2_N3); 
	int sensorTwoCounts=adc0.getConversion();  // ADC raw
	wattsConsumedADC += sensorTwoCounts;
    Serial.print(" || ADC2 = ");
    Serial.print(sensorTwoCounts);  

    Serial.print(" | Voltage = ");
	float sourceVoltage = sensorTwoCounts * adc0.getMvPerCount() / 1000 * 2; // 2 - is resistor devider ratio
    Serial.print(sourceVoltage,4);
    
	float loadPower = sourceVoltage * loadCurrent;
	
	// wattsConsumed += loadPower;
	// ampsConsumed += loadCurrent;
	Serial.print(" || Power = "); Serial.print(loadPower,4); Serial.print("W•h");
	Serial.print(" | Total Amps = "); Serial.print(ampsConsumed,4); Serial.print("A•h");
	Serial.print(" | Total Watts = "); Serial.print(wattsConsumed,4); Serial.print("W");
	Serial.println();
    
    delay(MEASURE_INTERVAL);
}
