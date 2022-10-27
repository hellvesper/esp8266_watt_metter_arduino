/*
 *
 * Watt meter battery capacity tester 5V max, 4A max
 * default shunt resistor 0.5 Ohm, voltage devider 10K/6.8K
 * 
 */

#include <Arduino.h>
#include "ADS1115.h"

#define HOUR_MILLIS 3.6e6
#define MEASURE_INTERVAL 500 //millis
#define PRINT_INTERVAL 1000 //millis
#define SHUNT_RESISTOR_mOhm 500 // resistor value in milliohms

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
	adc0.setGain(ADS1115_PGA_2P048);
}

void loop() {
	static float watts = 0;
	static float wattsConsumed = 0;
	static float ampsConsumed = 0;
	static uint64_t ampsConsumedADC = 0; // raw ADC counts
	static unsigned long measure_init, print_init = millis();
	static int sensorOneCounts, sensorTwoCounts = 0;
	static float loadCurrent, sourceVoltage, loadPower = 0;

	if (millis() - measure_init > MEASURE_INTERVAL)
	{
		measure_init = millis();		
		//*** Amp ***
		
		sensorOneCounts=adc0.getConversionP0N1();  // counts up to 16-bits
		ampsConsumedADC += sensorOneCounts;
		loadCurrent = sensorOneCounts * adc0.getMvPerCount() / 500; // Amps, 500mOhms is shunt resistor value

		// *** Voltage ***
		// Manually set the MUX  // could have used the getConversionP* above
		adc0.setMultiplexer(ADS1115_MUX_P2_N3); 
		sensorTwoCounts=adc0.getConversion();  // ADC raw
		// wattsConsumedADC += sensorTwoCounts;
		sourceVoltage = sensorTwoCounts * adc0.getMvPerCount() / 1000 * 2; // Volts, 2 - is resistor devider ratio
		wattsConsumed+= sourceVoltage * loadCurrent;
	}
	

	if (millis() - print_init > PRINT_INTERVAL)
	{
		print_init = millis();
		// Get the number of counts of the accumulator
		Serial.print("ADC1=");
		
		Serial.print(sensorOneCounts);

		// To turn the counts into a voltage, we can use
		Serial.print(" mVoltage=");
		Serial.print(sensorOneCounts*adc0.getMvPerCount(), 4);
		Serial.print("mV");
		Serial.print(" Amp=");
		Serial.print(loadCurrent,4);
		Serial.print("A");

		Serial.print("	ADC2=");
		Serial.print(sensorTwoCounts);  

		Serial.print(" Voltage=");
		Serial.print(sourceVoltage,4);
		
		loadPower = sourceVoltage * loadCurrent;
		
		watts = wattsConsumed / (HOUR_MILLIS / MEASURE_INTERVAL);
		ampsConsumed += ampsConsumedADC * adc0.getMvPerCount() / 500 / (HOUR_MILLIS / MEASURE_INTERVAL);
		Serial.print("	Power="); Serial.print(loadPower,4); Serial.print("W");
		Serial.print(" Total_Amps="); Serial.print(ampsConsumed,6); Serial.print("A•h");
		Serial.print(" Total_Watts="); Serial.print(watts,6); Serial.print("W•h");
		Serial.println();
		
		// delay(MEASURE_INTERVAL); // sleep all time left till next measure
		// int wait = MEASURE_INTERVAL - (millis() - measure_init);
    	// delay(MEASURE_INTERVAL - (millis() - measure_init)); // sleep all time left till next measure

	}
	
}
