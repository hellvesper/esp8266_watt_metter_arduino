/*
 *
 * Watt meter battery capacity tester 5V max, 4A max
 * default shunt resistor 0.5 Ohm, voltage devider 10K/6.8K
 * 
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "ADS1115.h"

#define DEBUG true

#define HOUR_MILLIS 3.6e6
#define MEASURE_INTERVAL 500 	// millis
#define PRINT_INTERVAL 1000 	// millis
#define SHUNT_RESISTOR_mOhm 500 // resistor value in milliohms
/*
 * Vsource x R2 / (R1 + R2) = Vout
 * Vsource = Vo x (R1 + R2) / R2
 * (10k + 6.8k) / 6.8k = 2.47 - ratio
 * Vsource = Vo x 2.47
 */
#define VOLTAGE_DIVIDER_RATIO 2.47

ADS1115 adc0(ADS1115_DEFAULT_ADDRESS); 

#define WIFI_SSID "MySSID"
#define WIFI_PASSWORD "mypassword"

#define MQTT_HOST IPAddress(127,0,0,1)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void connectToWifi() {
	#ifdef DEBUG
  	Serial.println("Connecting to Wi-Fi...");
	#endif
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
	#ifdef DEBUG
  	Serial.println("Connecting to MQTT...");
	#endif
  	mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
	#ifdef DEBUG
  	Serial.println("Connected to Wi-Fi.");
	#endif
  	connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
	#ifdef DEBUG
  	Serial.println("Disconnected from Wi-Fi.");
	#endif
  	mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  	wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
	#ifdef DEBUG
	Serial.println("Connected to MQTT.");
	Serial.print("Session present: ");
	Serial.println(sessionPresent);
	#endif
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
	#ifdef DEBUG
  	Serial.println("Disconnected from MQTT.");
	#endif

  if (WiFi.isConnected()) {
	mqttReconnectTimer.once(2, connectToMqtt);
  }
}


void setup() {                
    Wire.begin();  // join I2C bus
	#ifdef DEBUG
    Serial.begin(19200); // initialize serial communication 
    Serial.println("Initializing I2C devices..."); 
	#endif
    adc0.initialize(); // initialize ADS1115 16 bit A/D chip
    
	#ifdef DEBUG
    Serial.println("Testing device connections...");
    Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
	#endif
      
    // To get output from this method, you'll need to turn on the 
    //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
    adc0.showConfigRegister();
    
    // We're going to do continuous sampling
    adc0.setMode(ADS1115_MODE_CONTINUOUS);
	adc0.setGain(ADS1115_PGA_2P048);

	wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
	wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.setServer(MQTT_HOST, MQTT_PORT);

	connectToWifi();
}

void loop() {
	static float watts = 0;
	static float wattsConsumed = 0;
	static float ampsConsumed = 0;
	static int64_t ampsConsumedADC = 0; // raw ADC counts
	static unsigned long measure_init, print_init = millis();
	static int sensorOneCounts, sensorTwoCounts = 0;
	static float loadCurrent, sourceVoltage, loadPower = 0;

	if (millis() - measure_init > MEASURE_INTERVAL)
	{
		measure_init = millis();		
		//*** Amp ***
		sensorOneCounts = 0;
		sensorTwoCounts = 0;
		
		sensorOneCounts=adc0.getConversionP0N1();  // counts up to 16-bits
		ampsConsumedADC += sensorOneCounts;
		loadCurrent = sensorOneCounts * adc0.getMvPerCount() / 500; // Amps, 500mOhms is shunt resistor value

		// *** Voltage ***
		// Manually set the MUX  // could have used the getConversionP* above
		adc0.setMultiplexer(ADS1115_MUX_P2_N3); 
		sensorTwoCounts=adc0.getConversion();  // ADC raw
		sourceVoltage = sensorTwoCounts * adc0.getMvPerCount() / 1000 * VOLTAGE_DIVIDER_RATIO; // Volts
		wattsConsumed+= sourceVoltage * loadCurrent;
	}

	if (millis() - print_init > PRINT_INTERVAL)
	{
		print_init = millis();
		// Get the number of counts of the accumulator
		// Serial.print("ADC1=");
		// Serial.print(sensorOneCounts);

		// To turn the counts into a voltage, we can use
		#ifdef DEBUG
		Serial.print(" mVoltage=");
		Serial.print(sensorOneCounts*adc0.getMvPerCount(), 4);
		Serial.print("mV");
		Serial.print(" Amp=");
		Serial.print(loadCurrent,4);
		Serial.print("A");

		// Serial.print("	ADC2=");
		// Serial.print(sensorTwoCounts);  

		Serial.print(" Voltage=");
		Serial.print(sourceVoltage,4);
		#endif
		
		loadPower = sourceVoltage * loadCurrent;
		
		watts = wattsConsumed / (HOUR_MILLIS / MEASURE_INTERVAL);
		ampsConsumed = ampsConsumedADC * adc0.getMvPerCount() / 500 / (HOUR_MILLIS / MEASURE_INTERVAL);
		#ifdef DEBUG
		Serial.print("	Power="); Serial.print(loadPower,4); Serial.print("W");
		Serial.print(" Total_Amps="); Serial.print(ampsConsumed,6); Serial.print("A•h");
		Serial.print(" Total_Watts="); Serial.print(watts,6); Serial.print("W•h");
		Serial.println();
		#endif
		
		// delay(MEASURE_INTERVAL); // sleep all time left till next measure
		// int wait = MEASURE_INTERVAL - (millis() - measure_init);
    	// delay(MEASURE_INTERVAL - (millis() - measure_init)); // sleep all time left till next measure
		
		char buff[10]; // 9 digits + \0
		dtostrf(sourceVoltage, 0, 4, buff);
		mqttClient.publish("battester/voltage", 0, false, buff);
		dtostrf(loadCurrent, 0, 4, buff);
		mqttClient.publish("battester/current", 0, false, buff);
		dtostrf(loadPower, 0, 4, buff);
		mqttClient.publish("battester/power", 0, false, buff);
		dtostrf(ampsConsumed, 0, 6, buff);
		mqttClient.publish("battester/amps", 0, false, buff);
		dtostrf(watts, 0, 6, buff);
		mqttClient.publish("battester/watts", 0, false, buff);
	}
	
}
