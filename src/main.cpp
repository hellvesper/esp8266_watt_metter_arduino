/*
 *
 * Battery management system, 20V max and 20A max
 * default shunt resistor 0.001 Ohm, voltage devider 10K/1K
 * 
 * ESP8266 to ADS1115 wiring scheme:
 * 
 * 												 Rshunt (current sensing)
 * 											+---|/\/\/\|----+
 *  _ _ _ _ _ _			 _ _ _ _ _ _		|				|
 * |		 D1|--------|SCL	AIN0|-------+				|
 * |		 D2|--------|SDA	AIN1|-----------------------+
 * |		   |		|			|		Voltage sensing
 * |		   |		|		AIN2|---------( V )-----+
 * |		   |		|		AIN3|---+				|
 *  - - - - - -			 - - - - - -	|				|
 * 										+---------------+	
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "ADS1115.h"

#define DEBUG // remove or comment to disable serial debug

#define HOUR_MILLIS 3.6e6
#define MEASURE_INTERVAL 500 	// millis
#define PRINT_INTERVAL 1000 	// millis
#define SHUNT_RESISTOR_mOhm 1 // 0.001 Ohm resistor value in milliohms
/*
 * Vsource x R2 / (R1 + R2) = Vout
 * Vsource = Vo x (R1 + R2) / R2
 * (10k + 6.8k) / 6.8k = 2.47 -> ratio
 * Vsource = Vo x 2.47
 */
// #define VOLTAGE_DIVIDER_RATIO 2.47 // Vsource / Vout
/*
 * current calculation for gain 2.048V and 0.001 Ohm current shunt
 * 2.048 / 2^16 = 0,00003125 V measurement interval and minimum measure voltage
 * 0,00003125 / 0.001 = 0,03125 (31.25mA) A current measurement inreval and min measure current
 * 0.03125 * 2^16 = 2048 A teoretical maximum current
 * 
 * NB
 * To get higher current accuracy we can set as low gain as possible, but we should wery carefuly control MUX channels
 * to avoid overvoltage and burn ADC
*/
#define VOLTAGE_DIVIDER_RATIO 11 // Vsource / Vout

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
	// Set gain to 2.048 volts as our supply 3.3v
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
		loadCurrent = sensorOneCounts * adc0.getMvPerCount() / SHUNT_RESISTOR_mOhm; // Amps, 500mOhms is shunt resistor value

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

		// sent telemetry to mqtt broker
		char buff[10]; // 9 digits + \0
		dtostrf(sourceVoltage, 0, 4, buff);
		mqttClient.publish("battery/voltage", 0, false, buff);
		dtostrf(loadCurrent, 0, 4, buff);
		mqttClient.publish("battery/current", 0, false, buff);
		dtostrf(loadPower, 0, 4, buff);
		mqttClient.publish("battery/power", 0, false, buff);
		dtostrf(ampsConsumed, 0, 6, buff);
		mqttClient.publish("battery/amps", 0, false, buff);
		dtostrf(watts, 0, 6, buff);
		mqttClient.publish("battery/watts", 0, false, buff);
	}
	
}
