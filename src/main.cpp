// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
/*
 *
 * Battery management system, 20V max and 20A max
 * default shunt resistor 0.001 Ohm, voltage devider 10K/1K
 * 
 * ESP8266 to ADS1115 wiring scheme:
 * 
 * 												 Rshunt (current sensing)
 * 											                +---|/\/\/\|----+
 *  _ _ _ _ _ _           _ _ _ _ _        _|				|
 * |		      D1|--------|SCL	AIN0|-------+				|
 * |		      D2|--------|SDA	AIN1|-----------------------+
 * |		        |    	   |	  		|		Voltage sensing
 * |		        |		|		AIN2|---------( V )-----+
 * |		        |		|		AIN3|---+				|
 *  - - - - - -			 - - - - - -	|				|
 * 										+---------------+	
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "ADS1115.h"
#include <Arduino_GFX_Library.h>

#define DEBUG // remove or comment to disable serial debug

#define HOUR_MILLIS 3.6e6
#define MEASURE_INTERVAL 500 	// millis
#define PRINT_INTERVAL 1000 	// millis
#define SHUNT_RESISTOR_mOhm 1.0F // 0.001 Ohm resistor value in milliohms
#define SHUNT_RESISTOR_Ohm 0.001 // 0.001 Ohm resistor value in milliohms
/*
 * Vsource x R2 / (R1 + R2) = Vout
 * Vsource = Vo x (R1 + R2) / R2
 * (10k + 6.8k) / 6.8k = 2.47 -> ratio
 * Vsource = Vo x 2.47
 */
// #define VOLTAGE_DIVIDER_RATIO 2.47 // Vsource / Vout
/*
 * current calculation for gain 2.048V and 0.001 Ohm current shunt
 * ±2.048 / 2^16 = 00000625 V measurement interval and minimum measure voltage
 * 00000625 / 0.001 = 0.0625 (62.5mA) A current measurement inreval and min measure current
 * (0.0625 * 2^16)/2 = ±2048 A teoretical maximum current
 * 
 * NB
 * To get higher current accuracy we can set as low gain as possible, but we should wery carefuly control MUX channels
 * to avoid overvoltage and burn ADC
*/
#define VOLTAGE_DIVIDER_RATIO 11 // Vsource / Vout

extern "C" {
ADS1115 adc0(ADS1115_DEFAULT_ADDRESS); 
}

#define WIFI_SSID "MySSID"
#define WIFI_PASSWORD "mypassword"

#define MQTT_HOST IPAddress(127,0,0,1)
#define MQTT_PORT 1883

/*
 * Ring buffer structure for graph 
 */
#define MAX_SIZE 20

extern "C" {
/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_ESP8266SPI(D8 /* DC */, -1);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, D3 /* RST */, 0 /* rotation */, true /* IPS */,
  240 /* width */, 240 /* height */);
}
// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00


extern "C" {
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

typedef struct {
  float data[MAX_SIZE];
  int head;
  int tail;
  int size;
} Queue;
}

extern "C" void enqueue(Queue *q, float value) {
  if (q->size == MAX_SIZE) {
    // printf("Warning: queue is full, removing oldest element\n");
    q->head = (q->head + 1) % MAX_SIZE;
    q->size--;
  }
  q->data[q->tail] = value;
  q->tail = (q->tail + 1) % MAX_SIZE;
  q->size++;
}

extern "C" float dequeue(Queue *q) {
  float value;
  if (q->size == 0) {
    // printf("Error: queue is empty\n");
    value = -1.0F;
  } else
  {
    value = q->data[q->head];
    q->head = (q->head + 1) % MAX_SIZE;
    q->size--;
  }

  return value;
}

extern "C" float queue_get(const Queue *q, int index) {
  float result;
  if (index < 0 || index >= q->size) {
    // printf("Error: index out of range\n");
    result = INFINITY;
  } else {

  result = q->data[(q->head + index) % MAX_SIZE];
  }
  return result;
}

extern "C" void draw_graph(const Queue *q, int x0, int y0, int w, int h, uint16_t color = ST77XX_GREEN, uint16_t bg = BLUE, bool bicolor = false, uint16_t color2 = ST77XX_GREEN) {
  int16_t gy = y0+h;
  uint16_t _color = color;
  float min_y = h;
  float max_y = 0;
  for (int i = 0; i < q->size; i++)
  {
    if (min_y > queue_get(q,i)) {
      min_y = queue_get(q,i);
    }
    if (max_y < queue_get(q,i)) {
      max_y = queue_get(q,i);    
    }
  }

  // calculate scaling
  int ratio = static_cast<int>(static_cast<float>(h) / (max_y - min_y));

  int window = w - 15;
  int sparse_rate = window / q->size;

  gfx->fillRect(x0, y0, w, h+1, bg); // draw graph background
  for (int i = 0; i < q->size; i++)
  {
    if (i > 0)
    {
      if (bicolor)
      {
        if (queue_get(q, i-1) < queue_get(q, i))
        {
          _color = color;
          
        } 
        else {
          _color = color2;
          
        }
      }
      
      int16_t _y0 = static_cast<uint16_t>((queue_get(q, i-1) - min_y) * static_cast<float>(ratio));
      int16_t _y1 = static_cast<uint16_t>((queue_get(q, i) - min_y) * static_cast<float>(ratio));
      gfx->drawLine(x0+15+(i-1)*sparse_rate, gy-_y0, x0+15+i*sparse_rate, gy-_y1, _color);        
    }
  }
  if (queue_get(q, q->size - 2) < queue_get(q, q->size-1))
  {
    gfx->fillRect(x0, y0, 10, h+1, bg); // draw graph background
    gfx->fillTriangle(x0, y0+h-(h/4), x0+5, y0+(h/4), x0+10, y0+h-(h/4), GREEN);
  } 
  else
  {
    gfx->fillRect(x0, y0, 10, h+1, bg); // draw graph background
    gfx->fillTriangle(x0, y0+(h/4), x0+5, y0+h-(h/4), x0+10, y0+(h/4), RED);
  }  
}

extern "C" void draw_node(int x0 , int y0, int w, int h, char label, float value, const Queue *q, uint16_t label_color = RED, uint16_t border_color = 0xFC00, uint16_t text_color = RED)
{
  int16_t border_offset = 12;
  uint8_t label_t_sz = 4;
  uint8_t label_t_w = 4*6;
  // uint8_t label_t_h = 4*8;
  gfx->setCursor(x0,y0+8);
  gfx->setTextSize(label_t_sz); // sz 4 = 24x30
  gfx->setTextColor(label_color, BLACK);
  gfx->print(label);
  gfx->drawRoundRect(x0+label_t_w+1, y0, w, h, 10, border_color);
  gfx->setCursor(x0+border_offset+label_t_w+1,y0+border_offset);
  gfx->setTextSize(3);
  gfx->setTextColor(text_color, BLACK);
  if (value < 10.0F)
  {
    gfx->print(value,3);
  } else if (value < 100.0F) {
    gfx->print(value,2);
  } else if (value < 1000.0F) {
    gfx->print(value,1);
  } else if (value < 10000.0F)
  {
    gfx->print(' ');
    gfx->print(value,0);
  } else if (value < 100000.0F)
  { // 10_000 .. 99_999
    float kf = value / 1000.0F;
    float head = static_cast<int>(kf);
    float frac = kf - head;
    int frac_int = frac * 100.0F;
    gfx->print(static_cast<int>(head));
    gfx->print('K');
    gfx->print(frac_int);
  } else if (value < 1000000.0F)
  {
    float kf = value / 1000.0F;
    float head = static_cast<int>(kf);
    float frac = kf - head;
    int frac_int = frac * 10.0F;
    gfx->print(static_cast<int>(head));
    gfx->print('K');
    gfx->print(frac_int);
  } else if (value < 10000000.0F)
  {
    float kf = value / 1000000.0F;
    float head = static_cast<int>(kf);
    float frac = kf - head;
    int frac_int = frac * 1000000.0F;
    gfx->print(static_cast<int>(head));
    gfx->print('M');
    gfx->print(frac_int);
  } else
  {
    gfx->print(F("OVERF"));
  }
  

  draw_graph(q, x0+w+1+label_t_w+1+10, y0+8, gfx->width()-(x0+w+1+label_t_w+1+10)-1, 30, ST77XX_GREEN, BLACK, true, ST77XX_ORANGE);

}

extern "C" void connectToWifi() {
	#ifdef DEBUG
  	Serial.println(F("Connecting to Wi-Fi..."));
	#endif
	WiFi.begin(F(WIFI_SSID), F(WIFI_PASSWORD));
}

extern "C" void connectToMqtt() {
	#ifdef DEBUG
  	Serial.println(F("Connecting to MQTT..."));
	#endif
  	mqttClient.connect();
}

extern "C" void onWifiConnect(const WiFiEventStationModeGotIP& event) {
	#ifdef DEBUG
  	Serial.println(F("Connected to Wi-Fi."));
  	Serial.println(event.ip);
	#endif
  	connectToMqtt();
}

extern "C" void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
	#ifdef DEBUG
  	Serial.println(F("Disconnected from Wi-Fi."));
  	Serial.println(static_cast<int>(event.reason));
	#endif
  	mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  	wifiReconnectTimer.once(2.0F, connectToWifi);
}

extern "C" void onMqttConnect(bool sessionPresent) {
	#ifdef DEBUG
	Serial.println(F("Connected to MQTT."));
	Serial.print(F("Session present: "));
	Serial.println(sessionPresent);
	#endif
}

extern "C" void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
	#ifdef DEBUG
  	Serial.println(F("Disconnected from MQTT."));
  	Serial.println(static_cast<int>(reason));
	#endif

  if (WiFi.isConnected()) {
	mqttReconnectTimer.once(2.0F, connectToMqtt);
  }
}

extern "C" {
Queue power_q;
Queue watts_q;
Queue amps_q;
Queue volts_q;
// float random_values[MAX_SIZE*4];
}

extern "C" void setup() {
	power_q.head = 0;
    power_q.tail = 0;
    power_q.size = 0;

	watts_q.head = 0;
    watts_q.tail = 0;
    watts_q.size = 0;

	amps_q.head = 0;
    amps_q.tail = 0;
    amps_q.size = 0;

	volts_q.head = 0;
    volts_q.tail = 0;
    volts_q.size = 0;

    // for (size_t i = 0; i < MAX_SIZE*4; i++)
    // {
    //   random_values[i] = (float)random(700,1500)/100;
    // }

    gfx->begin();
    gfx->fillScreen(BLACK);

#ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
#endif

    gfx->setCursor(10, 10);
    gfx->setTextColor(RED, BLACK);
    gfx->println(F("Acid Battery Management System"));
    // gfx->setCursor(10, 50);

    Wire.begin();  // join I2C bus
	#ifdef DEBUG
    Serial.begin(74800); // initialize serial communication
    delay(2000);
    Serial.println(ESP.getResetReason());
    Serial.println(ESP.getResetInfo());
    Serial.println(F("Initializing I2C devices...")); 
	#endif
    adc0.initialize(); // initialize ADS1115 16 bit A/D chip
    
	#ifdef DEBUG
    Serial.println(F("Testing device connections..."));
    Serial.println(adc0.testConnection() ? F("ADS1115 connection successful") : F("ADS1115 connection failed"));
	#endif
      
    // To get output from this method, you'll need to turn on the 
    //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
    adc0.showConfigRegister();
    
    // We're going to do continuous sampling
    adc0.setMode(ADS1115_MODE_CONTINUOUS);
	// Set gain to 2.048 volts as our supply 3.3v
	adc0.setGain(ADS1115_PGA_2P048);
	adc0.setRate(ADS1115_RATE_8); // low reate -> lower noise

	wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
	wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.setServer(MQTT_HOST, MQTT_PORT);

	connectToWifi();
}

extern "C" void loop() {
	static float watts = 0;
	static float wattsConsumed = 0;
	static float ampsConsumed = 0;
	static int64_t ampsConsumedADC = 0; // raw ADC counts
	static unsigned long measure_init = millis(), print_init = millis(), current_millis;
	static int sensorOneCounts = -1, sensorTwoCounts = -1;
	static float amps = 0, volts = 0, power = 0;
	static bool VA = true;

  current_millis = millis();
	if (current_millis - measure_init > MEASURE_INTERVAL/2)
	{
		measure_init = current_millis;		
		
    //*** Amp ***		
		if (VA) // Amp measurement
		{
			if (sensorOneCounts == -1) // if first measure
			{
				/* code */
				sensorOneCounts=adc0.getConversionP2N3();  // counts up to 16-bits
				adc0.setMultiplexer(ADS1115_MUX_P0_N1); // set MUX to measure Voltage in next cycle
			} else
			{
				sensorOneCounts=adc0.getConversion();  // MUX for amps should be set in voltage measure section
				adc0.setMultiplexer(ADS1115_MUX_P0_N1); // set MUX to measure Voltage in next cycle
			}
			
			
			ampsConsumedADC += sensorOneCounts;
			amps = static_cast<float>(sensorOneCounts) * adc0.getMvPerCount() / 1000.0F / SHUNT_RESISTOR_Ohm; // Amps, 500mOhms is shunt resistor value

			VA = !VA; // flip measure type
		} else // Voltage measurement
		{
			// *** Voltage ***
			// Manually set the MUX  // could have used the getConversionP* above
			// adc0.setMultiplexer(ADS1115_MUX_P2_N3); 
			// sensorTwoCounts=adc0.getConversionP0N1();  // counts up to 16-bits
			sensorTwoCounts=adc0.getConversion();  // ADC raw
			adc0.setMultiplexer(ADS1115_MUX_P2_N3); // set MUX to measure Amps in next cycle
			volts = static_cast<float>(sensorTwoCounts) * adc0.getMvPerCount() / 1000.0F * static_cast<float>(VOLTAGE_DIVIDER_RATIO); // Volts
			power = volts * amps;
			wattsConsumed+= power;

			VA = !VA; // flip measure type
		}
		
		

	}

  current_millis = millis();
	if (current_millis - print_init > PRINT_INTERVAL)
	{
		print_init = current_millis;
		// Get the number of counts of the accumulator

		// To turn the counts into a voltage, we can use
		#ifdef DEBUG
		Serial.print(F("ADC1="));
		Serial.print(sensorOneCounts);
		Serial.print(F(" ADC2="));
		Serial.print(sensorTwoCounts);
		// Serial.print(" mV Per Count=");
		// Serial.print(adc0.getMvPerCount(),4);
		Serial.print(F(" mVoltage="));
		Serial.print(static_cast<float>(sensorOneCounts)*adc0.getMvPerCount(), 4);
		Serial.print(F("mV"));
		Serial.print(F(" Amp="));
		Serial.print(amps,4);
		Serial.print('A');

		// Serial.print("	ADC2=");
		// Serial.print(sensorTwoCounts);  

		Serial.print(F(" Voltage="));
		Serial.print(volts,4);
		#endif
		
		watts = wattsConsumed / (HOUR_MILLIS / static_cast<float>(MEASURE_INTERVAL));
		ampsConsumed = static_cast<double_t>(ampsConsumedADC) * adc0.getMvPerCount() / SHUNT_RESISTOR_mOhm / (HOUR_MILLIS / static_cast<float>(MEASURE_INTERVAL));
		#ifdef DEBUG
		Serial.print(F(" Power=")); Serial.print(power,3); Serial.print('W');
		Serial.print(F(" Total_Amps=")); Serial.print(ampsConsumed,3); Serial.print(F("A•h"));
		Serial.print(F(" Total_Watts=")); Serial.print(watts,3); Serial.print(F("W•h"));
		Serial.print(F(" WattsConsumed=")); Serial.print(wattsConsumed,3); Serial.print('W');
    Serial.print(F(" Heap:")); Serial.print(ESP.getFreeHeap());
    Serial.print(F(" Frag:")); Serial.print(ESP.getHeapFragmentation());
		Serial.println();
		#endif

		// print values on display
		enqueue(&power_q, power);
		enqueue(&watts_q, watts);
		enqueue(&amps_q, amps);
		enqueue(&volts_q, volts);

		draw_node(0,192,114,48,'V',volts, &volts_q, RED, ST77XX_ORANGE,RED);
		draw_node(0,140,114,48,'A', amps, &amps_q, DARKGREEN, PURPLE,DARKGREEN);

		// float instant_power = random_values[iter]*random_values[iter];
		draw_node(0,88,114,48,'P', power, &power_q, PURPLE, DARKCYAN,PURPLE);
		// watts_accumulator += instant_power;
		draw_node(0,36,114,48,'W', watts, &watts_q, OLIVE, MAROON,OLIVE);

		// sent telemetry to mqtt broker
		char buff[10]; // 9 digits + \0
		dtostrf(volts, 0, 4U, buff);
		mqttClient.publish("battery/voltage", 0, false, buff);
		dtostrf(amps, 0, 4U, buff);
		mqttClient.publish("battery/current", 0, false, buff);
		dtostrf(power, 0, 4U, buff);
		mqttClient.publish("battery/power", 0, false, buff);
		dtostrf(ampsConsumed, 0, 6U, buff);
		mqttClient.publish("battery/amps", 0, false, buff);
		dtostrf(watts, 0, 6U, buff);
		mqttClient.publish("battery/watts", 0, false, buff);
    itoa(ESP.getFreeHeap(), buff, 10);
		mqttClient.publish("battery/heap", 0, false, buff);
	}
	
}
