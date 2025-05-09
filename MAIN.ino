#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_NeoPixel.h>
#include <ADS1115.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <math.h>
#include "index.html"

// network credentials
const char* ssid = "Dude"; // Replace with your network SSID
const char* password = "password"; // Replace with your network password


// LED STATES:
// GREEN = WIFI CONNECTED & SYSTEM FULLY OPERATIONAL
// PINK = NO WIFI; SENSORS AND LIGHTS OPERATIONAL
// BLUE = WIFI CONNECTED; SENSORS FAILED
// WHITE = SETTING UP
// RED = NOTHING FUNCTIONAL!!!!


// ADJUSTABLE PARAMETERS
const float Vmin = 0.003; // adjust to min voltage measured when sensor is out of the water
float Vmax = 4.11; // adjust to max voltage seen when tank is at max volume
const float maxGallons = 46.4; //adjust based on the max volume of tank
const unsigned int NUMPIXELS = 100; // adjust based on number of individual LEDs in LED strip
const unsigned int LUX = 220; // choose value from 1-255 to adjust led brightness
const bool STATIC_IP_ENABLED = false; // set to "true" if you want to use a fixed IP
const IPAddress ip(10, 0, 0, 2); // adjust to desired IP address
const float alpha = 0.05; // constant parameter for low pass filtering

// CONSTANT DEFINITIONS
const unsigned int FAN1 = 20;
const unsigned int FAN2 = 21;
const unsigned int HEATER1 = 22;
const unsigned int HEATER2 = 23;
const unsigned int LED_PIN = 2;
enum HVAC_MODE {
  IDLE,
  HEATING,
  FAN1_HEATING,
  FANS_HEATING, 
  COOLING,
  NUM_HVAC_MODES
};


// INITIALIZATION
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // Temp Sensor
using namespace ADS1115; // 
ADS1115_ADC adc(I2CAddress::Gnd); // Initialize adc using default Wire object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_RGB + NEO_KHZ800);
AsyncWebServer server(80); // Create AsyncWebServer object on port 80
AsyncEventSource events("/events");
unsigned long lastTime = 0, lastTime1 = 0; 
bool sht41_on = false, adc_on = false, wifi_on = false;
float voltage = 0, dewPoint = 0, humidity = 0, temp = 0;
int level = 0, lit_pixels = 0, volume;
float rawLevel = 0, smoothLevel = 0;
float rawTemp = 0, smoothTemp = 0;
float rawHumidity = 0, smoothHumidity = 0;
bool ONLINE_CONTROL = 0;
HVAC_MODE currentMode = IDLE;


////////// FUNCTIONS //////////////////////////
bool connect_adc() {
  // Start ADS1115
  if (!adc.isConnected()) {
    // Handle the error
    return false;
  }
  Serial.println("adc detected.");
  adc.init();
  //Configure
  adc.setPga(Pga::FSR_6_144V);
  adc.setMux(Mux::P0_GND); // single ended read.
  adc.setDataRate(DataRate::SPS_128);
  adc.setAdcMode(AdcMode::SingleShot);
  adc.setComparatorQueue(CompQueue::Disable);

  adc.uploadConfig(); // must re send the config every time to reset the adc
  Serial.println("adc ready.");
  return true;
}

bool connect_sht41(){
  // Start SHT41
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  unsigned int start = millis();
  while (!sht4.begin(&Wire)) {
    Serial.println("Searching SHT4x sensor...");
    delay(500);
    if (millis() >= start + 2000) // try for 2 seconds
      return false;
  }

  Serial.println("SHT4x detected.");
  return true;
}


bool connect_wifi() {
  //INITIALIZE WIFI 
  if (STATIC_IP_ENABLED)  WiFi.config(ip);

  WiFi.begin(ssid, password);
  
  // Wait for connection
  unsigned int start = millis();
  while (WiFi.status() != WL_CONNECTED ) {
    delay(1000);
    Serial.print("Connecting to WiFi: ");
    Serial.print(ssid);
    Serial.println(" ...");
    if (millis() >= start + 10000) { //try for 10 seconds
      return false;
    } 
  }
  Serial.print("Succesfully connected to wifi!    IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  return true;
}

// Replaces placeholder with initial values. Needed for Webserver setup
String processor(const String& var) {
  if (var == "LEVEL") return "...";
  else if(var == "TIME") return "...";
  return String();
}


void connect_server() {
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, "text/html", index_html, processor);});
    // Handle Web Server Events
    events.onConnect([](AsyncEventSourceClient *client) {
      IPAddress remoteIP = client->client()->remoteIP();
      Serial.printf("Client connected from IP: %s\n", remoteIP.toString().c_str());
      if(client->lastId()) {
        Serial.printf("Client  @ %s reconnected! \n",  remoteIP.toString().c_str());
      } else {
       Serial.printf("Client @ %s connected for the first time \n",  remoteIP.toString().c_str());
      }
      client->send("Connected!", NULL, millis(), 1000); //set reconnect delay to 1 second
    });

    server.on("/setVmax", HTTP_GET, [](AsyncWebServerRequest *request){
    Vmax = voltage; // assume voltage is your latest ADC value
    request->send(200, "text/plain", "Vmax updated");
    Serial.println("Vmax set to: " + String(Vmax));
    });

    server.on("/ONLINE_CONTROL", HTTP_GET, [](AsyncWebServerRequest *request){
      ONLINE_CONTROL = !ONLINE_CONTROL;
      request->send(200, "text/plain", "CONTROL TOGGLED");
    });

    server.on("/next_mode", HTTP_GET, [](AsyncWebServerRequest *request){
      currentMode = static_cast<HVAC_MODE>((currentMode + 1) % NUM_HVAC_MODES);
      request->send(200, "text/plain", "mode changed");
    });

    // Start server
    server.addHandler(&events); //AsyncEventSource events("/events");
    server.begin();
}

void show_status_led() {
  //adjust onboard status light
  if (wifi_on) {
    if (adc_on && sht41_on)
      rgbLedWrite(RGB_BUILTIN,0,10,0); // GREEN
    else
      rgbLedWrite(RGB_BUILTIN,0,0,10); // BLUE;
  }
  else if (adc_on && sht41_on) {
    rgbLedWrite(RGB_BUILTIN,10,0,10); // PINK
  }
  else {
   rgbLedWrite(RGB_BUILTIN,10,0,0); // RED
  }
}


float calculateDewPoint(float temperatureC, float humidityPercent) {
  const float a = 17.625;
  const float b = 243.04; // degrees Celsius

  float gamma = log(humidityPercent / 100.0) + (a * temperatureC) / (b + temperatureC);
  return (b * gamma) / (a - gamma); //dew point
}


void control_HVAC(HVAC_MODE mode){
  switch (mode) {
    case IDLE:
      digitalWrite(HEATER1, 0);
      digitalWrite(HEATER2, 0);
      digitalWrite(FAN1, 0);
      digitalWrite(FAN2, 0);
      //show_status_led(); // recalculate general board status
      break;
    case HEATING:
      digitalWrite(HEATER1, 1);
      digitalWrite(HEATER2, 1);
      digitalWrite(FAN1, 0);
      digitalWrite(FAN2, 0);
      // rgbLedWrite(RGB_BUILTIN,100,30,0); // ORANGE
      break;
    case FAN1_HEATING:
      digitalWrite(HEATER1, 1);
      digitalWrite(HEATER2, 1);
      digitalWrite(FAN1, 1);
      digitalWrite(FAN2, 0);
      // rgbLedWrite(RGB_BUILTIN,20,20,0); // yellow
      break;
    case FANS_HEATING:
      digitalWrite(HEATER1, 1);
      digitalWrite(HEATER2, 1);
      digitalWrite(FAN1, 1);
      digitalWrite(FAN2, 1);
      // rgbLedWrite(RGB_BUILTIN,200,20,0); // ORANGE
      break;
    case COOLING:
      digitalWrite(HEATER1, 0);
      digitalWrite(HEATER2, 0);
      digitalWrite(FAN1, 1);
      digitalWrite(FAN2, 1);
      //rgbLedWrite(RGB_BUILTIN,0,10,10); // CYAN. 
      break;
  }
}


HVAC_MODE calculate_climate() {
  rawTemp = (temp - Vmin) * 100 / (Vmax - Vmin);
  smoothTemp = alpha * rawTemp + (1-alpha)*smoothTemp; // exponential smoothing, low-pass filter
  rawHumidity = (humidity - Vmin) * 100 / (Vmax - Vmin);
  smoothHumidity = alpha * rawHumidity + (1-alpha)*smoothHumidity; // exponential smoothing, low-pass filter
  // actuate moisture control
  if (temp < (dewPoint + 3))
    return HEATING;
  else if (temp > (dewPoint + 5)) {
    if (temp >= 28)
      return COOLING;
    else if (temp < 25)
      return IDLE;
  }

  return currentMode; //keep current mode in case of deadband. 
}



////////// ESP32 SETUP //////////////////////////

void setup() {
  Serial.begin(115200);
  digitalWrite(RGB_BUILTIN, HIGH); //STATUS ON BOARD LED
  pinMode(FAN1, OUTPUT);
  pinMode(FAN2, OUTPUT);
  pinMode(HEATER1, OUTPUT);
  pinMode(HEATER2, OUTPUT);

  Wire.begin(6, 7); // START I2C BUS (SDA, SCL)
  delay(1000);
  
  strip.begin();  // Initialize the strip
  strip.show();   // Initialize all pixels to 'off'

  adc_on = connect_adc();
  sht41_on = connect_sht41();
  wifi_on = connect_wifi();

  if (wifi_on)
    connect_server();
  else
    Serial.println("Could Not Connect to WiFi");

  show_status_led();

} // end of setup()



////////// MAIN LOOP //////////////////////////

void loop() {
  sensors_event_t h, t;
  // --- Condensation Control ---
  if (sht41_on) {
    sht4.getEvent(&h, &t);
    humidity = h.relative_humidity;
    temp = t.temperature;
    dewPoint = calculateDewPoint(temp, humidity);
    
    if (ONLINE_CONTROL) {
      //let server handle the change mode function. 
      control_HVAC(currentMode); //then control
    }
    else {
      currentMode = calculate_climate();
      control_HVAC(currentMode); // OPERATE heaters and fan
    }

  }

  // --- ADS1115 reading (AIN0) ---
  if (adc_on) {
    adc.uploadConfig();
    adc.startSingleShotConversion();
    delay(10); 

    Status status = adc.readConversionVoltage(voltage); //read adc 
    if (status != Status::Ok) {
      Serial.print("ERROR READING VOLTAGE!!! ");
      Serial.println(static_cast<uint8_t>(status));
      rgbLedWrite(RGB_BUILTIN,50,0,0); // turn on board led RED
    }
  }


  // --- Scaling and Conversion ---
  rawLevel = (voltage - Vmin) * 100 / (Vmax - Vmin);
  smoothLevel = alpha * rawLevel + (1-alpha)*smoothLevel; // exponential smoothing, low-pass filter

  level = round(smoothLevel); // to eliminate delay, simply round(rawLevel)

  volume = level/100.0 * maxGallons;

  if (level > 97) level = 100;
  else if (level <= 1) level = 0;  // clamp it


  // --- LED Indicator Control ---
  //lit_pixels = map(level, 0, 100, 0, NUMPIXELS + 1); // Map 0-100 level to 0-(number of pixels)
  strip.clear();
  for (int j = 0; j < level; j++) {
    strip.setPixelColor(j, strip.Color(0, 0, LUX));
  }
  strip.show();
  delay(10);


  // Send events to server every 0.2 second
  if (millis() - lastTime >= 200) {
    lastTime = millis();

    String voltage_str = String(voltage) + " V";
    String level_str = String(level) + " %" + "   (" + String(voltage) + "V)";
    String volume_str = String(volume) + " gallons";
    String temp_str = String(temp,1) + " °C";
    String smoothTemp_str = String(smoothTemp) +  " °C";
    String humidity_str = String(humidity,1) + " %";
    String dew_str = String(dewPoint, 1) + " °C";
    String signal = String(WiFi.RSSI());
    String ctrl_str = String(ONLINE_CONTROL);

    String time_str;
    if (lastTime < 60000)
      time_str = String(lastTime/1000) + " sec";
    else if (lastTime < 3600000)
      time_str = String(lastTime/60000.0 , 0) + " min";
    else
      time_str = String(lastTime/3600000.0 , 1) + " hours";

    
    // Send Events to the Web Client with the Sensor Readings
    if (WiFi.status() == WL_CONNECTED) {
      String payload = "{";
      payload += "\"ONLINE_CONTROL\":\"" + ctrl_str + "\",";
      payload += "\"level\":\"" + level_str + "\",";
      payload += "\"volume\":\"" + volume_str + "\",";
      payload += "\"temp\":\"" + temp_str + "\",";
      payload += "\"humidity\":\"" + humidity_str + "\",";
      payload += "\"dewPoint\":\"" + dew_str + "\",";
      payload += "\"signal\":\"" + signal + "\",";
      payload += "\"time\":\"" + time_str + "\"";
      payload += "}";

      events.send(payload.c_str(), "payload", millis());
    }
    else {
      Serial.print("Time: " + time_str);
      Serial.print("  Voltage: " + voltage_str);
      Serial.print("  Level: " + level_str);
      Serial.print("  Volume: " + volume_str);
      Serial.print("  Temp: " + temp_str);
       Serial.print("  smoothTemp: " + smoothTemp_str);
      Serial.print("  Humidity: " + humidity_str);
      Serial.println("  Dew Point: " + dew_str);
    }
  }

}
