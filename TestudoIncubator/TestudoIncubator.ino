#include <SPI.h>
#include <Ethernet.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// Define pins for relays
#define FAN_AIR_CIRCULATION_PIN 2
#define FAN_DEHUMIDIFIER_PIN 3
#define HEATER_PIN 4
#define WATER_ATOMIZER_PIN 5
// Define pins for sensors
#define DHT_PIN 6

#define TEMP_SENSOR_PIN A0

// Define LCD pins
#define LCD_RS 7
#define LCD_EN 8
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12
#define ONE_WIRE_BUS 13
#define LIGHT_PIN 14


#define EEPROM_CURRENT_DAY_ADDRESS 0
#define EEPROM_IP_ADDRESS_ADDRESS 4
#define EEPROM_SUBNET_MASK_ADDRESS 24
#define EEPROM_MAC_ADDRESS_ADDRESS 44
#define EEPROM_TARGET_TEMP_ADDRESS 64
#define EEPROM_TARGET_HUMIDITY_ADDRESS 68

const unsigned long BATCH_START_DAY = 0;
const int chipSelect = 10; // SD card CS pin

File dataFile; // File object to write data

// Define web server details
byte macAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ipAddress(192, 168, 1, 177);
IPAddress subnetMask(255, 255, 255, 0);
EthernetServer server(80);

// Define LCD object
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Define DHT sensor object
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHT_PIN, DHTTYPE);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Define variables for PID control
float Kp_temp = 1.0;
float Ki_temp = 0.1;
float Kd_temp = 0.05;
float setpoint_temp = 25.0; // Default target temperature
float integral_temp = 0;
float lastError_temp = 0;

float Kp_humidity = 1.0;
float Ki_humidity = 0.1;
float Kd_humidity = 0.05;
float setpoint_humidity = 50.0; // Default target humidity
float integral_humidity = 0;
float lastError_humidity = 0;

// Other global variables
unsigned long previousMillis = 0;
const long interval = 1000;  // Logging interval in milliseconds
unsigned long previousLogMillis = 0;
const long logInterval = 60000;  // Log data interval in milliseconds
unsigned long startDay = 0;
unsigned long currentDay = 0;
bool counting = false;

void setup() {
  // Initialize LCD
  lcd.begin(16, 2);
  
  // Restore IP address, subnet mask, and MAC address from EEPROM

  EEPROM.get(EEPROM_IP_ADDRESS_ADDRESS, ipAddress);
  EEPROM.get(EEPROM_SUBNET_MASK_ADDRESS, subnetMask);
  EEPROM.get(EEPROM_MAC_ADDRESS_ADDRESS, macAddress);
  
  // Configure Ethernet settings with restored values
  Ethernet.begin(macAddress, ipAddress, subnetMask);

  // Initialize Ethernet and start server
  Ethernet.begin(macAddress, ipAddress, subnetMask);
  server.begin();
  
  // Initialize sensors
  dht.begin();
  pinMode(TEMP_SENSOR_PIN, INPUT);
  
  // Initialize relays
  pinMode(FAN_AIR_CIRCULATION_PIN, OUTPUT);
  pinMode(FAN_DEHUMIDIFIER_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(WATER_ATOMIZER_PIN, OUTPUT);
  
  // Initialize SD card
  if (!SD.begin(10)) {
    lcd.print("SD Card Error");
    return;
  }  

  sensors.begin();
  EEPROM.get(EEPROM_CURRENT_DAY_ADDRESS, currentDay);
  EEPROM.get(EEPROM_TARGET_TEMP_ADDRESS, setpoint_temp);
  EEPROM.get(EEPROM_TARGET_HUMIDITY_ADDRESS, setpoint_humidity);
}

void loop() {
  // Handle web requests
  EthernetClient client = server.available();
  if (client) {
    // Process incoming request
    processRequest(client);
    client.stop();
  }
  
  // Read sensors
  float temperature = readTemperature();
  float humidity = readHumidity();
  
  // Control temperature and humidity
  controlTemperature(temperature);
  controlHumidity(humidity);
  
  // Log data
  logData(temperature, humidity);
  
  // Update LCD display
  updateLCD(temperature, humidity);
  
  // Count days
  if (counting) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      currentDay++;
      EEPROM.put(EEPROM_CURRENT_DAY_ADDRESS, currentDay);
    }
  }
}

void processRequest(EthernetClient client) {
  // Read the HTTP request
  String request = "";
  while (client.available()) {
    char c = client.read();
    request += c;
    // Check for end of HTTP request
    if (c == '\n') {
      break;
    }
  }
  
  // Check if the request contains "GET / HTTP/1.1" indicating homepage request
  if (request.indexOf("GET / HTTP/1.1") != -1) {
    // Send HTTP response with homepage content
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<html><body>");
    client.println("<h1>Reptile Egg Incubator Control Panel</h1>");
    client.println("<h2>Status</h2>");
    // Include current system status, fan status, relay status, temperature, humidity, etc.
    client.println("<p>System Status: Running</p>");
    client.println("<p>Temperature: 25°C</p>");
    client.println("<p>Humidity: 50%</p>");
    client.println("<h2>Actions</h2>");
    client.println("<form method='get'>");
    client.println("<button name='action' value='start'>Start</button>");
    client.println("<button name='action' value='stop'>Stop</button>");
    client.println("<button name='action' value='reset'>Reset</button>");
    client.println("<button name='action' value='update'>Update</button>");
    client.println("<button name='action' value='light_on'>Turn Light On</button>");
    client.println("<button name='action' value='light_off'>Turn Light Off</button>");
    client.println("</form>");
    client.println("<h2>Settings</h2>");
    client.println("<form method='get'>");
    client.println("IP Address: <input type='text' name='ip_address'><br>");
    client.println("Subnet Mask: <input type='text' name='subnet_mask'><br>");
    client.println("MAC Address: <input type='text' name='mac_address'><br>");
    client.println("<button name='action' value='save_settings'>Save Settings</button>");
    client.println("</form>");
    client.println("</body></html>");
  }
  
  // Extract the action from the request (if any)
  String action = "";
  if (request.indexOf("GET /?action=") != -1) {
    int actionIndex = request.indexOf("GET /?action=") + 14;
    int endIndex = request.indexOf(" ", actionIndex);
    action = request.substring(actionIndex, endIndex);
  }
  
  // Perform actions based on the request
  if (action == "start") {
    // Start counting
    counting = true;
  } else if (action == "stop") {
    // Stop counting
    counting = false;
  } else if (action == "reset") {
    // Reset counter
    currentDay = BATCH_START_DAY;
  } else if (action == "update") {
    // Save values and update system parameters
    // Extract parameters from the request (if any)
    String targetTempParam = getValue(request, "target_temp");
    String targetHumidityParam = getValue(request, "target_humidity");
    // Convert parameters to appropriate data types
    float targetTemp = targetTempParam.toFloat();
    float targetHumidity = targetHumidityParam.toFloat();
    
    // Save parameters to EEPROM
    EEPROM.put(EEPROM_TARGET_TEMP_ADDRESS, targetTemp);
    EEPROM.put(EEPROM_TARGET_HUMIDITY_ADDRESS, targetHumidity);
    
    // Send HTTP response indicating update success
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<html><body>");
    client.println("<h1>Parameters Updated Successfully</h1>");
    client.println("</body></html>");
  } else if (action == "light_on") {
    // Turn light on
    digitalWrite(LIGHT_PIN, HIGH);
  } else if (action == "light_off") {
    // Turn light off
    digitalWrite(LIGHT_PIN, LOW);
  } else if (action == "save_settings") {
    // Save IP address, subnet mask, and MAC address to EEPROM
    // Extract parameters from the request (if any)
    String ipAddressParam = getValue(request, "ip_address");
    String subnetMaskParam = getValue(request, "subnet_mask");
    String macAddressParam = getValue(request, "mac_address");
    
    // Convert parameters to appropriate data types
    IPAddress ipAddress;
    if (ipAddress.fromString(ipAddressParam)) {
      EEPROM.put(EEPROM_IP_ADDRESS_ADDRESS, ipAddress);
    }
    
    IPAddress subnetMask;
    if (subnetMask.fromString(subnetMaskParam)) {
      EEPROM.put(EEPROM_SUBNET_MASK_ADDRESS, subnetMask);
    }
    
    byte macAddress[6];
    if (parseMACAddress(macAddressParam, macAddress)) {
      EEPROM.put(EEPROM_MAC_ADDRESS_ADDRESS, macAddress);
    }
    
    // Send HTTP response indicating settings save success
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<html><body>");
    client.println("<h1>Settings Saved Successfully</h1>");
    client.println("</body></html>");
  }else  if (action == "air_circulation" ) {
    // Extract the state parameter from the request
    bool enable = false;
    String enable_value = getValue(request, "enable");
    if (enable_value.indexOf("true") != -1) {
      enable = true;
    }

    // Control the air circulation fan
    controlAirCirculation(enable);

    // Send HTTP response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println();
    client.println("Air circulation fan control successful");
  }
  
  // Send HTTP response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<html><body>");
  client.println("<h1>Action performed: " + action + "</h1>");
  client.println("</body></html>");
}

float readTemperature() {
  // Request temperature from sensor
  sensors.requestTemperatures();
  // Read temperature in Celsius
  float temperature = sensors.getTempCByIndex(0);
  return temperature;
}

float readHumidity() {
  // Read humidity from DHT sensor
  float humidity = dht.readHumidity();
  return humidity;
}

void controlTemperature(float temperature) {
  // Compute the error between current temperature and setpoint
  float error = setpoint_temp - temperature;
  
  // Calculate integral term
  integral_temp += error;
  
  // Calculate derivative term
  float derivative = error - lastError_temp;
  
  // Compute PID control signal
  float pid_temp = (Kp_temp * error) + (Ki_temp * integral_temp) + (Kd_temp * derivative);
  
  // Limit PID output to a reasonable range
  pid_temp = constrain(pid_temp, 0, 255);
  
  // Apply PID control signal to adjust the heater
  if (pid_temp > 0) {
    // Turn on heater
    digitalWrite(HEATER_PIN, HIGH);
  } else {
    // Turn off heater
    digitalWrite(HEATER_PIN, LOW);
  }
  
  // Update last error
  lastError_temp = error;
}

void controlHumidity(float humidity) {
  // Compute the error between current humidity and setpoint
  float error = setpoint_humidity - humidity;
  
  // Calculate integral term
  integral_humidity += error;
  
  // Calculate derivative term
  float derivative = error - lastError_humidity;
  
  // Compute PID control signal
  float pid_humidity = (Kp_humidity * error) + (Ki_humidity * integral_humidity) + (Kd_humidity * derivative);
  
  // Limit PID output to a reasonable range
  pid_humidity = constrain(pid_humidity, 0, 255);
  
  // Apply PID control signal to adjust the water atomizer
  if (pid_humidity > 0) {
    digitalWrite(WATER_ATOMIZER_PIN, HIGH);  // Turn on water atomizer
    digitalWrite(FAN_DEHUMIDIFIER_PIN, LOW); // Turn off dehumidifier fan
   } else if (pid_humidity < 0) {
     // Turn off water atomizer
     digitalWrite(WATER_ATOMIZER_PIN, LOW); // Turn off water atomizer
     digitalWrite(FAN_DEHUMIDIFIER_PIN, HIGH); // Turn on dehumidifier fan
  } else { 
    // Turn off both fans
    digitalWrite(WATER_ATOMIZER_PIN, LOW); // Turn off water atomizer
    digitalWrite(FAN_DEHUMIDIFIER_PIN, LOW); // Turn off dehumidifier fan
  }

  // Update last error
  lastError_humidity = error;
}

void logData(float temperature, float humidity) {
  unsigned long currentLogMillis = millis();
  static unsigned long previousLogMillis = 0;
  const long logInterval = 60000; // Log data interval in milliseconds

  if (currentLogMillis - previousLogMillis >= logInterval) {
    previousLogMillis = currentLogMillis;
    
    // Open file for writing
    dataFile = SD.open("data.txt", FILE_WRITE);
    
    // If the file is available, write data to it
    if (dataFile) {
      dataFile.print("Temperature: ");
      dataFile.print(temperature);
      dataFile.print("°C, Humidity: ");
      dataFile.print(humidity);
      dataFile.println("%");
      
      // Close the file
      dataFile.close();
    } else {
      Serial.println("Error opening file!");
    }
  }
}

void updateLCD(float temperature, float humidity) {
  // Update LCD display with current temperature and humidity
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
}

String getValue(String data, String key) {
  // Extract value associated with the key from the data
  String value = "";
  int keyIndex = data.indexOf(key);
  if (keyIndex != -1) {
    int startIndex = data.indexOf("=", keyIndex) + 1;
    int endIndex = data.indexOf("&", startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
    }
    value = data.substring(startIndex, endIndex);
  }
  return value;
}

// Convert MAC address string to byte array
bool parseMACAddress(String macStr, byte* macArray) {
  int byteIndex = 0;
  int strIndex = 0;
  while (byteIndex < 6) {
    int byteValue = 0;
    // Parse hexadecimal byte from string
    for (int i = 0; i < 2; i++) {
      char c = macStr.charAt(strIndex++);
      if (c >= '0' && c <= '9') {
        byteValue = (byteValue << 4) | (c - '0');
      } else if (c >= 'A' && c <= 'F') {
        byteValue = (byteValue << 4) | (c - 'A' + 10);
      } else if (c >= 'a' && c <= 'f') {
        byteValue = (byteValue << 4) | (c - 'a' + 10);
      } else {
        // Invalid character
        return false;
      }
    }
    macArray[byteIndex++] = byteValue;
    // Skip ':' or '-' separator
    if (byteIndex < 6 && strIndex < macStr.length()) {
      if (macStr.charAt(strIndex) == ':' || macStr.charAt(strIndex) == '-') {
        strIndex++;
      } else {
        // Invalid separator
        return false;
      }
    }
  }
  // Ensure all bytes were parsed and no extra characters are present
  return strIndex == macStr.length();
}

// Function to control the air circulation fan
void controlAirCirculation(bool enable) {
  if (enable) {
    // Turn on the air circulation fan
    digitalWrite(FAN_AIR_CIRCULATION_PIN, HIGH);
  } else {
    // Turn off the air circulation fan
    digitalWrite(FAN_AIR_CIRCULATION_PIN, LOW);
  }
}
