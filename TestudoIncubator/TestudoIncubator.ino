#include <SPI.h>
#include <Ethernet.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <SD.h>
#include <EEPROM.h>
#include <PID_v1.h>

#define TESTUDO_INCUBATOR_TITLE "Testudo Incubator"

#define ON true
#define OFF false
#define VERSION "1.0b"
#define UNDEFINED_PIN 9999
#define SERIAL_DEBUG true
#define SERVICE_PORT 8080

// Define pins for relays
#define FAN_AIR_CIRCULATION_PIN 2
#define FAN_DEHUMIDIFIER_PIN 5
#define HEATER_PIN 6
#define WATER_ATOMIZER_PIN 7
// Define pins for sensors
#define DHT_PIN 8


// Define LCD pins
#define LCD_RS 20
#define LCD_EN 21
#define LCD_D4 22
#define LCD_D5 23
#define LCD_D6 24
#define LCD_D7 25
#define LIGHT_PIN 14

#define EEPROM_CURRENT_DAY_ADDRESS 0
#define EEPROM_IP_ADDRESS_ADDRESS EEPROM_CURRENT_DAY_ADDRESS + 4          // 4
#define EEPROM_SUBNET_MASK_ADDRESS EEPROM_IP_ADDRESS_ADDRESS + 20         // 24
#define EEPROM_MAC_ADDRESS_ADDRESS EEPROM_SUBNET_MASK_ADDRESS + 20        // 44 
#define EEPROM_TARGET_TEMP_ADDRESS EEPROM_MAC_ADDRESS_ADDRESS + 20        // 64
#define EEPROM_TARGET_HUMIDITY_ADDRESS EEPROM_TARGET_TEMP_ADDRESS + 4     // 68
#define EEPROM_CURRENT_COUNTER_ADDRESS EEPROM_TARGET_HUMIDITY_ADDRESS + 4 // 72
#define EEPROM_RUNNING_STATE EEPROM_CURRENT_COUNTER_ADDRESS + 4
#define EEPROM_ID EEPROM_RUNNING_STATE + 4


#define LOG_INTERVAL 1000

unsigned int id = 0;

const unsigned long BATCH_START_DAY = 0;
const int chipSelect = 10; // SD card CS pin

File dataFile; // File object to write data

// Define web server details
byte macAddress[] = { 0x90, 0xA2, 0xDA, 0x00, 0x52, 0xC7 };
  
IPAddress ipAddress(192, 168, 60, 177);
IPAddress subnetMask(255, 255, 254, 0);

EthernetServer server = EthernetServer(SERVICE_PORT);

EthernetLinkStatus g_LinkStatus;
EthernetLinkStatus g_PreviousLinkStatus;

// Define LCD object
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Define DHT sensor object
#define DHTTYPE DHT22   // DHT 11
DHT dht(DHT_PIN, DHTTYPE);


// Define variables for PID control
double Kp_temp = 1.0;
double Ki_temp = 0.1;
double Kd_temp = 0.05;

double setpoint_temp = 30.0; // Default target temperature
//float integral_temp = 0;
//float lastError_temp = 0;
double temperature = 0.0;
double pid_temp  = 0;

PID g_TemperaturePID(&temperature, &pid_temp, &setpoint_temp, Kp_temp, Ki_temp, Kd_temp, DIRECT);

double Kp_humidity = 1.0;
double Ki_humidity = 0.1;
double Kd_humidity = 0.05;

double setpoint_humidity = 70.0; // Default target humidity
double humidity = 0.0;
double pid_humidity =0;

PID g_HumidityePID(&humidity, &pid_humidity, &setpoint_humidity, Kp_humidity, Ki_humidity, Kd_humidity, DIRECT);

// Other global variables
unsigned long previousMillis = 0;
const long interval = 1000;  // Logging interval in milliseconds
unsigned long previousLogMillis = 0;
const long logInterval = 60000;  // Log data interval in milliseconds
unsigned long startDay = 0;
unsigned long currentDay = 0;
unsigned long g_CurrentCounter = 0;
bool counting = false;
bool g_SDEnabled = false;

//heater state
bool g_Heating = false;

//heater state
bool g_AirCirculation = false;

//dehumidification state
bool g_Dehumidify = false;

//atomizer state
bool g_Atomizer = false;

//light status
bool g_Light = false;

#define ETHERNET_LINK_OFF_MESSAGE "Ethernet link is OFF"
#define ETHERNET_LINK_ON_MESSAGE "Ethernet link is ON"
#define ETHERNET_LINK_UNKNOWN_MESSAGE "Ethernet link is Unknown:  Link status detection is only available with W5200 and W5500."
#define SD_FILE_OPEN_ERROR "Error opening file."
#define SD_INIT_ERROR "SD Card Error."

#if (defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__))
#define SD_SS_PIN 4
#define ETH_SS_PIN 10
#define ETH_MOSI_PIN 11
#define ETH_MISO_PIN 12
#define ETH_SCK_PIN 13
#elif defined (__AVR_ATmega2560__)
#define ETH_MISO_PIN 50
#define ETH_MOSI_PIN 51
#define ETH_SCK_PIN 52
#define ETH_SS_PIN 10
#define SD_SS_PIN 4
#endif

void LogMessage(String message) {
    if (SERIAL_DEBUG)
        Serial.println(message);
}

void BadFault(String message) {
    LogMessage("\nHARDWARE FAULT: "+ message);
    while (true);
}


uint8_t simpleHash() {
  
    int eepromSize = EEPROM.length(); // Get the total number of bytes in the EEPROM

    uint8_t hash = 0;
    for (size_t i = 0; i < eepromSize-1; i++) {
        uint8_t data = EEPROM.read(i);
        hash += data;
    }

    return hash;
}

void CheckPIN(int pin1,int pin2, String Message) 
{
    if (pin1 == pin2 && pin1 != UNDEFINED_PIN)
    {
        BadFault(Message);
    }
}

void CheckChannelPIN(int channel_pin,String channel)
{
    CheckPIN(channel_pin, ETH_MOSI_PIN, channel + " Output PIN conflicted with Ethernet MOSI PIN on " + String(channel_pin));
    CheckPIN(channel_pin, ETH_MISO_PIN, channel + " Output PIN conflicted with Ethernet MISO PIN on " + String(channel_pin));
    CheckPIN(channel_pin, ETH_SCK_PIN,  channel + " Output PIN conflicted with Ethernet SCK PIN on " + String(channel_pin));
    CheckPIN(channel_pin, ETH_SS_PIN,   channel + " Output PIN conflicted with Ethernet SS PIN on " + String(channel_pin));
    CheckPIN(channel_pin, SD_SS_PIN,    channel + " Output PIN conflicted with Ethernet SD SS PIN on " + String(channel_pin));
}


void LogEthernetStatus() {
    switch (g_LinkStatus) {
    case LinkOFF:
        LogMessage(ETHERNET_LINK_OFF_MESSAGE);
        break;
    case Unknown:
        LogMessage(ETHERNET_LINK_UNKNOWN_MESSAGE);
        break;
    case LinkON:
        LogMessage(ETHERNET_LINK_ON_MESSAGE);
        break;
    }
}


void setup_Serial()
{
  Serial.begin (115200);
}

void setup_Arduino()
{
  #if defined (__AVR_ATmega2560__)
    pinMode(53, OUTPUT); // for arduino Mega 2560 in order to have SPI interface working
  #endif

  //layout check
  CheckChannelPIN(FAN_AIR_CIRCULATION_PIN, "FAN AIR CIRCULATION");
  CheckChannelPIN(FAN_DEHUMIDIFIER_PIN, "FAN DEHUMIDIFIER");
  CheckChannelPIN(HEATER_PIN, "HEATER");
  CheckChannelPIN(WATER_ATOMIZER_PIN, "ATOMIZER");
  CheckChannelPIN(DHT_PIN, "DHT");
  CheckChannelPIN(LCD_RS, "LCD RS");
  CheckChannelPIN(LCD_EN, "LCD EN");
  CheckChannelPIN(LCD_D4, "LCD D4");
  CheckChannelPIN(LCD_D5, "LCD D5");
  CheckChannelPIN(LCD_D6, "LCD D6");
  CheckChannelPIN(LCD_D7, "LCD D7");
  CheckChannelPIN(LIGHT_PIN, "LIGHT");

    
  // Initialize relays
  pinMode(FAN_AIR_CIRCULATION_PIN, OUTPUT);
  pinMode(FAN_DEHUMIDIFIER_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(WATER_ATOMIZER_PIN, OUTPUT);
  
 
}

void setup_LCD()
{
  // Initialize LCD
  lcd.begin(16, 2);
}

void setup_Ethernet()
{
// Configure Ethernet settings with restored values
  Ethernet.init(ETH_SS_PIN);  // use pin 10 for Ethernet CS
  Ethernet.begin(macAddress, ipAddress, subnetMask);

  EthernetHardwareStatus status = Ethernet.hardwareStatus();

  switch (status)
  {
    case EthernetNoHardware:
        BadFault("No Ethernet shield detected.");
        break;
    case EthernetW5100:
        LogMessage("Ethernet W5100 shield found");
        break;
    case EthernetW5200:
        LogMessage("Ethernet W5200 shield found");
        break;
    case EthernetW5500:
        LogMessage("Ethernet W5500 shield found");
        break;
  }

    g_LinkStatus = g_PreviousLinkStatus = Ethernet.linkStatus();

    LogEthernetStatus();

    LogMessage("Local IP: " + Ethernet.localIP());

  // Initialize Ethernet and start server
  server.begin();

   // Initialize SD card
  if (!SD.begin(SD_SS_PIN)) 
  {
    LogMessage(SD_INIT_ERROR);
    g_SDEnabled = false;
  } else {
    LogMessage("SD Begin OK");
     g_SDEnabled = true;
  }
}

void setup_DHT()
{
  // Initialize sensors
  dht.begin();
}

void setup_PID()
{
  g_HumidityePID.SetMode(AUTOMATIC);
  g_HumidityePID.SetOutputLimits(-1.0, 1.0);
  g_TemperaturePID.SetMode(AUTOMATIC);
  g_TemperaturePID.SetOutputLimits(-1.0, 1.0);
}

void setup_EEPROM(){
   uint8_t hash = simpleHash();
   uint8_t loaded_hash = getHash();
   
   if (hash ==loaded_hash ){
    LoadSettings();
   }
}

void setup()
{
 

  setup_Serial();
  setup_EEPROM();

  setup_Arduino();
  setup_LCD();
  setup_Ethernet();
  setup_DHT();
  setup_PID();
  setup_Timer();
 


  if (SERIAL_DEBUG)
  {
   Serial.println(currentDay);
   Serial.println(setpoint_temp);
   Serial.println(setpoint_humidity);
  }

  LogMessage("Boot OK " VERSION);
}

void setup_Timer()
{
  noInterrupts();
  
  TCCR3A = 0; // Set entire TCCR3A register to 0
  TCCR3B = 0; // Same for TCCR3B

  TCNT3 = 0; // Initialize counter value to 0

  // Set compare match register for 1hz increments
  OCR3A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)

  // Turn on CTC mode and set prescaler to 1024
  TCCR3B |= (1 << WGM32) | (1 << CS32) | (1 << CS30);
  
  // Enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);

  interrupts();
}

// Action to be taken every second
ISR(TIMER3_COMPA_vect)
{ // ISR for Timer3
  g_CurrentCounter++;
}


void loop_Ethernet()
{
   g_LinkStatus = Ethernet.linkStatus();

    if (g_LinkStatus != g_PreviousLinkStatus) {
        g_PreviousLinkStatus = g_LinkStatus;
        LogEthernetStatus();
    }

    if (g_LinkStatus == LinkOFF)
        return;


  // Handle web requests
  EthernetClient client = server.available();

  if (client) {
    // Process incoming request
    processRequest(client);
    client.stop();
  }
}

void loop_PID()
{
  // Compute PID  
  g_HumidityePID.Compute();
  g_TemperaturePID.Compute();

  // Control temperature and humidity
  controlTemperature();
  controlHumidity();

  bool air_circulation_enabled = g_Heating ||  g_Atomizer || g_Dehumidify;

  controlAirCirculation(air_circulation_enabled);
}


void Off()
{
  controlAirCirculation(false);
  controlHumidity(OFF,OFF);
  controlTemperature(OFF);
}

void loop()
{
  // Read sensors
  temperature = readTemperature();
  humidity = readHumidity();

  if (!counting)
    Off();
  else {
    loop_PID();
  
    // Log data
    logData(temperature, humidity);

    // Count days
    if (counting)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;

        //g_CurrentCounter++;
        SaveRuntimeSettings();
        if ( g_CurrentCounter >= 24*60*60 )
        {
          g_CurrentCounter = 0;
          currentDay++;
          SaveRuntimeSettings();
        }
      }
    }
  }

  loop_Ethernet();

  // Update LCD display
  updateLCD(temperature, humidity);
}

void start()
{
    // Start counting
    if (SERIAL_DEBUG)
      Serial.println("Start counting");
    counting = true;
    SaveRuntimeSettings();
}

void stop(){
// Stop counting
    if (SERIAL_DEBUG)
      Serial.println("Stop counting");
    counting = false;
    SaveRuntimeSettings();
}

void(* reboot) (void) = 0;//declare reset function at address 0


void reset()
{
    if (SERIAL_DEBUG)
      Serial.println("Reset counting");
    currentDay = BATCH_START_DAY;
    g_CurrentCounter = 0;
}


void export_metrics(EthernetClient client){
 if (client)
 {
   String metric = "# HELP " TESTUDO_INCUBATOR_TITLE " Temperature reading\n";
  metric += "sensor_temperature ";
  metric += temperature;
  metric += " {sensor=\"DHT22\", id=\"";
  metric += String(id);
  metric += "\"} \n";

  metric += "# HELP " TESTUDO_INCUBATOR_TITLE " Humidity reading\n";
  metric += "sensor_humidity ";
  metric += humidity;
  metric += " {sensor=\"DHT22\", id=\"";
  metric += String(id);
  metric += "\"} \n";
 }
}



void processRequest(EthernetClient client)
{
  if (client) {
    // Read the HTTP request
    bool new_line =false;
    String line;
    boolean isBody = false; // Flag to check if we're at the body part of the request
    String postData = ""; // To store POST data
    String response ="";
    String request ="";
    String request_header ="";
    String body ="";

    while (client.connected()) {
      if (client.available())
      {
        char c = client.read(); // Read a byte, then store in character c
 
        if (!isBody){
          if (!new_line)
            request += c; // Collect the characters of the POST data
            else
            request_header += c; // Collect the characters of the POST data
        }
        else 
          body+=c;

        if (c == '\n') 
        { // End of a line
          new_line = true;
          if (line == "\r") {
            isBody = true; // After headers, the next line should be the body
            
          }
          line = ""; // Clear the line variable for the next line
        } else {
          line += c; // Collect the line
        }
      } else {
       break;
      }
    }
    

  // Extract the action from the request (if any)
  if (SERIAL_DEBUG){
      Serial.println("Request:");
      Serial.println(request);
      Serial.println("Request Header:");
      Serial.println(request_header);
  }

  // Extract the action from the request (if any)
  if (SERIAL_DEBUG) {
      Serial.println("Body:");
      Serial.println(body);
  }

  String action = "";

  if (request.indexOf("/?action=") != -1 ) {
    int actionIndex = request.indexOf("/?action=") + 9;
    action = request.substring(actionIndex);
    int endIndex = action.indexOf("HTTP/1.1");
    action = action.substring(0,endIndex-1);
  }

  if (SERIAL_DEBUG)
  {
      Serial.print("Action:");
      Serial.println(action);
  }

  // Perform actions based on the request
  if (action == "start") {
    start();
  } else if (action == "export_metrics") {
    export_metrics(client);
    return;
  } else if (action == "stop"){
    stop();
  } else if (action == "reboot"){
    reboot();
  } else if (action == "reset") {
    reset();
  } else if (action == "light_on") {
    g_Light = true;
    // Turn light on
    digitalWrite(LIGHT_PIN, HIGH);
  } else if (action == "light_off") {
    // Turn light off
    g_Light = false;
    digitalWrite(LIGHT_PIN, LOW);
  } else if (action == "save_settings") {
    // Save IP address, subnet mask, and MAC address to EEPROM
    // Extract parameters from the request (if any)
    String idParam= getValue(body, "id");
    String ipAddressParam = getValue(body, "ip_address");
    String subnetMaskParam = getValue(body, "subnet_mask");
    String macAddressParam = getValue(body, "mac_address");
    String setpoint_tempParam= getValue(body, "setpoint_temp");
    String setpoint_humidityParam= getValue(body, "setpoint_humidity");
    String decoded = urlDecode(macAddressParam);

    id = atoi(idParam);
    if (SERIAL_DEBUG)
    {
      Serial.print("ID:");
      Serial.println(id);
      Serial.print("New IP Address:");
      Serial.println(ipAddressParam);
      Serial.print("New Subnet Mask:");
      Serial.println(subnetMaskParam);
      Serial.print("New MAC Address:");
      Serial.println(macAddressParam);
      Serial.print("Decoded MAC Address:");
      Serial.println(decoded);
      Serial.print("New Setpoint Temperature:");
      Serial.println(setpoint_tempParam);
      Serial.print("New Setpoint Humidity:");
      Serial.println(setpoint_humidityParam);
    }

    IPAddress new_ipAddress;

    //ip address
    if (new_ipAddress.fromString(ipAddressParam))
      ipAddress = new_ipAddress;
    else
      response = "ERROR: IP Address is not valid.";
    

    //subnet
    IPAddress new_subnetMask;
    if (new_subnetMask.fromString(subnetMaskParam))
      subnetMask = new_subnetMask;
    else
      response = "ERROR: Subnet mask is not valid.";
    

    //mac address
    byte new_macAddress[6];
    if (parseMACAddress(decoded, new_macAddress))
    {
      memcpy(new_macAddress,macAddress, sizeof(macAddress));
    }


    //setpoint temperature
    if (!tryParseDouble(setpoint_tempParam, setpoint_temp))
      response = "Temperature setpoint is not a number.";

    //setpoint humidity
    if (!tryParseDouble(setpoint_humidityParam, setpoint_humidity))
      response = "Humidity setpoint is not a number.";

    Serial.print(setpoint_humidity);
    SaveSettings();
    Serial.print(setpoint_humidity);
  }else  if (action == "air_circulation" ) {
    // Extract the state parameter from the request
    bool enable = false;
    String enable_value = getValue(request, "enable");
    if (enable_value.indexOf("true") != -1) {
      enable = true;
    }

    // Control the air circulation fan
    controlAirCirculation(enable);
  }


  // Check if the request contains "GET / HTTP/1.1" indicating homepage request
  //if (request.indexOf("GET / HTTP/1.1") != -1) {
    // Send HTTP response with homepage content
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.print("<html><head><style>");
    client.print("body { background-color: #232327; color: #d7d7db; } h1,h2,h3 { color:#00a67d;}");
    client.print(".table { display: table; width: 30%; border-collapse: collapse; } .container { display: flex; }  .row { display: table-row; align-items: center; }  .cell, .head { display: table-cell; border: none; padding: 8px; text-align: left; }");
   
    client.print(".btn { \
            background-color: #333;\
            border: 1px solid #444;\
            color: #fff;\
            padding: 10px 20px;\
            font-size: 16px;\
            cursor: pointer;\
            border-radius: 5px;\
            font-family: Arial, sans-serif;\
            text-transform: uppercase;\
            transition: background-color 0.3s, box-shadow 0.3s, transform 0.2s;\
            outline: none;\
    }\
    .btn:hover, .btn:focus {\
            background-color: #555;\
            box-shadow: 0 8px 16px rgba(0, 0, 0, 0.2);\
        }\
        .btn:active {\
            background-color: #222;\
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);\
            transform: translateY(2px);\
        }\
    ");

    client.print("@keyframes rotate { from { transform: rotate(0deg); } to { transform: rotate(360deg); } } .rotate-continuously { animation: rotate 2s linear infinite; } .off {width: 16px; height: 16px; background-color: red; border: 3px solid black; display: inline-block; }  .on {width: 16px; height: 16px; background-color: green; border: 3px solid black; display: inline-block; } </style></head<body>");
    client.print("<h1>Reptile Egg Incubator Control Panel</h1>");




    client.print("<h2>Actions</h2>");
    client.print("<form id='action_bar' method='get'>");
    client.print("<button class='btn' name='action' value='start'>Start hatching</button>");
    client.print("<button class='btn' name='action' value='stop'>Stop hatching</button>");
    client.print("<button class='btn' name='action' value='reset'>Reset hatching Counter</button>");
    client.print("<button class='btn' name='action' value='light_on'>Turn Light On</button>");
    client.print("<button class='btn' name='action' value='light_off'>Turn Light Off</button>");
    client.print("<button class='btn' name='action' value='reboot'>Reboot Arduino</button>");
    client.print("</form>");


    client.print("<h2>Settings</h2>");
    client.print("<form method='post' action='?action=save_settings'>");
    client.print("<div class=\"table\">");


    client.print("<div class=\"row\"><div class=\"cell\">ID:</div> <div class=\"cell\"><input type='text' name='id' value='");
    client.print(id);
    client.print("'></div></div>");

    client.print("<div class=\"row\"><div class=\"cell\">IP Address:</div> <div class=\"cell\"><input type='text' name='ip_address' value='");
    
    //print ip
    client.print(ipAddress[0]);
    client.print(".");
    client.print(ipAddress[1]);
    client.print(".");
    client.print(ipAddress[2]);
    client.print(".");
    client.print(ipAddress[3]);
    client.print("'></div></div>");

    client.print("<div class=\"row\"><div class=\"cell\">Subnet Mask</div> <div class=\"cell\"><input type='text' name='subnet_mask' value='");
    //print mask
    client.print(subnetMask[0]);
    client.print(".");
    client.print(subnetMask[1]);
    client.print(".");
    client.print(subnetMask[2]);
    client.print(".");
    client.print(subnetMask[3]);
    client.print("'></div></div>");

    client.print("<div class=\"row\"><div class=\"cell\">MAC Address</div> <div class=\"cell\"><input type='text' name='mac_address' value='");
    //print mac
    client.print(mac2String(macAddress));
    client.print("'></div></div>");

    //setpoint_temp
    client.print("<div class=\"row\"><div class=\"cell\">Temperature Setpoint</div> <div class=\"cell\"><input type='text' name='setpoint_temp' value='");
    client.print(setpoint_temp);
    client.print("'></div></div>");

    //setpoint_humidity
    client.print("<div class=\"row\"><div class=\"cell\">Humidity Setpoint</div> <div class=\"cell\"><input type='text' name='setpoint_humidity' value='");
    client.print(setpoint_humidity);
    client.print("'></div></div>");

    client.print("<div class=\"row\"><div class=\"cell\"><button class='btn' name='submit' value='true'>Save Settings</button></div></div>");
    client.print("</form>");

    if (action != "") {
      client.println("<div><h1>Last action performed</h1><p>" + action + "</p></div>");

      if (response != "")
        client.print("<div><h1>Response</h1><p>" + response + "</p></div>");
    }

    client.print("</div>"); //END OF SETTINGS


    client.print("<h2>Status</h2>");
    client.print("<div class=\"table\">");

    // System status
    client.print("<div class=\"row\"><div class=\"cell\">System Status</div>");
    if (counting)
      client.print("<div class=\"cell\">Running</div>");
    else 
      client.print("<div class=\"cell\">Not Running</div>");
    client.print("</div>");

    // Counter
    client.print("<div class=\"row\"><div class=\"cell\">Count interval</div><div class=\"cell\">");
    client.print(interval);
    client.print("</div></div>");

    //Logging
    client.print("<div class=\"row\"><div class=\"cell\">Log interval</div><div class=\"cell\">");
    client.print(logInterval);
    client.print("</div></div>");

    
    //Current Counter
    client.print("<div class=\"row\"><div class=\"cell\">Current counter</div><div class=\"cell\">");
    client.print(g_CurrentCounter);
    client.print("</div></div>");

    //Current Day
    client.print("<div class=\"row\"><div class=\"cell\">Current day</div><div class=\"cell\">");
    client.print(currentDay);
    client.print("</div></div>");

    //Start day
    client.print("<div class=\"row\"><div class=\"cell\">Start day</div><div class=\"cell\">");
    client.print(startDay);
    client.print("</div></div>");

    //Current Temperature
    client.print("<div class=\"row\"><div class=\"cell\">Temperature</div><div class=\"cell\">");
    client.print(temperature);
    client.print("&deg;C");
    client.print("</div></div>");

    // Humidity
    client.print("<div class=\"row\"><div class=\"cell\">Humidity</div><div class=\"cell\">");
    client.print(humidity);
    client.print("&#37;");
    client.print("</div></div>");

  client.print("<div class=\"row\"><div class=\"cell\">SD Logging</div>");
  if (g_SDEnabled)
    client.print("<div class=\"cell\"><div class=\"on\"></div></div>");
  else 
    client.print("<div class=\"cell\"><div class=\"off\"></div></div>");
  client.print("</div>");

  client.print("<div class=\"row\"><div class=\"cell\">Heating</div>");
  if (g_Heating)
     client.print("<div class=\"cell\"><div class=\"on\"></div></div>");
    else 
      client.print("<div class=\"cell\"><div class=\"off\"></div></div>");
  client.print("</div>");


  client.print("<div class=\"row\"><div class=\"cell\">Dehumidify</div>");
  if (g_Dehumidify)
     client.print("<div class=\"cell\"><div class=\"on\"></div></div>");
    else 
      client.print("<div class=\"cell\"><div class=\"off\"></div></div>");
  client.print("</div>");

  client.print("<div class=\"row\"><div class=\"cell\">Atomizer</div>");
  if (g_Atomizer)
     client.print("<div class=\"cell\"><div class=\"on\"></div></div>");
    else 
      client.print("<div class=\"cell\"><div class=\"off\"></div></div>");
  client.print("</div>");


  client.print("<div class=\"row\"><div class=\"cell\">Air Circulation</div>");
    if (g_AirCirculation)
      client.print("<div class=\"cell\"><div class=\"on\"></div></div>");
    else 
      client.print("<div class=\"cell\"><div class=\"off\"></div></div>");
  client.print("</div>");


  client.print("<div class=\"row\"><div class=\"cell\">Light</div>");
  if (g_Light)
    client.print("<div class=\"cell\"><div class=\"on\"></div></div>");
  else 
    client.print("<div class=\"cell\"><div class=\"off\"></div></div>");
  client.print("</div>");


client.print("<h3>Temperature PID</h3>");
client.print("<p>Setpoint:");
client.print(setpoint_temp);
client.print("</p><p>KP:");
client.print(Kp_temp);
client.print("</p><p>KI:");
client.print(Ki_temp);
client.print("</p><p>KD:");
client.print(Kd_temp);
client.print("</p><p>Output:");
client.print(pid_temp);
client.print("</p>");

client.print("<h3>Humidity PID</h3>");
client.print("<p>Setpoint:");
client.print(setpoint_humidity);
client.print("</p><p>KP:");
client.print(Kp_humidity);
client.print("</p><p>KI:");
client.print(Ki_humidity);
client.print("</p><p>KD:");
client.print(Kd_humidity);
client.print("</p><p>Output:");
client.print(pid_humidity);

    client.print("</div>");
  //}
  
  // Send HTTP response
  client.println("</body></html>");

  client.stop(); // Close the connection
}
}


  bool tryParseDouble (String input, double& output)
  {
    input.trim();

    float value = input.toFloat();
    if (value==0 &&  ! (input == "0" || input =="0.0" || input =="0.00" || input =="0.000" || input =="0.0000" || input =="0.00000" ))
        return false;

    output = value;
    return true;
  }

  
  void SaveSettings()
  {
    EEPROM.put(EEPROM_IP_ADDRESS_ADDRESS, ipAddress);
    EEPROM.put(EEPROM_SUBNET_MASK_ADDRESS, subnetMask);
    EEPROM.put(EEPROM_MAC_ADDRESS_ADDRESS, macAddress);
    EEPROM.put(EEPROM_TARGET_TEMP_ADDRESS, setpoint_temp);
    EEPROM.put(EEPROM_TARGET_HUMIDITY_ADDRESS, setpoint_humidity);
    EEPROM.put(EEPROM_ID, id);


    writeHash();
  }

uint8_t getHash()
{
   return EEPROM.read(EEPROM.length()-1);
}

void writeHash()
{
   int eepromSize = EEPROM.length(); // Get the total number of bytes in the EEPROM
   uint8_t hash = simpleHash();
   EEPROM.put(eepromSize-1,hash);
}

void SaveRuntimeSettings()
{
  EEPROM.put(EEPROM_CURRENT_DAY_ADDRESS, currentDay);
  EEPROM.put(EEPROM_CURRENT_COUNTER_ADDRESS, g_CurrentCounter );
  EEPROM.put(EEPROM_RUNNING_STATE, counting );

  writeHash();
}



  void LoadSettings()
  {
    if (SERIAL_DEBUG)
      Serial.println("Loading EEPROM settings");

    EEPROM.get(EEPROM_CURRENT_DAY_ADDRESS, currentDay);
    EEPROM.get(EEPROM_CURRENT_COUNTER_ADDRESS, g_CurrentCounter );
    EEPROM.get(EEPROM_IP_ADDRESS_ADDRESS, ipAddress);
    EEPROM.get(EEPROM_SUBNET_MASK_ADDRESS, subnetMask);
    EEPROM.get(EEPROM_MAC_ADDRESS_ADDRESS, macAddress);
    EEPROM.get(EEPROM_TARGET_TEMP_ADDRESS, setpoint_temp);
    EEPROM.get(EEPROM_TARGET_HUMIDITY_ADDRESS, setpoint_humidity);
    EEPROM.get(EEPROM_RUNNING_STATE, counting);
    EEPROM.get(EEPROM_ID, id);
  }
  
float readTemperature() {
  // Read temperature in Celsius
  float temperature = dht.readTemperature();
  return temperature;
}

float readHumidity() {
  // Read humidity from DHT sensor
  float humidity = dht.readHumidity();
  return humidity;
}
 
void controlTemperature() {
  // Apply PID control signal to adjust the heater
  if (pid_temp > 0) {
    controlTemperature(ON);
  } else {
    controlTemperature(OFF);
  }
}

void controlTemperature(bool enabled){
  g_Heating = enabled;
  if (enabled)
    digitalWrite(HEATER_PIN, HIGH);
    else
     digitalWrite(HEATER_PIN, LOW);
}

void controlHumidity(bool dehumidify,bool atomizer)
{
  g_Atomizer = atomizer;
  g_Dehumidify = dehumidify;

  if (g_Atomizer)
  {
    digitalWrite(WATER_ATOMIZER_PIN, HIGH);  // Turn on water atomizer
  }
   else {
       digitalWrite(WATER_ATOMIZER_PIN, LOW); // Turn off water atomizer
   }

    if (g_Dehumidify){
        digitalWrite(FAN_DEHUMIDIFIER_PIN, HIGH); // Turn off dehumidifier fan
    } else{
      digitalWrite(FAN_DEHUMIDIFIER_PIN, LOW); // Turn off dehumidifier fan
    }
}

void controlHumidity() {
  // Apply PID control signal to adjust the water atomizer
  if (pid_humidity > 0) {
    controlHumidity(OFF,ON);
   } else if (pid_humidity < 0) {
    controlHumidity(ON,OFF);
  } else { 
      controlHumidity(OFF,OFF);
  }
}

void logData(float temperature, float humidity) {
  
  unsigned long currentLogMillis = millis();
  static unsigned long previousLogMillis = 0;
  const long logInterval = LOG_INTERVAL; // Log data interval in milliseconds

  if (currentLogMillis - previousLogMillis >= logInterval) {
    previousLogMillis = currentLogMillis;

    if (g_SDEnabled){
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
      
      LogMessage(SD_FILE_OPEN_ERROR);
    }
    }

    if (SERIAL_DEBUG){

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");

    Serial.print("PID Temperature: ");
    Serial.println(pid_temp);

    
    Serial.print("PID Humidity: ");
    Serial.println(pid_humidity);

      if (pid_temp > 0) {
      LogMessage("Heater ON");
      
    } else {
      LogMessage("Heater OFF");
    }

      
  // Apply PID control signal to adjust the water atomizer
  if (pid_humidity > 0) {
    LogMessage("Atomizer ON");
   } else if (pid_humidity < 0) {
     // Turn off water atomizer
    LogMessage("Atomizer OFF");
  } else { 
    // Turn off both fans
    LogMessage("Atomizer OFF");
  }
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

String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]); // J-M-L: slight modification, added the 0 in the format for padding 
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
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

String urlDecode(String encodedString) {
    String decodedString = "";
    char tempChar;
    int num;

    for (int i = 0; i < encodedString.length(); i++) {
      if (encodedString[i] == '%') {
            // Extract the next two characters after '%'
            String hexStr = encodedString.substring(i + 1, i + 3);
            // Convert the hexadecimal string to an integer
            num = strtol(hexStr.c_str(), NULL, 16);
            // Cast the integer to a char and append to the decoded string
            decodedString += char(num);
            // Skip the next two characters in the loop
            i += 2;
        } else if (encodedString[i] == '+') {
            // Convert '+' to space (optional, depending on the context)
            decodedString += ' ';
        } else {
            // Append the character as it is
            decodedString += encodedString[i];
        }
    }
    return decodedString;
}


// Function to control the air circulation fan
void controlAirCirculation(bool enable) {
  g_AirCirculation = enable;
  if (g_AirCirculation) {
    // Turn on the air circulation fan
    digitalWrite(FAN_AIR_CIRCULATION_PIN, HIGH);
  } else {
    // Turn off the air circulation fan
    digitalWrite(FAN_AIR_CIRCULATION_PIN, LOW);
  }
}
