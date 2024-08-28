/*------------------
  Author          : Brorn Munyratanak x Noch Kakada
  Place           : Sen Sok, Phnom Penh
  * Department      : Electronic PPI (B.EcE17-5A14)
  * @Noch Kakada x @Brorn Munyratanak
  * ESP32 Gateway (PlatformIO)

*/
/* Private includes ----------------------------------------------------------*/
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Arduino_ESP32_Updater.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
/*------------------*/
//#include <Ethernet.h> //Function to call RJ45 ethernet port 

/* USER Header END Includes */
/*------------------*/
/*Private define----------------------------------------------------------------*/
#define RXp2 33 // UART pin config GPIO33
#define TXp2 32 // UART pin config GPIO32

typedef struct
{
  float I1;
  float I2;
  float I3;
  float I4;
  float I5;

  float ActivePower1;
  float ActivePower2;
  float ActivePower3;
  float ActivePower4;
  float ActivePower5;

  float energy_value1;
  float energy_value2;
  float energy_value3;
  float energy_value4;
  float energy_value5;

  float Voltage;
  float Freq;
} EnergyPackage;

EnergyPackage received_package;

// Wi-Fi and ThingsBoard configuration
constexpr char WIFI_SSID[] = "BMR";
constexpr char WIFI_PASSWORD[] = "@1234567";
constexpr char TOKEN[] = "rKfyDEr14iQ4oT7ru14y";
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
bool rpcSubscribed = false; // Indicates if RPC subscription is done

// Initialize WiFi and MQTT clients
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// Function declarations
void initWiFi();
bool reconnect();
RPC_Response processSetRELAYStatus(const RPC_Data &data, int RLIndex);
void processTime(const JsonVariantConst &data);
 
// Define the array of RPC callbacks
const std::array<RPC_Callback, 5> callbacks = {
    RPC_Callback{"setRLStatus1", [](const RPC_Data &data)
                 { return processSetRELAYStatus(data, 0); }},
    RPC_Callback{"setRLStatus2", [](const RPC_Data &data)
                 { return processSetRELAYStatus(data, 1); }},
    RPC_Callback{"setRLStatus3", [](const RPC_Data &data)
                 { return processSetRELAYStatus(data, 2); }},
    RPC_Callback{"setRLStatus4", [](const RPC_Data &data)
                 { return processSetRELAYStatus(data, 3); }},
    RPC_Callback{"setRLStatus5", [](const RPC_Data &data)
                 { return processSetRELAYStatus(data, 4); }}};

void sendDataToThingsBoard(float Current1, float Current2, float Current3, float Current4, float Current5, float AP1, float AP2, float AP3,  float AP4,  float AP5, 
float EN1, float EN2, float EN3, float EN4, float EN5, float vols, float freq, float TotalEnergy) {

  String jsonData = "{\"Energy1\":" + String(EN1) + ", \"Energy2\":" + String(EN2) + ", \"Energy3\":" + String(EN3) + 
  ", \"Energy4\":" + String(EN4) + ", \"Energy5\":" + String(EN5) + ", \"voltage\":" + String(vols)+ ", \"freq1\":" + String(freq)+ ", \"TotalEN\":" + String(TotalEnergy)+  
  ", \"CurrentI1\":" + String(Current1)+ ", \"CurrentI2\":" + String(Current2)+ ", \"CurrentI3\":" + String(Current3)+ ", \"CurrentI4\":" + String(Current4)+
  ", \"CurrentI5\":" + String(Current5)+ ", \"ActivePower1\":" + String(AP1)+ ", \"ActivePower2\":" + String(AP2)+ ", \"ActivePower3\":" + String(AP3)+ ", \"ActivePower4\":" + String(AP4)+
  ", \"ActivePower5\":" + String(AP5)+ "}" ;

  tb.sendTelemetryJson(jsonData.c_str());
 
}

void sendStateToSTM32(int RLIndex, bool state) //Relay Control
{
  char message;
  switch (RLIndex)
  {
  case 0:
    message = state ? 'A' : 'B';
    break;
  case 1:
    message = state ? 'C' : 'D';
    break;
  case 2:
    message = state ? 'E' : 'F';
    break;
  case 3:
    message = state ? 'G' : 'H';
    break;
  case 4:
    message = state ? 'I' : 'J';
    break;
  default:
    return; // Invalid RELAY index
  }

  // Send the state over UART to STM32
  Serial1.write(message);
}

void setup()
{
  Serial.begin(115200); 
  Serial1.begin(115200, SERIAL_8N1, RXp2, TXp2);
  //setup_ETH();
  // Small delay to ensure the serial monitor is ready
  delay(500);

  // Initialize Wi-Fi connection
  initWiFi();
}

void loop()
{
  //loop_ETH();
  // Read BL0910 data from STM32 as structured package
  if (Serial1.available() >= sizeof(EnergyPackage))
  {
    Serial1.readBytes((char *)&received_package, sizeof(EnergyPackage));

    Serial.print("Current1: ");
    Serial.println(received_package.I1);
    Serial.print("Current2: ");
    Serial.println(received_package.I2);
    Serial.print("Current3: ");
    Serial.println(received_package.I3);
    Serial.print("Current4: ");
    Serial.println(received_package.I4);
    Serial.print("Current5: ");
    Serial.println(received_package.I5);

    Serial.print("ActivePower1: ");
    Serial.println(received_package.ActivePower1);
    Serial.print("ActivePower2: ");
    Serial.println(received_package.ActivePower2);
    Serial.print("ActivePower3: ");
    Serial.println(received_package.ActivePower3);
    Serial.print("ActivePower4: ");
    Serial.println(received_package.ActivePower4);
    Serial.print("ActivePower5: ");
    Serial.println(received_package.ActivePower5);

    Serial.print("Received energy 1: ");
    Serial.println(received_package.energy_value1);
    Serial.print("Received energy 2: ");
    Serial.println(received_package.energy_value2);
    Serial.print("Received energy 3: ");
    Serial.println(received_package.energy_value3);
    Serial.print("Received energy 4: ");
    Serial.println(received_package.energy_value4);
    Serial.print("Received energy 5: ");
    Serial.println(received_package.energy_value5);

    Serial.print("Received Voltage: ");
    Serial.println(received_package.Voltage);
    Serial.print("Received Freq: ");
    Serial.println(received_package.Freq);
  }
  // Small delay to avoid overwhelming the loop
  delay(1000);

  // Attempt to reconnect if the Wi-Fi connection is lost
  if (!reconnect())
  {
    return;
  }

  // Check if we are connected to ThingsBoard
  if (!tb.connected())
  {
    // Attempt to connect to ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
    {
      Serial.println("Failed to connect");
      return;
    }
  }
  // Subscribe to RPC callbacks if not already subscribed
  if (!rpcSubscribed)
  {
    Serial.println("Subscribing for RPC...");
    if (tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
    {
      rpcSubscribed = true;
      Serial.println("RPC subscription successful");
    }
    else
    {
      Serial.println("Failed to subscribe for RPC");
    }
  }

  // Generate and send random values every 10 seconds
  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead > 10000)
  {
    lastTempRead = millis();
    float Current1= received_package.I1;
    float Current2= received_package.I2;
    float Current3= received_package.I3;
    float Current4= received_package.I4;
    float Current5= received_package.I5;

    float AP1= received_package.ActivePower1;
    float AP2= received_package.ActivePower2;
    float AP3= received_package.ActivePower3;
    float AP4= received_package.ActivePower4;
    float AP5= received_package.ActivePower5;
    
    float EN1 = received_package.energy_value1;
    float EN2 = received_package.energy_value2;
    float EN3 = received_package.energy_value3;
    float EN4 = received_package.energy_value4;
    float EN5 = received_package.energy_value5;

    float vols = received_package.Voltage;
    float freq = received_package.Freq;
    float TotalEnergy = EN1 + EN2 + EN3 +EN4 + EN5;  
   sendDataToThingsBoard(Current1, Current2, Current3, Current4, Current5, AP1, AP2, AP3, AP4, AP5, EN1, EN2, EN3, EN4, EN5, vols, freq, TotalEnergy);
  }

  // Maintain the connection and process incoming messages
  tb.loop();
}
void initWiFi()
{
  // Start the connection to the Wi-Fi network
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

bool reconnect()
{
  // Check the Wi-Fi connection status and reconnect if necessary
  if (WiFi.status() != WL_CONNECTED)
  {
    initWiFi();
  }
  return WiFi.status() == WL_CONNECTED;
}

RPC_Response processSetRELAYStatus(const RPC_Data &data, int RLIndex)
{
  // Process the RPC request to change the LED state
  int dataInt = data;
  if (RLIndex >= 0 && RLIndex < 5)
  {
    bool States = dataInt == 1; // Update the LED state based on the received data
    Serial.print("BreakCircuit ");
    Serial.print(RLIndex + 1);
    Serial.println(States ? " ON" : " OFF");
    // Send the state to STM32 via UART
    sendStateToSTM32(RLIndex, States);
    return RPC_Response("newStatus", dataInt); // Respond with the new status
  }
  return RPC_Response("error", "Invalid LED index");
}

void processTime(const JsonVariantConst &data)
{
  // Process the RPC response containing the current time
  Serial.print("Received time from ThingsBoard: ");
  Serial.println(data["time"].as<String>());
}