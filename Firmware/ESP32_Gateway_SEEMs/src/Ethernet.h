// #include <ETH.h>
// #include <WiFi.h>
// #include <Arduino.h>

// // Define hardware-specific constants (replace these with actual values for your setup)
// #define ETH_ADDR 0
// #define ETH_TYPE ETH_PHY_LAN8720
// #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// #define ETH_RESET_PIN 5
// #define ETH_MDC_PIN 23
// #define ETH_MDIO_PIN 18
// #define ETH_POWER_PIN 12
// #define SD_MISO_PIN 2
// #define SD_MOSI_PIN 15
// #define SD_SCLK_PIN 14
// #define SD_CS_PIN 13

// // Define Ethernet event identifiers if they are not defined in your library
// #ifndef ARDUINO_EVENT_ETH_START
// #define ARDUINO_EVENT_ETH_START SYSTEM_EVENT_ETH_START
// #endif

// #ifndef ARDUINO_EVENT_ETH_CONNECTED
// #define ARDUINO_EVENT_ETH_CONNECTED SYSTEM_EVENT_ETH_CONNECTED
// #endif

// #ifndef ARDUINO_EVENT_ETH_GOT_IP
// #define ARDUINO_EVENT_ETH_GOT_IP SYSTEM_EVENT_ETH_GOT_IP
// #endif

// #ifndef ARDUINO_EVENT_ETH_DISCONNECTED
// #define ARDUINO_EVENT_ETH_DISCONNECTED SYSTEM_EVENT_ETH_DISCONNECTED
// #endif

// #ifndef ARDUINO_EVENT_ETH_STOP
// #define ARDUINO_EVENT_ETH_STOP SYSTEM_EVENT_ETH_STOP
// #endif

// // IPAddress staticIP(192, 168, 1, 25);  // Set the static IP address
// // IPAddress gateway(192, 168, 1, 3);    // Set the gateway IP address
// // IPAddress subnet(255, 255, 255, 192); // Set the subnet mask
// // IPAddress dns(8, 8, 8, 8);            // Set the primary DNS server IP address

// static bool eth_connected = false;

// void WiFiEvent(WiFiEvent_t event)
// {
//   switch (event)
//   {
//   case ARDUINO_EVENT_ETH_START:
//     Serial.println("ETH Started");
//     ETH.setHostname("esp32-ethernet");
//     break;
//   case ARDUINO_EVENT_ETH_CONNECTED:
//     Serial.println("ETH Connected");
//     break;
//   case ARDUINO_EVENT_ETH_GOT_IP:
//     Serial.print("ETH MAC: ");
//     Serial.print(ETH.macAddress());
//     Serial.print(", IPv4: ");
//     Serial.print(ETH.localIP());
//     if (ETH.fullDuplex())
//     {
//       Serial.print(", FULL_DUPLEX");
//     }
//     Serial.print(", ");
//     Serial.print(ETH.linkSpeed());
//     Serial.println("Mbps");
//     eth_connected = true;
//     break;
//   case ARDUINO_EVENT_ETH_DISCONNECTED:
//     Serial.println("ETH Disconnected");
//     eth_connected = false;
//     break;
//   case ARDUINO_EVENT_ETH_STOP:
//     Serial.println("ETH Stopped");
//     eth_connected = false;
//     break;
//   default:
//     break;
//   }
// }
//   void setup_ethernet()
// {
//   Serial.println("Initializing Ethernet...");
//   //ETH.begin();
//   // Set static IP and other network configurations if needed
//   // ETH.config(staticIP, gateway, subnet, dns);
//   Serial.println("Ethernet Initialized");
//   Serial.begin(115200);
//   while (!Serial) ; // Wait for serial port to connect

//   WiFi.onEvent(WiFiEvent);

// #ifdef ETH_POWER_PIN
//   pinMode(ETH_POWER_PIN, OUTPUT);
//   digitalWrite(ETH_POWER_PIN, HIGH);
// #endif

// #if CONFIG_IDF_TARGET_ESP32
//   if (!ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE))
//   {
//     Serial.println("ETH start Failed!");
//     delay(1000);
//     ESP.restart();
//   }
// #else
//   if (!ETH.beginSPI(ETH_MISO_PIN, ETH_MOSI_PIN, ETH_SCLK_PIN, ETH_CS_PIN, ETH_RST_PIN, ETH_INT_PIN))
//   {
//     Serial.println("ETH start Failed!");
//     delay(1000);
//     ESP.restart();
//   }
// #endif

//   // if (!ETH.config(staticIP, gateway, subnet, dns, dns))
//   // {
//   //   Serial.println("Configuration failed.");
//   //   delay(1000);
//   //   ESP.restart();
//   // }
// }
