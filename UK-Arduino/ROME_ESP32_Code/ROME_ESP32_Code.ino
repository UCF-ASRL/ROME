#include <WiFi.h>
 
// =====================================================
// Network Configuration
// =====================================================
const char* AP_SSID = "ROME_ESP32";
const char* AP_PASS = "asrl_elgohary";
 
static constexpr uint16_t TCP_PORT = 3333;
 
WiFiServer server(TCP_PORT);
WiFiClient client;
 
// =====================================================
// Hardware Serial Pins
// =====================================================
// Ground Vehicle (Arduino Mega) - Standard GPIOs
#define GV_RX_PIN 25
#define GV_TX_PIN 26
 
// Arm (Teensy 4.1) - Hardware Serial 2
#define ARM_RX_PIN 16
#define ARM_TX_PIN 17
 
HardwareSerial GVSerial(1);
HardwareSerial ARMSerial(2);
 
static constexpr uint32_t BAUD = 115200;
 
// Fixed char buffer for incoming TCP stream to prevent heap fragmentation
static char tcpRxBuf[256];
static size_t tcpRxIdx = 0;
 
// Line buffers for forwarding telemetry safely
String gvBuffer = "";
String armBuffer = "";
 
void setup()
{
    Serial.begin(115200);
 
    // Initialize Hardware Serials
    GVSerial.begin(BAUD, SERIAL_8N1, GV_RX_PIN, GV_TX_PIN);
    ARMSerial.begin(BAUD, SERIAL_8N1, ARM_RX_PIN, ARM_TX_PIN);
 
    // Access Point Setup
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
 
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
 
    server.begin();
    server.setNoDelay(true);
 
    Serial.println("ROME ESP32 Unified Router Ready.");
}
 
void loop()
{
    handleClientConnection();
    handleTCPInput();
    forwardTelemetry();
}
 
void handleClientConnection()
{
    if (!client || !client.connected())
    {
        WiFiClient newClient = server.available();
        if (newClient)
        {
            if (client)
            {
                client.stop();
            }
 
            client = newClient;
            client.setNoDelay(true);
 
            Serial.println("TCP Client Connected.");
            client.println("#CONNECTED");
 
            flushUARTs();
        }
    }
}
 
void flushUARTs()
{
    while (GVSerial.available())  { GVSerial.read(); }
    while (ARMSerial.available()) { ARMSerial.read(); }
}
 
void handleTCPInput()
{
    if (!client || !client.connected()) return;
 
    while (client.available())
    {
        char c = client.read();
 
        if (c == '\r') continue;
 
        if (c == '\n')
        {
            tcpRxBuf[tcpRxIdx] = '\0';
            processCommand(String(tcpRxBuf));
            tcpRxIdx = 0; // Reset buffer index
        }
        else if (tcpRxIdx < sizeof(tcpRxBuf) - 1)
        {
            tcpRxBuf[tcpRxIdx++] = c;
        }
    }
}
 
void processCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;
 
    // Ground Vehicle Handshake
    if (cmd == "START_GV")
    {
        GVSerial.println("Y");
        Serial.println("[CMD] START_GV -> Mega");
        return;
    }
 
    // Arm System Handshake & Commands
    if (cmd == "START_ARM" || cmd == "CAL_ARM" || cmd == "STATUS" || cmd.startsWith("HOME"))
    {
        ARMSerial.println(cmd);
        Serial.print("[CMD] Arm Forward: ");
        Serial.println(cmd);
        return;
    }
 
    // System Emergency Stop
    if (cmd == "STOP_ALL")
    {
        GVSerial.println("!");
        ARMSerial.println("STOP");
        Serial.println("[CMD] EMERGENCY STOP_ALL BROADCAST");
        return;
    }
 
    // Combined ROME Packet Transmission
    if (cmd.startsWith("ROME,"))
    {
        routeROMECommand(cmd);
        return;
    }
 
    // Fallback passthrough for standalone GV wheel RPM vectors (w1,w2,w3,w4)
    if (cmd.indexOf(',') != -1 && !cmd.startsWith("ROME"))
    {
        GVSerial.println(cmd);
        return;
    }
 
    Serial.print("Unknown Command: ");
    Serial.println(cmd);
}
 
void routeROMECommand(String packet)
{
    // Remove "ROME," prefix
    packet.remove(0, 5);
 
    float vals[10];
    int idx = 0;
    char temp[256];
    packet.toCharArray(temp, sizeof(temp));
 
    char* token = strtok(temp, ",");
    while (token != nullptr && idx < 10)
    {
        vals[idx++] = atof(token);
        token = strtok(nullptr, ",");
    }
 
    if (idx != 10)
    {
        Serial.println("Error: Invalid ROME packet size");
        return;
    }
 
    // Route Ground Vehicle Wheel RPMs
    char gvPacket[128];
    snprintf(gvPacket, sizeof(gvPacket), "%.3f,%.3f,%.3f,%.3f\n",
             vals[0], vals[1], vals[2], vals[3]);
    GVSerial.print(gvPacket);
 
    // Route 6-DOF Arm Joint Angles
    char armPacket[256];
    snprintf(armPacket, sizeof(armPacket), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             vals[4], vals[5], vals[6], vals[7], vals[8], vals[9]);
    ARMSerial.print(armPacket);
}
 
void forwardTelemetry()
{
    if (!client || !client.connected()) return;
 
    // Line-buffered forwarding for Ground Vehicle
    while (GVSerial.available())
    {
        char c = GVSerial.read();
        gvBuffer += c;
        if (c == '\n')
        {
            client.print(gvBuffer);
            gvBuffer = "";
        }
    }
 
    // Line-buffered forwarding for Arm
    while (ARMSerial.available())
    {
        char c = ARMSerial.read();
        armBuffer += c;
        if (c == '\n')
        {
            client.print(armBuffer);
            armBuffer = "";
        }
    }
}