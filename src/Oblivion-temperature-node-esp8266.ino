/*
 Name:		Oblivion_temperature_node_esp8266.ino
 Created:	2020/12/24 3:39:45
 Author:	11476
*/

//#define DEBUG

#include "lib/debug.h"
#include "lib/helpers.h"

#include "user_interface.h"
#include "Esp.h"

#include "DHT.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "protobuf/generated_nanopb/temperature_node.pb.h"
#include "protobuf/generated_nanopb/temperature_node.pb.c"
#include "protobuf/generated_nanopb/device_discover.pb.h"
#include "protobuf/generated_nanopb/device_discover.pb.c"
#include <MQTT.h>
#include <FS.h>

#define DHTPIN 4
#define DHTTYPE DHT22

#define WIFINAME "ASUS-Home_2.4G"  
#define WIFIPW   "asdqwe123" 
#define BROADCASR_PORT 1483
#define UDPDiscoverPort 11213
#define ConfigFilename "./config.dat"
#define MQTT_ID_PREFIX "temp-node-"

WiFiUDP Udp;
IPAddress broadcastIp;
DHT dht(DHTPIN, DHTTYPE);
MQTTClient mqttClient;
WiFiClient net;
bool isRegistered = false;

void messageReceived(MQTTClient* client, char topic[], char buffer[], int payloadLength) {
    LogReceivedMessage(topic, buffer, payloadLength);

    String strTopic = String(topic);
    String wifiMacString = WiFi.macAddress();
    String deviceId = MQTT_ID_PREFIX + wifiMacString;

    if (strTopic == ("oblivion/discover-device_register_response-" + deviceId)) {
        Serial.println("Received Register Response");
        DeviceRegisterResponse registerResp = DeviceRegisterResponse_init_zero;
        pb_istream_t is_stream = pb_istream_from_buffer((uint8_t*)buffer, payloadLength);
        pb_decode(&is_stream, DeviceRegisterResponse_fields, &registerResp);

        Serial.print("Register Response:");
        Serial.println(registerResp.success);

        isRegistered = true;
    }
}

void setup() {
    // put your setup code here, to run once:
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    connectToWifi();
    dht.begin();
    delay(200);

    #ifdef DEBUG
    Serial.println("Debug Mode");
    #endif

    bool isMountSPIFFSSuccess = SPIFFS.begin();
    if (!isMountSPIFFSSuccess) {
        Serial.println("Error occurred while mounting the SPIFFS");
        return;
    }
    DeviceConfig deviceConf = loadLocalDeviceConf();
    // DeviceConfig deviceConf = discoverCenter();

    Serial.println("storage data:");
    Serial.println((unsigned int)deviceConf.mqtt_port);
    Serial.println((unsigned int)deviceConf.mqtt_ip);

    IPAddress mqtt_ip = IPAddress((unsigned int)deviceConf.mqtt_ip);

    Serial.println(mqtt_ip.toString());

    Serial.print("\nconnecting to mqtt...");
    String wifiMacString = WiFi.macAddress();
    String deviceId = MQTT_ID_PREFIX + wifiMacString;
    mqttClient.begin(mqtt_ip, (int)deviceConf.mqtt_port, net);
    mqttClient.onMessageAdvanced(messageReceived);
    while (!mqttClient.connect(deviceId.c_str())) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("Connected to MQTT");
    
    DeviceRegisterRequest regReq = DeviceRegisterRequest_init_zero;

    deviceId.toCharArray(regReq.device_name, 31);
    String device_type_s = "temp-sensor";
    device_type_s.toCharArray(regReq.device_type, 31);

    for (unsigned int i = 0; i < 32; i++) {
        Serial.print(regReq.device_type[i]);
    }

    uint64_t macAddr = getMacAddr();
    regReq.device_mac = macAddr;
    // regReq.device_mac = 152006086161916;

    uint8_t buffer[60];

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, DeviceRegisterRequest_fields, &regReq);

    // Serial.println("raw buffer");
    // for (int i = 0; i < stream.bytes_written; i++) {
    //     Serial.print(buffer[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println("");
    // Serial.println("---");

    size_t size = stream.bytes_written;
    char data[size + 1];
    memcpy(&data, &buffer, size);

    data[size] = '\0';

    mqttClient.subscribe("oblivion/discover-device_register_response-" + deviceId);
    Serial.println("Subscribe to oblivion/discover-device_register_response-" + deviceId);
    bool result = mqttClient.publish("oblivion/discover-device_register", data);
    // Serial.println("Send Result:");
    // Serial.println(result);
    // Serial.println(mqttClient.lastError());

    while (!isRegistered) {
        mqttClient.loop();
        delay(200);
    }

    /*RegisterRequest regReq = {
        device_name: device_name_tmp,
        device_type: device_type,
        device_mac: device_mac_tmp
    };*/
    delay(1000);
    Serial.println("Done!");
    WiFi.disconnect();

    ESP.deepSleep(15e6);
}

DeviceConfig loadLocalDeviceConf() {
    File fileHandle = SPIFFS.open(ConfigFilename, "r");
    if (!fileHandle) {
        // config file does not exist
        fileHandle.close();
        return discoverCenter();
    }
    else {
        Serial.println("Found Local Device Config File");
        size_t fileSize = fileHandle.size();
        uint8_t buffer[fileSize];

        for (unsigned int i = 0; i < fileSize; i++) {
            buffer[i] = fileHandle.read();
        }
        fileHandle.close();

        // Decode Discover Response
        DeviceConfig deviceConf = DeviceConfig_init_zero;
        pb_istream_t is_stream = pb_istream_from_buffer(buffer, fileSize);
        pb_decode(&is_stream, DeviceConfig_fields, &deviceConf);

        if (deviceConf.mqtt_port != 0) {
            return deviceConf;
        }
        else {
            Serial.println("Incorrect Conf, fetching new Conf");
            return discoverCenter();
        }
    }
}

DeviceConfig discoverCenter() {
    Udp.beginPacket(broadcastIp, BROADCASR_PORT);
    DiscoverRequest regReqMsg = DiscoverRequest_init_zero;
    regReqMsg.req_flag = 0xFFC2;
    regReqMsg.discover_port = UDPDiscoverPort;

    uint8_t buffer[128];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, DiscoverRequest_fields, &regReqMsg);

    for (int i = 0; i < stream.bytes_written; i++) {
        Serial.printf("%02X", buffer[i]);
        Udp.write(buffer[i]);
    }

    Udp.endPacket();

    int packetSize = 0;
    Udp.begin(UDPDiscoverPort);
    while (!packetSize) {
        delay(100);
        packetSize = Udp.parsePacket();
    }
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.println(remoteIp);
    int packetLen = Udp.read(buffer, 255);

    // Decode Discover Response
    DiscoverResponse disResMsg = DiscoverResponse_init_zero;
    pb_istream_t is_stream = pb_istream_from_buffer(buffer, packetLen);
    pb_decode(&is_stream, DiscoverResponse_fields, &disResMsg);

    Serial.print("Received");
    Serial.println((unsigned int)disResMsg.sensor_center_mqtt_port);

    DeviceConfig deviceConf = DeviceConfig_init_zero;
    deviceConf.mqtt_ip = uint32_t(Udp.remoteIP());
    deviceConf.mqtt_port = (unsigned int)disResMsg.sensor_center_mqtt_port;
    deviceConf.register_topic = disResMsg.register_topic;

    if ((unsigned int)disResMsg.sensor_center_mqtt_port == 0) {
        Serial.println("Received Incorrect MQTT Port");
        ESP.restart();
        return deviceConf;
    }

    stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, DeviceConfig_fields, &deviceConf);

    File fileHandle = SPIFFS.open(ConfigFilename, "w");
    int bytesWritten = fileHandle.write((char*)buffer, sizeof(buffer));
    if (bytesWritten == 0) {
        Serial.println("Save Config Failed");
        return deviceConf;
    }
    fileHandle.close();

    return deviceConf;
}

boolean connectToWifi() {
    WiFi.disconnect();
    delay(1);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFINAME, WIFIPW);
    Serial.print("Connecting..");

    //uint8_t connect_result;
    //connect_result = WiFi.waitForConnectResult();
    //Serial.println("Connect Result:");
    //Serial.println(connect_result);

    int i = 0;
    boolean state = true;
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        Serial.print(WiFi.localIP().toString());
        Serial.println(WiFi.status());
        i++;

        if (i > 50) {
            state = false;
            break;
        }
    }

    if (state) {
        Serial.println("Connected");
        Serial.println("IP Address:");
        Serial.println(WiFi.localIP());
        broadcastIp = ~uint32_t(WiFi.subnetMask()) | uint32_t(WiFi.gatewayIP());
    }
    else {
        Serial.println("Connection failed.");
    }

    // return connect_result == 1;
    return state;
}

void loop() {
    // put your main code here, to run repeatedly:
  //  float t = dht.readTemperature();
  //  Serial.print(F("%  Temperature: "));
  //  Serial.print(t);
  //  Serial.print(F("��C "));
  //  Serial.println("---");
  //
  //  char t_str[20];
  //  Serial.printf(t_str, "%f", t);
  //
  //  Udp.beginPacket(broadcastIp, 1122);
  //  Serial.printf("Send Buffer");
  //  Udp.write(t_str);
  //  Udp.endPacket(); 
  //  delay(1500);
}

