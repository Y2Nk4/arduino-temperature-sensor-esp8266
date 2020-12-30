/*
 Name:		Oblivion_temperature_node_esp8266.ino
 Created:	2020/12/24 3:39:45
 Author:	11476
*/

#define DEBUG
#include "lib/debug.h"

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

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "lib/helpers.h"
#include "lib/storage.h"
#include "lib/encoder.h"

Adafruit_BME280 bme; // I2C

#define DHTPIN 2
#define DHTTYPE DHT22

#define WIFINAME "ASUS-Home_2.4G"  
#define WIFIPW   "asdqwe123" 
#define BROADCASR_PORT 1483
#define UDPDiscoverPort 11213
#define ConfigFilename "./config.dat"
#define RunRecordFilename "./record.dat"
#define MQTT_ID_PREFIX "temp-node-"

WiFiUDP Udp;
IPAddress broadcastIp;
DHT dht(DHTPIN, DHTTYPE);
MQTTClient mqttClient;
WiFiClient net;
bool isRegistered = false;
bool isConnectedToWifi = false;
bool isConnectedToMQTT = false;


typedef struct
{
    TemperatureRecord members[16];
    size_t count = 0;

    size_t size() {
        return count;
    }

    size_t addToList(TemperatureRecord& item) {
        members[count] = item;
        count++;
        return count;
    }
}
TemperatureRecordList;
typedef struct
{
    uint32_t run_count = 0;
    TemperatureRecordList temperature_record;
}
ActualRunRecord;

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

    bool isMountSPIFFSSuccess = SPIFFS.begin();
    if (!isMountSPIFFSSuccess) {
        Serial.println("Error occurred while mounting the SPIFFS");
        return;
    }
    dht.begin();

    DeviceConfig deviceConf = loadLocalDeviceConf();
    if (!deviceConf.is_registered) {
        // register device in the SensorCenter
        // if it has not registered yet

        connectToWifi();
        // have not registered yet
        if (!deviceConf.mqtt_ip) {
            deviceConf = discoverCenter();
        }

        // prepare to register
        Serial.println("storage data:");
        Serial.println(deviceConf.mqtt_port);
        Serial.println(deviceConf.mqtt_ip);

        IPAddress mqtt_ip = IPAddress(deviceConf.mqtt_ip);
        Serial.println(mqtt_ip.toString());

        Serial.print("\nconnecting to mqtt...");
        String wifiMacString = WiFi.macAddress();
        String deviceId = MQTT_ID_PREFIX + wifiMacString;

        connectToMQTT(mqtt_ip, deviceConf.mqtt_port, deviceId);

        bool result = registerDevice(deviceConf);
        if (result) {
            deviceConf.is_registered = true;
            saveDeviceConfig(deviceConf);
        }
    }

    /*while (false) {
        float t = dht.readTemperature();
        Serial.print("Temperature: ");
        Serial.println(t);
        delay(2000);
    }*/

    ActualRunRecord runRec = loadLocalRunRecord();
    // Serial.println(runRec.temperature_record_count);

    Serial.print("run_count: ");
    Serial.println(runRec.run_count);
    Serial.print("TmpRec Length: ");
    Serial.println(runRec.temperature_record.size());

    if (runRec.run_count < 5) {
        float t = dht.readTemperature();
        TemperatureRecord tmpRec = TemperatureRecord_init_zero;
        tmpRec.temperature_value = t;
        tmpRec.delta = runRec.run_count;

        runRec.run_count++;
        runRec.temperature_record.addToList(tmpRec);
    }
    else {
        // reset TemperatureRecordList
        Serial.println("Upload and reset");
        runRec.run_count = 0;
        TemperatureRecordList newList;
        runRec.temperature_record = newList;
    }
    saveRunRecord(runRec);

    Serial.print("New TmpRec Length: ");
    Serial.println(runRec.temperature_record.size());

    Serial.println("Done!");
    WiFi.disconnect();

    ESP.deepSleep(15e6);
}

boolean connectToMQTT(const IPAddress brokerIP, const unsigned int brokerPort, const String& deviceId) {
    if (!isConnectedToMQTT) {
        mqttClient.onMessageAdvanced(messageReceived);
        mqttClient.begin(brokerIP, (int)brokerPort, net);
        while (!mqttClient.connect(deviceId.c_str())) {
            Serial.print(".");
            delay(200);
        }
        isConnectedToMQTT = true;
        Serial.println("Connected to MQTT");
        return true;
    }
    else {
        return isConnectedToMQTT;
    }
}

boolean registerDevice(DeviceConfig& deviceConf) {
    DeviceRegisterRequest regReq = DeviceRegisterRequest_init_zero;
    String wifiMacString = WiFi.macAddress();
    String deviceId = MQTT_ID_PREFIX + wifiMacString;

    deviceId.toCharArray(regReq.device_name, 31);
    String device_type_s = "temperature-sensor";
    device_type_s.toCharArray(regReq.device_type, 31);

    for (unsigned int i = 0; i < 32; i++) {
        Serial.print(regReq.device_type[i]);
    }

    uint64_t macAddr = getMacAddr();
    regReq.device_mac = macAddr;

    uint8_t buffer[128];

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, DeviceRegisterRequest_fields, &regReq);

    size_t size = stream.bytes_written;
    char data[size + 1];
    memcpy(&data, &buffer, size);
    data[size] = '\0';

    mqttClient.subscribe("oblivion/discover-device_register_response-" + deviceId);
    Serial.println("Subscribe to oblivion/discover-device_register_response-" + deviceId);
    bool result = mqttClient.publish("oblivion/discover-device_register", data);

    while (!isRegistered) {
        mqttClient.loop();
        delay(200);
    }

    return true;
}

ActualRunRecord loadLocalRunRecord() {
    ActualRunRecord actRunRec;
    
    File fileHandle = SPIFFS.open(RunRecordFilename, "r");
    if (!fileHandle) {
        // config file does not exist
        fileHandle.close();
        return actRunRec;
    }
    else {
        Serial.println("Found Local Device Config File");
        size_t fileSize = fileHandle.size();

        RunRecord runRec = RunRecord_init_zero;
        uint8_t buffer[fileSize + 1];

        for (unsigned int i = 0; i < fileSize; i++) {
            buffer[i] = fileHandle.read();
        }
        buffer[fileSize] = '\0';
        fileHandle.close();

        Serial.println("loaded buffer:");
        for (int i = 0; i < fileSize; i++) {
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println("");

        TemperatureRecordList tmpList;

        runRec.temperature_record.arg = &tmpList;
        runRec.temperature_record.funcs.decode = decode_temperature_record;

        pb_istream_t is_stream = pb_istream_from_buffer(buffer, fileSize);
        pb_decode(&is_stream, RunRecord_fields, &runRec);
        // runRec.temperature_record = tmpRecLists.members;
        // bool result = loadAndDecodeLocalFile(RunRecordFilename, runRec, RunRecord_fields);
        actRunRec.run_count = runRec.run_count;
        actRunRec.temperature_record = tmpList;

        return actRunRec;
    }
}

bool decode_temperature_record(pb_istream_t* istream, const pb_field_t* field, void** arg)
{
    TemperatureRecordList* tmpRecLists = (TemperatureRecordList*)(*arg);

    // decode single instance
    TemperatureRecord tempData = TemperatureRecord_init_zero;
    
    if (!pb_decode(istream, TemperatureRecord_fields, &tempData)) {
        const char* error = PB_GET_ERROR(istream);
        Serial.printf("dncode error: %s", error);
        return false;
    }

    (*tmpRecLists).addToList(tempData);
    return true;
}

bool encode_temperature_record(pb_ostream_t* ostream, const pb_field_t* field, void* const* arg)
{
    TemperatureRecordList tmpRecLists = *(TemperatureRecordList*)*arg;

    // encode all numbers
    for (int i = 0; i < tmpRecLists.size(); i++)
    {
        if (!pb_encode_tag_for_field(ostream, field))
        {
            const char* error = PB_GET_ERROR(ostream);
            Serial.printf("SimpleMessage_encode_numbers error: %s", error);
            return false;
        }

        if (!pb_encode_submessage(ostream, TemperatureRecord_fields, &(tmpRecLists.members[i]))) {
            const char* error = PB_GET_ERROR(ostream);
            Serial.printf("encode error: %s", error);
            return false;
        }
    }

    return true;
}

DeviceConfig loadLocalDeviceConf() {
    DeviceConfig deviceConf = DeviceConfig_init_zero;
    bool result = loadAndDecodeLocalFile(ConfigFilename, deviceConf, DeviceConfig_fields);

    if (result && deviceConf.mqtt_port != 0) {
        return deviceConf;
    }
    else {
        Serial.println("Incorrect Conf, fetching new Conf");
        return deviceConf;
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

    saveDeviceConfig(deviceConf);

    return deviceConf;
}

inline boolean saveDeviceConfig(DeviceConfig& deviceConf) {
    return encodeAndSaveLocalFile(ConfigFilename, &deviceConf, DeviceConfig_fields);
}

inline boolean saveRunRecord(ActualRunRecord& actRunRec) {
    RunRecord runRec = RunRecord_init_zero;
    uint8_t buffer[128];

    runRec.run_count = actRunRec.run_count;
    runRec.temperature_record.arg = &(actRunRec.temperature_record);
    runRec.temperature_record.funcs.encode = encode_temperature_record;
    
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, RunRecord_fields, &runRec);

    Serial.println("data save:");
    for (int i = 0; i < stream.bytes_written; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");

    return writeBufferToFile(RunRecordFilename, (char*)&buffer, stream.bytes_written);
}

boolean connectToWifi() {
    if (!false) {
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
            delay(200);
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
            isConnectedToWifi = true;
        }
        else {
            Serial.println("Connection failed.");
        }

        // return connect_result == 1;
        return state;
    }
    else {
        return isConnectedToWifi;
    }
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

