// helpers.h

#include <ESP8266WiFi.h>

#ifndef _HELPERS_h
#define _HELPERS_h

	#if defined(ARDUINO) && ARDUINO >= 100
		#include "arduino.h"
	#else
		#include "WProgram.h"
	#endif

        void LogReceivedMessage(char topic[], char buffer[], int payloadLength) {
            Serial.println("Received MQTT Message");
            Serial.println(topic);

            Serial.println("Received Register Response Length");
            Serial.println(payloadLength);
            for (unsigned int i = 0; i < payloadLength; i++) {
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println("");
        }

        void print_uint64_t(uint64_t num) {
            char rev[128];
            char* p = rev + 1;

            while (num > 0) {
                *p++ = '0' + (num % 10);
                num /= 10;
            }
            p--;
            /*Print the number which is now in reverse*/
            while (p > rev) {
                Serial.print(*p--);
            }
            Serial.println();
        }

        uint64_t getMacAddr() {
            uint64_t addr = 0;
            byte mac[6];
            WiFi.macAddress(mac);

            for (unsigned int i = 0; i < 6; i++) {
                *((byte*)&addr + i) = mac[i];
            }

            Serial.println("Mac:");
            print_uint64_t(addr);
            return addr;
        }

        template<class T, size_t N>
        constexpr size_t arrLength(T(&)[N]) { return N; }

#endif