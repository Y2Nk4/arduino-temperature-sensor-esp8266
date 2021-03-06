#ifndef _STORAGES_h
	#define _STORAGES_h

	#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
	#else
	#include "WProgram.h"
	#endif

    #include "helpers.h"

    template <typename type> boolean loadAndDecodeLocalFile(const String& fileName, type& msgBody, const pb_msgdesc_t* fields) {
        File fileHandle = SPIFFS.open(fileName, "r");
        if (!fileHandle) {
            // config file does not exist
            fileHandle.close();
            return false;
        }
        else {
            Serial.println("Found Local Device Config File");
            size_t fileSize = fileHandle.size();
            uint8_t buffer[fileSize + 1];

            for (unsigned int i = 0; i < fileSize; i++) {
                buffer[i] = fileHandle.read();
            }
            buffer[fileSize] = '\0';
            fileHandle.close();

            // Decode Discover Response
            pb_istream_t is_stream = pb_istream_from_buffer(buffer, fileSize);
            pb_decode(&is_stream, fields, &msgBody);
            return true;
        }
    }

    //uint8_t* loadLocalFile(const String& fileName, size_t* i_fileSize = 0) {
    //    File fileHandle = SPIFFS.open(fileName, "r");
    //    if (!fileHandle) {
    //        // config file does not exist
    //        fileHandle.close();
    //        return 0;
    //    }
    //    else {
    //        Serial.println("Found Local Device Config File");
    //        size_t fileSize = fileHandle.size();

    //        if (i_fileSize != 0)
    //            *i_fileSize = fileSize;

    //        uint8_t buffer[fileSize + 1];

    //        for (unsigned int i = 0; i < fileSize; i++) {
    //            buffer[i] = fileHandle.read();
    //        }
    //        buffer[fileSize] = '\0';
    //        fileHandle.close();

    //        return buffer;
    //    }
    //}

    boolean writeBufferToFile(const String& fileName, const char* buffer, size_t bufferSize = 128) {
        File fileHandle = SPIFFS.open(fileName, "w");
        int bytesWritten = fileHandle.write(buffer, bufferSize);
        fileHandle.close();
        if (bytesWritten == 0) {
            Serial.println("Save File Failed");
            return false;
        }
        return true;
    }

    template <typename type> boolean encodeAndSaveLocalFile(const String& fileName, type* msgBody, const pb_msgdesc_t* fields, size_t bufferSize = 128) {
        uint8_t buffer[bufferSize];
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);
        pb_encode(&stream, fields, msgBody);

        return writeBufferToFile(fileName, (char*)buffer, stream.bytes_written);
    }

#endif