//
//
//

#ifndef _DEBUG_h
    #define _DEBUG_h
    #ifndef DEBUG

        #define DEC 10
        #define HEX 16
        #define OCT 8
        #define BIN 2

        #define Serial ActualSerial

        static class {
        public:
            void begin(unsigned long, uint8_t) {}
            // void print() {}

            size_t printf(const char* format, ...)  __attribute__((format(printf, 2, 3))) {}
            size_t printf_P(PGM_P format, ...) __attribute__((format(printf, 2, 3))) {}
            size_t print(const __FlashStringHelper*) {}
            size_t print(const String&) {}
            size_t print(const char[]) {}
            size_t print(char) {}
            size_t print(unsigned char, int = DEC) {}
            size_t print(int, int = DEC) {}
            size_t print(unsigned int, int = DEC) {}
            size_t print(long, int = DEC) {}
            size_t print(unsigned long, int = DEC) {}
            size_t print(double, int = 2) {}
            size_t print(const Printable&) {}

            size_t println(const __FlashStringHelper*) {}
            size_t println(const String& s) {}
            size_t println(const char[]) {}
            size_t println(char) {}
            size_t println(unsigned char, int = DEC) {}
            size_t println(int, int = DEC) {}
            size_t println(unsigned int, int = DEC) {}
            size_t println(long, int = DEC) {}
            size_t println(unsigned long, int = DEC) {}
            size_t println(double, int = 2) {}
            size_t println(const Printable&) {}
            size_t println(void) {}
        } Serial;
    #else
        #define DebugSerial Serial;
    #endif
#endif