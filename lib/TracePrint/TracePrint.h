/*
  TracePrint.h Traces execution of Arduino source code
  For all boards that support RTCZero library
  */

#pragma region Version   
  // 1.0.1 2020-01-23 HM    0.1.1 Shared var indent between instances
  // 1.0.2 2020-11-26 HM    0.1.2 Date added to Time
  // 1.0.3 2020-11-26 HM    0.1.2 Date added to Time
  // 1.0.4 2020-11-27 HM    0.1.3 Fixed Arduino library issues
#pragma endregion Version

#ifndef TRACEPRINT_H_
  #define TRACEPRINT_H_

  // Macro to disable TracePrint
  #ifdef TRACEOFF
    #define TRACE(...) 
    #define TRACELINE(...) 
  #else
    #define TRACE(...) trace.printf(__func__ , __VA_ARGS__)
    #define TRACELINE(...) trace.printf(__func__ , __LINE__, __VA_ARGS__)
  #endif

  #define BUFFERSIZE 200

  #include <Arduino.h>
  #include <stdint.h>
  #include <Stream.h>
  #include <RTCZero.h>

  class TracePrint {
    public:
      TracePrint() {
        isOn = true;
        indent = 0;
        rtc.begin();
        rtc.setDate(1, 1, 0);
        rtc.setTime(0, 0, 0);
      };
      ~TracePrint() {};
      void setTraceStream(Stream& stream);
      void printf(const char* functionName, uint16_t lineNumber, const char* c, ...); 
      void printf(const char* functionName, const char* c, ...); 
      void on();
      void off();
      void showDate(bool flag);
      void reset();
    protected:
      Stream* _traceStream;
      RTCZero rtc;
    private:
      static byte indent; // static to share between instances
      boolean isOn;
      boolean isDate ;
  };

#endif //TRACEPRINT_H_