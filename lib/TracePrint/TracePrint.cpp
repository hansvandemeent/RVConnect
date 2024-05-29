/*
  TracePrint.cpp Traces execution of Arduino source code
  For all boards that support RTCZero library
  */

#include "TracePrint.h"
#include <Arduino.h>
#include <stdarg.h>

byte TracePrint::indent = 0;

void TracePrint::printf(const char* functionName, uint16_t lineNumber ,const char* c, ...) {
  if (isOn) {
    char buffer[BUFFERSIZE];
    va_list args;
    char currentDateTime[26];
    if (isDate) {
      sprintf(currentDateTime, "%02i-%02i-%02i %02i:%02i:%02i ", 
        rtc.getDay(), rtc.getMonth(), rtc.getYear(),
        rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    }
    else {
      sprintf(currentDateTime, "%02i:%02i:%02i ", 
        rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); 
    }
    _traceStream->print(currentDateTime);
    if ((c[0] == '<')  && (c[1] == '-')) this->indent--;
    if ((c[0] == '<') && (c[1] == '<')) this->indent = 0;
    for (byte i = 0; i < this->indent; i++) {
      _traceStream->print(".  ");
    }
    _traceStream->print(functionName);
    _traceStream->print(':');
    _traceStream->print(lineNumber); 
    _traceStream->print(' ');
    va_start (args, c);
    vsnprintf (buffer, BUFFERSIZE, c, args);
    _traceStream->println(buffer);
    va_end (args); 
    if ((c[0] == '-') && (c[1] == '>')) this->indent++;
  }
}

void TracePrint::printf(const char* functionName ,const char* c, ...) {
  if (isOn) {
    
    char buffer[BUFFERSIZE];
    va_list args;
    char currentDateTime[26];
    if (isDate) {
      sprintf(currentDateTime, "%02i-%02i-%02i %02i:%02i:%02i ", 
        rtc.getDay(), rtc.getMonth(), rtc.getYear(),
        rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    }
    else {
      sprintf(currentDateTime, "%02i:%02i:%02i ", 
        rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()); 
    }    
    _traceStream->print(currentDateTime);
    if ((c[0] == '<')  && (c[1] == '-')) this->indent--;
    if ((c[0] == '<') && (c[1] == '<')) this->indent = 0;
    for (byte i = 0; i < this->indent; i++) {
      _traceStream->print(".  ");
    }
    _traceStream->print(functionName); 
    _traceStream->print(' ');
    va_start (args, c);
    vsnprintf (buffer, BUFFERSIZE, c, args);
    _traceStream->println(buffer);
    va_end (args); 
    if ((c[0] == '-') && (c[1] == '>')) this->indent++;
  }
}

void TracePrint::setTraceStream(Stream& stream) {
  this->_traceStream = &stream;
}

void TracePrint::on() {
  this->isOn = true;
}

void TracePrint::off() {
  this->isOn = false;
}

void TracePrint::showDate(bool flag) {
  this->isDate = flag;
}

void TracePrint::reset() {
  this->indent = 0;
}