#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"


#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"

#include "io/serial_msp.h"

#include "Print.h"



uint8_t checksum;


void serializeString(const char* msg)
{
    for (uint8_t i = 0; i < strlen(msg); i++) {
        serialize8Debug(msg[i]);
        checksum ^= (msg[i]);
    }
}

void debugPrint(const char* msg)
{
    // if(isDebugPrintEnabled){
    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');
    serialize8Debug(strlen(msg));
    checksum ^= strlen(msg);
    serializeString(msg);
    serialize8Debug(checksum);
    // }
}

void debugPrint(const char* msg, double number, uint8_t digits)
{
    // if(isDebugPrintEnabled){
    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');

    if (isnan(number)) {
        const char* err = "not a number";
        uint8_t data_size = strlen(msg) + strlen(err);
        serialize8Debug(data_size);
        checksum ^= data_size;
        serializeString(msg);
        serialize8Debug('\t');
        checksum ^= '\t';
        serializeString(err);
        serialize8Debug(checksum);
        return;
    }
    if (isinf(number)) {
        const char* err = "infinite";
        uint8_t data_size = (uint8_t) (strlen(msg) + strlen(err));
        serialize8Debug(data_size);
        checksum ^= data_size;
        serializeString(msg);
        serialize8Debug('\t');
        checksum ^= '\t';
        serializeString(err);
        serialize8Debug(checksum);
        return;
    }
    if (number > (double) 4294967040.0 || number < (double) -4294967040.0) {

        const char* err = "overflow";
        uint8_t data_size = strlen(msg) + strlen(err) + 1;
        serialize8Debug(data_size);
        checksum ^= data_size;
        serializeString(msg);
        serialize8Debug('\t');
        checksum ^= '\t';
        serializeString(err);
        serialize8Debug(checksum);
        return;
    }

    bool isNumNeg = false;
    if (number < (double) 0.0) {
        number = -number;
        isNumNeg = true;
    }

    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
        rounding /= (double) 10.0;
    number += rounding;

    uint32_t int_part = (uint32_t) number;
    double remainder = number - (double) int_part;

    char buf[8 * sizeof(int_part) + 1];
    char *str = &buf[sizeof(buf) - 1];
    *str = '\0';
    do {
        char digit = int_part % 10;
        int_part /= 10;
        *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;
    } while (int_part);

    uint8_t data_size = (uint8_t) strlen(msg) + (uint8_t) strlen(str) + (isNumNeg ? 1 : 0) + (
            digits > 0 ? (digits + 1) : 0) + 1;
    serialize8Debug(data_size);
    checksum ^= data_size;
    serializeString(msg);
    serialize8Debug('\t');
    checksum ^= '\t';

    if (isNumNeg) {
        serialize8Debug('-');
        checksum ^= '-';
    }

    serializeString(str);

    if (digits > 0) {
        serialize8Debug('.');
        checksum ^= '.';
    }
    while (digits-- > 0) {
        remainder *= (double) 10.0;
        uint8_t toPrint = (uint8_t) (remainder);
        toPrint = toPrint < 10 ? toPrint + '0' : toPrint + 'A' - 10;
        serialize8Debug(toPrint);
        checksum ^= toPrint;
        remainder -= toPrint;
    }
    serialize8Debug(checksum);
    //  }
}

void debugPrint(const char* msg, int number)
{

    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');

    bool isNumNeg = false;
    if (number < 0) {
        number = -number;
        isNumNeg = true;
    }
    char buf[8 * sizeof(number) + 1];
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';
    do {
        char digit = number % 10;
        number /= 10;
        *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;
    } while (number);

    uint8_t data_size = (uint8_t) strlen(msg) + (uint8_t) strlen(str) + (isNumNeg ? 1 : 0) + 1;
    serialize8Debug(data_size);
    checksum ^= data_size;

    serializeString(msg);
    serialize8Debug('\t');
    checksum ^= '\t';

    if (isNumNeg) {
        serialize8Debug('-');
        checksum ^= '-';
    }
    serializeString(str);
    serialize8Debug(checksum);

}



void Print_P::monitor(const char* msg)
  {



          debugPrint(msg);

  }


void Print_P::monitor(const char* tag, int number)
     {


               debugPrint(tag, number);

       }

       void Print_P::monitor(const char* tag, double number, uint8_t precision)
       {

    	       precision=constrain(precision, 0, 7);
               debugPrint(tag, number, precision);

       }




      void Print_P::redGraph(double value, uint8_t precision)
      {
    	  precision=constrain(precision, 0, 7);
    	  debugPrint("~R", value, precision);

      }

      void Print_P::blueGraph(double value, uint8_t precision)

      {
    	  precision=constrain(precision, 0, 7);
    	  debugPrint("~B", value, precision);


      }

      void Print_P::greenGraph(double value, uint8_t precision)

      {

    	  precision=constrain(precision, 0, 7);
    	  debugPrint("~G", value, precision);

      }

      Print_P Print;
