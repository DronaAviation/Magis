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

#include "API-Utils.h"
#include "Utils.h"

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

    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');
    serialize8Debug(strlen(msg));
    checksum ^= strlen(msg);
    serializeString(msg);
    serialize8Debug(checksum);

}

void debugPrint(const char* msg, double number, uint8_t digits)
{

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
        uint8_t data_size = (uint8_t)(strlen(msg) + strlen(err));
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

    uint8_t data_size = (uint8_t) strlen(msg) + (uint8_t) strlen(str)
            + (isNumNeg ? 1 : 0) + (digits > 0 ? (digits + 1) : 0) + 1;
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
        uint8_t toPrint = (uint8_t)(remainder);
        toPrint = toPrint < 10 ? toPrint + '0' : toPrint + 'A' - 10;
        serialize8Debug(toPrint);
        checksum ^= toPrint;
        remainder -= toPrint;
    }
    serialize8Debug(checksum);

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

    uint8_t data_size = (uint8_t) strlen(msg) + (uint8_t) strlen(str)
            + (isNumNeg ? 1 : 0) + 1;
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

//###########################################//

void LED_P::set(led_e LED, led_state_e STATE)
{

    ledOperator(LED, STATE);

}

void LED_P::flightStatus(flightstatus_state_e STATE)
{

    switch (STATE) {

    case DEACTIVATE:

        FlightStatusEnabled = false;
        ledOperator(RED, OFF);
        ledOperator(GREEN, OFF);
        ledOperator(BLUE, OFF);

        break;

    case ACTIVATE:

        FlightStatusEnabled = true;

        break;
    }

}

void Graph_P::red(double value, uint8_t precision)
{

    precision = constrain(precision, 0, 7);
    debugPrint("~R", value, precision);

}

void Graph_P::green(double value, uint8_t precision)
{

    precision = constrain(precision, 0, 7);
    debugPrint("~G", value, precision);

}

void Graph_P::blue(double value, uint8_t precision)
{

    precision = constrain(precision, 0, 7);
    debugPrint("~B", value, precision);

}

bool Interval::set(uint32_t time, bool repeat)
{

	this->repeat=repeat;

    if (this->time == 0) {

        this->time = constrain(time, 1, 5000);
        this->loopTime = millis() + this->time;
    }

    if ((int32_t)(millis() - this->loopTime) >= 0) {
        if (this->repeat)
            loopTime = millis() + this->time;

        return true;

    }

    return false;

}

void Interval::reset(void)
{

    this->time = 0;
    this->loopTime = 0;

}


bool Interval::check()
{

    if ((int32_t)(millis() - this->loopTime) >= 0) {

        if(this->repeat)
            loopTime = millis() + this->time;

        return true;

    }

    return false;

}


void Monitor_P::print(const char* msg)
{

    debugPrint(msg);

}

void Monitor_P::print(const char* tag, int number)
{

    debugPrint(tag, number);

}

void Monitor_P::print(const char* tag, double number, uint8_t precision)
{

    precision = constrain(precision, 0, 7);
    debugPrint(tag, number, precision);

}

void Monitor_P::println(const char* msg)
{

    debugPrint(msg);
    debugPrint("\n");

}

void Monitor_P::println(const char* tag, int number)
{

    debugPrint(tag, number);
    debugPrint("\n");

}

void Monitor_P::println(const char* tag, double number, uint8_t precision)
{

    precision = constrain(precision, 0, 7);
    debugPrint(tag, number, precision);
    debugPrint("\n");

}

LED_P LED;
Graph_P Graph;
Monitor_P Monitor;
