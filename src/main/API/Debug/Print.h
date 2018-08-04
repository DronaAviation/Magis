/*
 Pluto V3R API V.0.6
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

class Print_P {

public:






	void monitor(const char* tag, int number);

	void monitor(const char* tag, double number, uint8_t precision);

	void monitor(const char* msg);

	void redGraph(double value, uint8_t precision=4);

    void greenGraph(double value, uint8_t precision=4);

	void blueGraph(double value, uint8_t precision=4);


};








extern Print_P Print;

#ifdef __cplusplus
}
#endif
