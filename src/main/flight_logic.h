#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include <Arduino.h> // only need to include this in the header file

// Custom Files
#include "data_formatter.h"
#include "data_handler.h"
#include "initialization.h"

// FLIGHT FUNCTIONS
void canSatFlightLogic();

// Variable Declarations
extern bool newState;
extern float apogee;

#endif