#ifndef RIDE_CONFIG_H
#define RIDE_CONFIG_H

// Wheel diameter (in inches)
const float WHEEL_DIAMETER = 0.6278;

// Number of poles in the motor
const int POLES = 56;

// Gear ratio
const float GEAR_RATIO = 1.0;

// velocity unit
const bool IS_MPH = false;

// Speedometer update interval (in milliseconds)
const int UPDATE_INTERVAL = 500;

// Battery max amps
const float BATTERY_MAX_AMPS = 700.0;

// RX and TX pins for UART communication
const int RX_PIN = 18;
const int TX_PIN = 17;

#endif // RIDE_CONFIG_H