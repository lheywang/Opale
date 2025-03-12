/*
ms5611.h
Library for barometric pressure sensor MS5611-01BA on I2C with arduino

by Petr Gronat@2014
*/

#include <cstdint>
#include <zephyr/drivers/i2c.h>

// Include guard token - prevents to include header file twice
#ifndef MS5611_h
#define MS5611_h // create token

// Removed arduino libs, and replaced with our proper devices.
#define N_PROM_PARAMS 6

// address of the device MS5611
#define ADD_MS5611 0x77 // can be 0x76 if CSB pin is connected to GND

class MS5611
{
public:
    /*
     * CONSTRUCTORS AND DESTRUCTORS
     */
    MS5611();  // constructor
    ~MS5611(); // destructor

    /*
     * FUNCTIONS
     */
    void begin();
    uint32_t getRawTemperature();
    double getTemperature();
    uint32_t getRawPressure();
    double *getPressure();
    void readCalibration();
    void getCalibration(uint16_t *);
    void sendCommand(uint8_t);
    uint32_t readnBytes(const uint8_t *, uint8_t, uint8_t);

private:
    void reset();

    // variables
    int32_t _P;
    int32_t _T;
    int32_t _dT;
    int64_t _Off2;
    int64_t _Sens2;
    uint16_t _C[N_PROM_PARAMS];
    uint32_t _lastTime;

    i2c_dt_spec *dev;
};

#endif
