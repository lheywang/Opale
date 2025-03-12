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

    /**
     * @brief   Begin the sensor operation mode. Automatically called on construction of the class.
     *
     * @return  None
     */
    void begin();

    /**
     * @brief   Read the raw temperature. For a fully compensated one, call getTemperature.
     *
     * @return  uint32_t    Raw register value
     */
    uint32_t getRawTemperature();

    /**
     * @brief   Read the compensated temperature with the second order approx.
     *
     * @return  double      Temperature value
     */
    double getTemperature();

    /**
     * @brief   Get the Raw Pressure. For a fully compensated one, call getPressure
     *
     * @return  uint32_t    Raw register value
     */
    uint32_t getRawPressure();

    /**
     * @brief   Read the compensated pressure (and the temperture, as a byproduct)
     *
     * @return  double*
     * @retval  [0] :   Pressure in mbar
     * @retval  [1] :   Temperature in degree C
     */
    double *getPressure();

    /**
     * @brief   Read the calibration of the sensor. Called once on the start.
     *
     * @return  None
     */
    void readCalibration();

    /**
     * @brief   Return the calibration stored on the class.
     *
     * @param   Buf     Output buffer of 6 element longs.
     */
    void getCalibration(uint16_t *);

    /**
     * @brief   Send a command on the I2C bus to the sensor.
     *
     * @param   CMD     The command to be sent
     */
    void sendCommand(uint8_t);

    /**
     * @brief   Read n bytes from the sensor
     *
     * @param   CMD     The command sent to trigger the read
     * @param   WLEN    The lenght of the command in bytes
     * @param   RLEN    The lenght of the read in bytes
     *
     * @return  The rode buffer concatenaned into a single integer or -1 if RLEN > 4
     */
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

    struct i2c_dt_spec dev;
};

#endif
