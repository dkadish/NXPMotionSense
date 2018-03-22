#include "NXPMotionSense.h"
#include "nxp_config.h"
#include "utility/NXPSensorRegisters.h"
#include <util/crc16.h>
#include <elapsedMillis.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

NXPMotionSense::NXPMotionSense()
{
	//Set initial values for private vars
	_wire = &Wire;
    _altWire = &Wire;
}

NXPMotionSense::NXPMotionSense(wire_t * wire)
{
	//Set initial values for private vars
	_wire = wire;
    _altWire = wire;
}
NXPMotionSense::NXPMotionSense(wire_t * wire, wire_t * altWire)
{
    //Set initial values for private vars
    _wire = wire;
    _altWire = altWire;
}

bool NXPMotionSense::begin()
{
	unsigned char buf[NXP_MOTION_CAL_SIZE];
	uint8_t i;
	uint16_t crc;

	//_wire->begin();
	//_wire->setClock(400000);

	memset(accel_mag_raw, 0, sizeof(accel_mag_raw));
	memset(gyro_raw, 0, sizeof(gyro_raw));

	while (!FXOS8700_begin()) {
		Serial.println("config error FXOS8700");
		delay(1000);
	}
	while (!FXAS21002_begin()) {
		Serial.println("config error FXAS21002");
		delay(1000);
	}
#if !defined(NO_ALTIMETER)
	while (!MPL3115_begin()) {
		Serial.println("config error MPL3115");
		delay(1000);
	}
#endif

	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
	}
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, buf[i]);
	}
	if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
		memcpy(cal, buf+2, sizeof(cal));
	} else {
		memset(cal, 0, sizeof(cal));
		cal[9] = 50.0f;
	}
	return true;

}


void NXPMotionSense::update()
{
	static elapsedMillis msec;

	if (FXOS8700_read(accel_mag_raw)) { // accel + mag
	}
#if !defined(NO_ALTIMETER)
	if (MPL3115_read(&altitude_raw, &temperature_raw)) { // alt
	}
#endif
	if (FXAS21002_read(gyro_raw)) {  // gyro
		newdata = 1;
	}
}

static bool write_reg(wire_t * wire, uint8_t i2c, uint8_t addr, uint8_t val)
{
    wire->beginTransmission(i2c);
    wire->write(addr);
    wire->write(val);
    return wire->endTransmission() == 0;
}

static bool read_regs(wire_t * wire, uint8_t i2c, uint8_t addr, uint8_t *data, uint8_t num)
{
    wire->beginTransmission(i2c);
    wire->write(addr);
    if (wire->endTransmission(false) != 0) return false;
    wire->requestFrom(i2c, num);
    if (wire->available() != num) return false;
    while (num > 0) {
        *data++ = wire->read();
        num--;
    }
    return true;
}

static bool read_regs(wire_t * wire, uint8_t i2c, uint8_t *data, uint8_t num)
{
    wire->requestFrom(i2c, num);
    if (wire->available() != num) return false;
    while (num > 0) {
        *data++ = wire->read();
        num--;
    }
    return true;
}

bool NXPMotionSense::FXOS8700_begin()
{
    #if defined(USE_ADAFRUIT_IMU)
    const uint8_t i2c_addr=FXOS8700_I2C_ADDR3;
    #else
    const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
    #endif
	uint8_t b;

	// detect if chip is present
	if (!read_regs(_wire, i2c_addr, FXOS8700_WHO_AM_I, &b, 1)) return false;
	if (b != 0xC7) return false;
	// place into standby mode
	if (!write_reg(_wire, i2c_addr, FXOS8700_CTRL_REG1, 0)) return false;
	// configure magnetometer
	if (!write_reg(_wire, i2c_addr, FXOS8700_M_CTRL_REG1, 0x1F)) return false;
	if (!write_reg(_wire, i2c_addr, FXOS8700_M_CTRL_REG2, 0x20)) return false;
	// configure accelerometer
	if (!write_reg(_wire, i2c_addr, FXOS8700_XYZ_DATA_CFG, 0x01)) return false; // 4G range
	if (!write_reg(_wire, i2c_addr, FXOS8700_CTRL_REG2, 0x02)) return false; // hires
	if (!write_reg(_wire, i2c_addr, FXOS8700_CTRL_REG1, 0x15)) return false; // 100Hz A+M
	return true;
}

bool NXPMotionSense::FXOS8700_read(int16_t *data)  // accel + mag
{
	static elapsedMicros usec_since;
	static int32_t usec_history=5000;
#if defined(USE_ADAFRUIT_IMU)
    const uint8_t i2c_addr=FXOS8700_I2C_ADDR3;
#else
    const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
#endif
    uint8_t buf[13];

	int32_t usec = usec_since;
	if (usec + 100 < usec_history) return false;

	if (!read_regs(_wire, i2c_addr, FXOS8700_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;

	if (!read_regs(_wire, i2c_addr, FXOS8700_OUT_X_MSB, buf+1, 12)) return false;
	//if (!read_regs(i2c_addr, buf, 13)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	data[5] = (int16_t)((buf[11] << 8) | buf[12]);
	return true;
}

bool NXPMotionSense::FXAS21002_begin()
{

    #if defined(USE_ADAFRUIT_IMU)
        const uint8_t i2c_addr=FXAS21002_I2C_ADDR1;
    #else
        const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
    #endif
        uint8_t b;

	if (!read_regs(_wire, i2c_addr, FXAS21002_WHO_AM_I, &b, 1)) return false;
	if (b != 0xD7) return false;

	// place into standby mode
	if (!write_reg(_wire, i2c_addr, FXAS21002_CTRL_REG1, 0)) return false;
	// switch to active mode, 100 Hz output rate
	if (!write_reg(_wire, i2c_addr, FXAS21002_CTRL_REG0, 0x00)) return false;
	if (!write_reg(_wire, i2c_addr, FXAS21002_CTRL_REG1, 0x0E)) return false;

	return true;
}

bool NXPMotionSense::FXAS21002_read(int16_t *data) // gyro
{
	static elapsedMicros usec_since;
	static int32_t usec_history=10000;
#if defined(USE_ADAFRUIT_IMU)
    const uint8_t i2c_addr=FXAS21002_I2C_ADDR1;
#else
    const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
#endif
	uint8_t buf[7];

	int32_t usec = usec_since;
	if (usec + 100 < usec_history) return false;

	if (!read_regs(_wire, i2c_addr, FXAS21002_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;

	if (!read_regs(_wire, i2c_addr, FXAS21002_STATUS, buf, 7)) return false;
	//if (!read_regs(i2c_addr, buf, 7)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	return true;
}

/*TODO It is inclear that this code is actually doing anything.
 * radiohound simply imported SFs libraries into his fork.
 * Might be the way to go.
 */
bool NXPMotionSense::MPL3115_begin() // pressure
{
    const uint8_t i2c_addr=MPL3115_I2C_ADDR;
    uint8_t b;

	if (!read_regs(_altWire, i2c_addr, MPL3115_WHO_AM_I, &b, 1)) return false;
	if (b != 0xC4) return false;

	// place into standby mode
	//if (!write_reg(_altWire, i2c_addr, MPL3115_CTRL_REG1, 0)) return false;

	// switch to active, altimeter mode, 512 ms measurement, polling mode
	if (!write_reg(_altWire, i2c_addr, MPL3115_CTRL_REG1, 0xB9)) return false;
	// enable events
	//if (!write_reg(_altWire, i2c_addr, MPL3115_PT_DATA_CFG, 0x07)) return false;

	return true;
}

bool NXPMotionSense::MPL3115_read(int32_t *altitude, int16_t *temperature)
{
	static elapsedMicros usec_since;
	static int32_t usec_history=980000;
	const uint8_t i2c_addr=MPL3115_I2C_ADDR;
	uint8_t buf[6];

	int32_t usec = usec_since;
	if (usec + 500 < usec_history) return false;

	if (!read_regs(_altWire, i2c_addr, MPL3115_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;

	if (!read_regs(_altWire, i2c_addr, MPL3115_STATUS, buf, 6)) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -1000) diff = -1000;
	else if (diff > 1000) diff = 1000;
	usec_history += diff;

	/*int32_t a = ((uint32_t)buf[1] << 12) | ((uint16_t)buf[2] << 4) | (buf[3] >> 4);
	if (a & 0x00080000) a |= 0xFFF00000;
	*altitude = a;
	 * */

	// The least significant bytes l_altitude and l_temp are 4-bit,
	// fractional values, so you must cast the calulation in (float),
	// shift the value over 4 spots to the right and divide by 16 (since
	// there are 16 values in 4-bits).
	float tempcsb = (buf[3]>>4)/16.0;
	float a = (float)( (buf[1] << 8) | buf[2]) + tempcsb;

	*altitude = (int)(a*1000);
	*temperature = (int16_t)((buf[4] << 8) | buf[5]);

	return true;
}

bool NXPMotionSense::writeCalibration(const void *data)
{
	const uint8_t *p = (const uint8_t *)data;
	uint16_t crc;
	uint8_t i;

	if (p[0] != 117 || p[1] != 84) return false;
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, p[i]);
	}
	if (crc != 0) return false;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
	}
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
	}
	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
	return true;
}

