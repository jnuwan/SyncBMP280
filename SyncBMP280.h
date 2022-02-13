#ifndef SyncBMP280_h
#define SyncBMP280_h

#include "Arduino.h"
#include "Wire.h"

/** The default I2C address for BMP280 */
#define BMP280_ADDRESS (0x77) 
#define BMP280_ADDRESS_ALT (0x76)

/** Calibration register details for BMP280 data.
 *  Ref. Datasheet v.1.19 (Jan 2018) P21 [Table 17]
*/
typedef enum bmp280Registers {
  BMP280_CHIPID = 0xD0,
  BMP280_VERSION = 0xD1, //not used
  BMP280_SOFTRESET = 0xE0,
  BMP280_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_STATUS = 0xF3,
  BMP280_CONTROL = 0xF4,
  BMP280_CONFIG = 0xF5,
  BMP280_TEMPDATA_MBS = 0xFA,
  BMP280_TEMPDATA_LBS = 0xFB,
  BMP280_TEMPDATA_XLBS = 0xFC,
  //BMP280_PRESSUREDATA = 0xF7,
	BMP280_PRESSURE_MSB = 0xF7,
	BMP280_PRESSURE_LSB = 0xF8,
	BMP280_PRESSURE_XLSB = 0xF9,
  ///Trimming parameter
  BMP280_DIG_T1 = 0x88,
  BMP280_DIG_T2 = 0x8A,
  BMP280_DIG_T3 = 0x8C,
  BMP280_DIG_P1 = 0x8E,
  BMP280_DIG_P2 = 0x90,
  BMP280_DIG_P3 = 0x92,
  BMP280_DIG_P4 = 0x94,
  BMP280_DIG_P5 = 0x96,
  BMP280_DIG_P6 = 0x98,
  BMP280_DIG_P7 = 0x9A,
  BMP280_DIG_P8 = 0x9C,
  BMP280_DIG_P9 = 0x9E
};


typedef struct bmp280Calibration {
  uint16_t dig_T1; ///< dig_T1 cal register.
  int16_t dig_T2;  ///< dig_T2 cal register.
  int16_t dig_T3;  ///< dig_T3 cal register.

  uint16_t dig_P1; ///< dig_P1 cal register.
  int16_t dig_P2;  ///< dig_P2 cal register.
  int16_t dig_P3;  ///< dig_P3 cal register.
  int16_t dig_P4;  ///< dig_P4 cal register.
  int16_t dig_P5;  ///< dig_P5 cal register.
  int16_t dig_P6;  ///< dig_P6 cal register.
  int16_t dig_P7;  ///< dig_P7 cal register.
  int16_t dig_P8;  ///< dig_P8 cal register.
  int16_t dig_P9;  ///< dig_P9 cal register.
};

class SyncBMP280{

  private:

    /** Encapsulates the config register for BMP280 data.
     *  Ref. Datasheet v.1.19 (Jan 2018) P26 [Table 23]
    */
    struct config {
      /** Initialize to power-on-reset state */
      config() : t_sb(STANDBY_MS_1), filter(FILTER_OFF), none(0), spi3w_en(0){}
      unsigned int t_sb : 3;//<standby time in normal mode (3 bits)
      unsigned int filter : 3;//<Filter settings (3bits)
      unsigned int none : 1;//<Unused - don't set
      unsigned int spi3w_en : 1;//<Enables 3-wire SPI
      /** Used to retrieve the assembled config register's byte value. */
      unsigned int get(){ return (t_sb << 5) | (filter << 2) | spi3w_en; }
    };

    /** Encapsulates the ctrl_meas register for BMP280 data.
     *  Ref. Datasheet v.1.19 (Jan 2018) P25 [Table 20]
    */
    struct ctrl_meas {
      /** Initialize to power-on-reset state */
      ctrl_meas() : osrs_t(SAMPLING_NONE), osrs_p(SAMPLING_NONE), mode(MODE_SLEEP) {}
      unsigned int osrs_t : 3;//<Temperature oversampling
      unsigned int osrs_p : 3;//<Pressure oversampling
      unsigned int mode : 2;//<Power mode
      /** Used to retrieve the assembled ctrl_meas register's byte value. */
      unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
    };

    void writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t data);
    uint8_t read8Register(uint8_t I2CAddress, uint8_t reg);
    uint16_t read16Register(uint8_t I2CAddress, uint8_t reg);
    uint32_t read24Register(uint8_t I2CAddress, uint8_t reg);

    uint8_t _bmp280I2CAddress;
    void readCalibration(void);
    bmp280Calibration _bmp280Calibration;
    config _configReg;
    ctrl_meas _measReg;
		int32_t t_fine;

  public:

    /** Filtering level for BMP280 data.
     *  Ref. Datasheet v.1.19 (Jan 2018) P14 [Table 6]
    */
    enum bmp280Filter {
      FILTER_OFF = 0x00,//< Filter off
      FILTER_X2 = 0x01,//<2x filtering
      FILTER_X4 = 0x02,//<4x filtering
      FILTER_X8 = 0x03,//<8x filtering
      FILTER_X16 = 0x04//<16x filtering
    };

    /** Power modes for BMP280.
     *  Ref. Datasheet v.1.19 (Jan 2018) P15 [Table 10]
    */
    enum bmp280PowerMode {
      MODE_SLEEP = 0x00,//<Sleep mode
      MODE_FORCED = 0x01,//<Forced mode
      MODE_NORMAL = 0x03//<Normal mode
      //MODE_SOFT_RESET_CODE = 0xB6//<Software reset
    };

    /** Standby duration in ms for BMP280.
     *  Ref. Datasheet v.1.19 (Jan 2018) P17 [Table 11]
    */
    enum bmp280StandbyTime {
      STANDBY_MS_1 = 0x00,//<0.5 ms standby
      STANDBY_MS_63 = 0x01,//<62.5 ms standby
      STANDBY_MS_125 = 0x02,//<125 ms standby
      STANDBY_MS_250 = 0x03,//<250 ms standby
      STANDBY_MS_500 = 0x04,//<500 ms standby
      STANDBY_MS_1000 = 0x05,//<1000 ms standby
      STANDBY_MS_2000 = 0x06,//<2000 ms standby
      STANDBY_MS_4000 = 0x07//<4000 ms standby
    };

    /** Pressure oversampling rate for BMP280.
     *  Ref. Datasheet v.1.19 (Jan 2018) P25 [Table 21]
    */
    enum bmp280Sampling {
      SAMPLING_NONE = 0x00,//<Skipped over-sampling
      SAMPLING_X1 = 0x01,//<1x over-sampling
      SAMPLING_X2 = 0x02,//<2x over-sampling
      SAMPLING_X4 = 0x03,//<4x over-sampling
      SAMPLING_X8 = 0x04,//<8x over-sampling
      SAMPLING_X16 = 0x05//<16x over-sampling
    };

    SyncBMP280();
    void begin(void);
    void setSampling(bmp280PowerMode mode = MODE_NORMAL,
                   bmp280Sampling tempSampling = SAMPLING_X16,
                   bmp280Sampling pressSampling = SAMPLING_X16,
                   bmp280Filter filter = FILTER_OFF,
                   bmp280StandbyTime duration = STANDBY_MS_1);
    float readTemperature();
		float readPressure();
		float readAltitude(float seaLevelhPa = 1013.25);
};
#endif
