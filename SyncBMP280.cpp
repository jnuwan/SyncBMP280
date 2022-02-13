#include "SyncBMP280.h"

/** ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  BMP280 RELATED FUNCTIONS
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
SyncBMP280::SyncBMP280(){

}

/*!
 *	@brief Initialize BMP280.
 *	@return void
 */
void SyncBMP280::begin(void){
  Wire.begin();
  Wire.beginTransmission(BMP280_ADDRESS);
  uint8_t error = Wire.endTransmission();
  if(error == 0){
    _bmp280I2CAddress = BMP280_ADDRESS;
  }
  else{
    Wire.beginTransmission(BMP280_ADDRESS_ALT);
    error = Wire.endTransmission();
    if(error == 0){
      _bmp280I2CAddress = BMP280_ADDRESS_ALT;
    }
  }
  readCalibration();
  setSampling();
}

/*!
 *	@brief Once we measure devide readings, we get uncalibrated values.
 *		To correct the readings we need the calibration coefficients.
 *		Hence, we read the these coefficients at initial.
 *		These are stored into the BMP280 during production.
 *	@param void
 *	@return void
 */
void SyncBMP280::readCalibration(void){
  _bmp280Calibration.dig_T1 = read16Register(_bmp280I2CAddress, BMP280_DIG_T1);
  _bmp280Calibration.dig_T2 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_T2);
  _bmp280Calibration.dig_T3 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_T3);

  _bmp280Calibration.dig_P1 = read16Register(_bmp280I2CAddress, BMP280_DIG_P1);
  _bmp280Calibration.dig_P2 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P2);
  _bmp280Calibration.dig_P3 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P3);
  _bmp280Calibration.dig_P4 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P4);
  _bmp280Calibration.dig_P5 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P5);
  _bmp280Calibration.dig_P6 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P6);
  _bmp280Calibration.dig_P7 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P7);
  _bmp280Calibration.dig_P8 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P8);
  _bmp280Calibration.dig_P9 = (int16_t)read16Register(_bmp280I2CAddress, BMP280_DIG_P9);
}

/*!
 *  @brief Sets the sampling config for BMP280
 *  @param mode -Power mode
 *  @param tempSampling -Sampling scheme for temp readings
 *  @param pressSampling -Sampling scheme for pressure readings
 *  @param filter -Filtering mode to apply (if any)
 *  @param duration -Standby duration
 *  @return void
 */
void SyncBMP280::setSampling(bmp280PowerMode mode,
          bmp280Sampling tempSampling, bmp280Sampling pressSampling,
          bmp280Filter filter, bmp280StandbyTime duration){
  if(_bmp280I2CAddress == BMP280_ADDRESS || 
        _bmp280I2CAddress == BMP280_ADDRESS_ALT){
    _measReg.mode = mode;
    _measReg.osrs_t = tempSampling;
    _measReg.osrs_p = pressSampling;

    _configReg.filter = filter;
    _configReg.t_sb = duration;

    writeRegister(_bmp280I2CAddress, BMP280_CONFIG, _configReg.get());
    writeRegister(_bmp280I2CAddress, BMP280_CONTROL, _measReg.get());
  }
  delay(100);
}

/*!
 *	@brief Reads the temperature from BMP280.
 *	@return The temperature in degress celcius.
 *	Ref. Datasheet v.1.19 (Jan 2018) P27 [Table 25], P22 code rev.1.1
 */
float SyncBMP280::readTemperature(){
  if(_bmp280I2CAddress == BMP280_ADDRESS || 
        _bmp280I2CAddress == BMP280_ADDRESS_ALT){
    int32_t adc_T;
		//		= read24Register(_bmp280I2CAddress, BMP280_TEMPDATA);
		//adc_T >>= 4;
    adc_T = (uint32_t)read8Register(_bmp280I2CAddress, BMP280_TEMPDATA_MBS) << 12;
    adc_T |= (uint32_t)read8Register(_bmp280I2CAddress, BMP280_TEMPDATA_LBS) << 4;
    adc_T |= (read8Register(_bmp280I2CAddress, BMP280_TEMPDATA_XLBS) >> 4 )& 0b00001111;
    
    int64_t var1;
    int64_t var2;

    var1 = ((((adc_T>>3) - ((int32_t)_bmp280Calibration.dig_T1<<1))) * 
           ((int32_t)_bmp280Calibration.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)_bmp280Calibration.dig_T1)) * 
           ((adc_T>>4) - ((int32_t)_bmp280Calibration.dig_T1))) >> 12) *
           ((int32_t)_bmp280Calibration.dig_T3)) >> 14;

    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    T = T / 100;
    return T;
  }

  return 0.0;
}

/*!
 *	@brief Reads the atmospheric pressure from BMP280.
 *	@return atmospheric pressure in Pa.
 *	Ref. Datasheet v.1.19 (Jan 2018) P26 [Table 24], P22 code rev.1.1
 */
float SyncBMP280::readPressure(){
  int64_t var1;
	int64_t var2;
	int64_t p;

	if(_bmp280I2CAddress == BMP280_ADDRESS || 
					_bmp280I2CAddress == BMP280_ADDRESS_ALT){
		// Must be done first to get the t_fine variable set up
		readTemperature();

  //int32_t adc_P = read24Register(_bmp280I2CAddress, BMP280_PRESSUREDATA);//read24(BMP280_REGISTER_PRESSUREDATA);
  //adc_P >>= 4;
	
		int32_t adc_P;
		adc_P = (uint32_t)read8Register(_bmp280I2CAddress, BMP280_PRESSURE_MSB) << 12;
		adc_P |= (uint32_t)read8Register(_bmp280I2CAddress, BMP280_PRESSURE_LSB) << 4;
		adc_P |= (read8Register(_bmp280I2CAddress, BMP280_PRESSURE_XLSB) >> 4 )& 0b00001111;
		

		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)_bmp280Calibration.dig_P6;
		var2 = var2 + ((var1 * (int64_t)_bmp280Calibration.dig_P5) << 17);
		var2 = var2 + (((int64_t)_bmp280Calibration.dig_P4) << 35);
		var1 = ((var1 * var1 * (int64_t)_bmp280Calibration.dig_P3) >> 8) +
					 ((var1 * (int64_t)_bmp280Calibration.dig_P2) << 12);
		var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280Calibration.dig_P1) >> 33;

		if (var1 == 0) {
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576 - adc_P;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((int64_t)_bmp280Calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t)_bmp280Calibration.dig_P8) * p) >> 19;

		p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280Calibration.dig_P7) << 4);
		return (float)p / 256;
	}
	return 0.0;
}

/*!
 *	@brief Calculates the approximate altitude using atmospheric pressure and the
 *		supplied sea level hPa as a reference.
 *	@param seaLevelhPa
 *		The current hPa at sea level.
 *	@return The approximate altitude above sea level in meters.
 */
float SyncBMP280::readAltitude(float seaLevelhPa){
  float pressure = readPressure() / 100;
  float altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  return altitude;
}

/** ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  BASIC FUNCTIONS
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/*!
 *	@brief Writes an 8 bit value over I2C
 *	@param I2CAddress -Slave device address
 *		reg -Register address
 *		data -Data
 *	@return value from selected register (one byte)
 */
void SyncBMP280::writeRegister(uint8_t I2CAddress, uint8_t reg, uint8_t data){
	byte buffer[2];
	buffer[1] = data;
	buffer[0] = reg;
  Wire.beginTransmission(I2CAddress);
  Wire.write(buffer, 2);
	Wire.endTransmission();
}

/*!
 *  @brief Reads an 8 bit value over I2C
 *  @param I2CAddress -Slave device address
 *		reg -Register address
 *	@return value from selected register (one byte)
 */
uint8_t SyncBMP280::read8Register(uint8_t I2CAddress, uint8_t reg){
  Wire.beginTransmission(I2CAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2CAddress, 1);   
  uint8_t value = Wire.read();
  return value;
}

/*!
 *	@brief Reads a 16 bit value over I2C
 *	@param I2CAddress -Slave device address
 *		reg -Register address
 *	@return value from selected register (two bytes)
 */
uint16_t SyncBMP280::read16Register(uint8_t I2CAddress, uint8_t reg){
  Wire.beginTransmission(I2CAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2CAddress, 2);   
  uint16_t value = Wire.read();
	value |= Wire.read()<<8;
  return value;

}
/*!
 *  @brief Reads a 16 bit value over I2C
 *  @param I2CAddress -Slave device address
 *		reg -Register address
 *	@return value from selected register (two bytes)
 */
uint32_t SyncBMP280::read24Register(uint8_t I2CAddress, uint8_t reg){
	Wire.beginTransmission(I2CAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(reg, 3);   
  uint32_t value = Wire.read();
	value |= Wire.read()<<8;
	value |= Wire.read()<<16;
  return value;
}
