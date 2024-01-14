
int device_begin(void);

int mydevice_readRegister(uint8_t RegNum, uint8_t *Value);

int mydevice_writeRegister(uint8_t RegNum, uint8_t Value);

int32_t mydevice_readPressure(); // returns Pressure * 100
int32_t mydevice_readTemperature(); // returns Temperature * 100

void mydevice_readCalibrationData(void);
