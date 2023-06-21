#include "Wire.h"

#define I2C_DEV_ADDR 0x4c // 7bit addr 0x4c << 1;

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();
  //Write message to the slave
  Wire.beginTransmission(I2C_DEV_ADDR);
  uint8_t error = Wire.endTransmission();
  Serial.printf("endTransmission: %u\n", error);
  
  reset();
  SetMode(3);
  uint8_t id = readRegister8(0x18);
  //Range: 8g
  SetRangeCtrl(0b011);
  //Sampling Rate: 50Hz by default
  SetSampleRate(0x17);
  //Mode: Active
  SetMode(1);
  readRawAccel();
}

void loop() {
  //delay(5000);
  readRawAccel();
}
uint8_t mcube_read_regs(uint8_t chip_select, uint8_t reg, uint8_t * value, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++) {
    value[i] = readRegister8(reg + i);
  }
  return 0;
}
float val = 1.2;
float x2,y2,z2;
void readRawAccel(){
  //{2g, 4g, 8g, 16g, 12g}
  float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
  // 16bit
  float faResolution = 32768.0f;

  byte rawData[6];
  // Read the six raw data registers into data array
  mcube_read_regs(I2C_DEV_ADDR, 0x0d, rawData, 6);
  uint8_t x=((((unsigned short)rawData[1]) << 8) | rawData[0]);
  uint8_t y=((((unsigned short)rawData[3]) << 8) | rawData[2]);
  uint8_t z=((((unsigned short)rawData[5]) << 8) | rawData[4]);
  float x1 = (float) (x) / faResolution * faRange[3];
  float y1 = (float) (y) / faResolution * faRange[3];
  float z1 = (float) (z) / faResolution * faRange[3];
  if(((x2+val) < x1) || ((y2+val) < y1) || ((z2+val) < z1) || ((x2-val) > x1) || ((y2-val) > y1) || ((z2-val) > z1)){
    x2 = x1;
    y2 = y1;
    z2 = z1;
    Serial.printf("x = %.2f y = %.2f z = %.2f \r\n",x1,y1,z1);
  }
}
void reset(void){
  writeRegister8(I2C_DEV_ADDR,0x07, 3);
  delay(10);
  // power-on-reset
  writeRegister8(I2C_DEV_ADDR,0x1c, 0x40);
  delay(50);
  // Disable interrupt
  writeRegister8(I2C_DEV_ADDR,0x06, 0x00);
  delay(10);
  // 1.00x Aanalog Gain
  writeRegister8(I2C_DEV_ADDR,0x2B, 0x00);
  delay(10);
  // DCM disable
  writeRegister8(I2C_DEV_ADDR,0x15, 0x00);
  delay(500);
  }
void SetSampleRate(int sample_rate)
{
  uint8_t value;
  SetMode(3);
  value = readRegister8(8);
  value &= 0b00000000;
  value |= sample_rate;
  writeRegister8(I2C_DEV_ADDR,8, value);
}
void writeRegister8(uint8_t chip_select, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(chip_select); 
  Wire.write(reg);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();
  Serial.printf("endTransmission 8: %u\n", error);
  //Wire.endTransmission();
 
  return;
}
void SetRangeCtrl(int range)
{
  uint8_t value;
  //CfgRange = range;
  SetMode(3);
  value = readRegister8(0x20);
  value &= 0b00000111;
  value |= (range << 4) & 0x70;
  writeRegister8(I2C_DEV_ADDR,0x20, value);
}
uint8_t readRegister8(uint8_t reg)
{
  uint8_t value;
  //value = readRegister8(0x07);
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission();
  //Serial.printf("end Transmission 2: %u\n", error);
  value = Wire.requestFrom(I2C_DEV_ADDR, 1);  
  //Serial.printf("Wire.red: %u\n", value);
  value = Wire.read();
  //Serial.printf("Wire.red: %u\n", value);
  return value;
}

void SetMode(int mode)
{
  uint8_t value;
  //value = readRegister8(0x07);
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(8);
  uint8_t error = Wire.endTransmission();
  Serial.printf("end Transmission 2: %u\n", error);
  value = Wire.requestFrom(I2C_DEV_ADDR, 1);  
  value = Wire.read();
  Serial.printf("Wire.requestFrom(8, 1): %u\n", value);
  Serial.println("readRegister8(MC34X9_REG_MODE);");
  value &= 0b11110000;
  value |= mode;
  Serial.printf("Wire.red: %u\n", value);
  
  writeRegister8(I2C_DEV_ADDR,0x07, value);
}
