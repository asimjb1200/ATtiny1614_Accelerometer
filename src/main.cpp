#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>

#define ACCELEROMETER_ADDR 0x18
#define SCL_PIN PIN_PB0 // PIN 9
#define SDA_PIN PIN_PB1 // PIN 8
#define WHO_AM_I_REGISTER_ACCEL 0x0F
#define INT1_SRC_REGISTER 0x31

volatile uint8_t lastPortAIntFlags = 0;


typedef enum {
    RESTING, 
    FALL_DETECTED,
    DEVICE_RECOVERED,
    CHARGING
} MCU_State_t;

volatile MCU_State_t mcu_state = RESTING;

uint8_t sendDataToRegister(uint8_t deviceAddress, uint8_t deviceRegister, uint8_t command) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(deviceRegister);
  Wire.write(command);
  return Wire.endTransmission();
}

bool initFreeFallDetection() {
  uint8_t status = 0;

  // turn on the sensor, enable X,Y, and Z. ODR = 100Hz
  uint8_t ctrl_reg1 = 0x20;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg1, 0x57);
  if (status != 0) { Serial.print("CTRL_REG1 failed, status: "); Serial.println(status); return false; }

  // High-pass filter disabled
  uint8_t ctrl_reg2 = 0x21;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg2, 0x00);
  if (status != 0) { Serial.print("CTRL_REG2 failed, status: "); Serial.println(status); return false; }

  // Interrupt activity 1 driven to INT1 pin
  uint8_t ctrl_reg3 = 0x22;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg3, 0x40);
  if (status != 0) { Serial.print("CTRL_REG3 failed, status: "); Serial.println(status); return false; }

  // FS = ±2 G
  uint8_t ctrl_reg4 = 0x23;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg4, 0x00);
  if (status != 0) { Serial.print("CTRL_REG4 failed, status: "); Serial.println(status); return false; }

  // interrupt 1 pin latched
  uint8_t ctrl_reg5 = 0x24;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg5, 0x08);
  if (status != 0) { Serial.print("CTRL_REG5 failed, status: "); Serial.println(status); return false; }

  // set free-fall threshold = 350mg
  uint8_t int1_ths_reg = 0x32;
  status = sendDataToRegister(ACCELEROMETER_ADDR, int1_ths_reg, 0x16);
  if (status != 0) { Serial.print("INT1_THS failed, status: "); Serial.println(status); return false; }

  // set minimum event duration
  uint8_t int1_duration_reg = 0x33;
  status = sendDataToRegister(ACCELEROMETER_ADDR, int1_duration_reg, 0x03);
  if (status != 0) { Serial.print("INT1_DURATION failed, status: "); Serial.println(status); return false; }

  // configure free-fall recognition
  uint8_t int1_cfg_reg = 0x30;
  status = sendDataToRegister(ACCELEROMETER_ADDR, int1_cfg_reg, 0x95);
  if (status != 0) { Serial.print("INT1_CFG failed, status: "); Serial.println(status); return false; }

  Serial.println("Free fall detection initialized successfully");
  Serial.flush();
  return true;
}

void initAccelInterruptPin() {
  // set pin 2 as input, which is PA4
  PORTA.DIRCLR = PIN4_bm;

  //PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  PORTA.PIN4CTRL = PORT_ISC_FALLING_gc;
}

bool verifyAccelConnection() {
  Wire.beginTransmission(ACCELEROMETER_ADDR);
  Wire.write(WHO_AM_I_REGISTER_ACCEL);
  uint8_t status = Wire.endTransmission(false);

  if (status == 0) {
    Serial.println("Slave sent an ACK");
    Wire.requestFrom(ACCELEROMETER_ADDR, 1);
    if (Wire.available()) {
      uint8_t data = Wire.read();
      Serial.print("WHO_AM_I: 0x");
      Serial.println(data, HEX);
      return true;
    }
  } else if (status == 2) {
    Serial.println("Received NACK on transmit of address");
  } else if (status == 3) {
    Serial.println("Received NACK on transmit of data");
  } else {
    Serial.println("4: Line busy, etc.");
  }
  Serial.flush();
  return false;
}

void initMCUClock() 
{
  // disable prescaler
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0 << CLKCTRL_PEN_bp;

  // Set the clock to use 20MHz
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc;

  // give time for clock to switch if necessary
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm){}
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t deviceRegister) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(deviceRegister);
    Wire.endTransmission(false); // repeated start
    Wire.requestFrom(deviceAddress, 1);
    return Wire.read();
}

void scanBusForDevices() {
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at: 0x");
      Serial.println(addr, HEX);
    }
  }
}

void setup() {
  initMCUClock();

  Serial.begin(115200);

  initAccelInterruptPin();

  sei();
  
  delay(10000);
  
  uint8_t deviceID = SIGROW_DEVICEID0;
  uint8_t serialNum = SIGROW_SERNUM0;
  
  Serial.print("Device ID: 0x"); Serial.println(deviceID, HEX);

  Serial.print("Serial Num: 0x"); Serial.println(serialNum, HEX);

  Serial.flush();

  Wire.begin();

  uint8_t okayToSetup = verifyAccelConnection();

  if (okayToSetup) {
    uint8_t accelIntSuccess = initFreeFallDetection();
  }

  // select which sleep mode to enter and enable the sleep controller
  set_sleep_mode(SLEEP_MODE_STANDBY);
}

void loop() {
  Serial.println("Sleep Mode");
  Serial.flush();
  sleep_mode();
  
  switch (mcu_state)
  {
    case RESTING:
      Serial.println("Resting State");
      Serial.flush();
      break;

    case FALL_DETECTED:
      Serial.println("Fall Detected");
      Serial.flush();
      // clear the interrupt
      readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);
      mcu_state = RESTING;
      break;

    default:
      break;
  }

}

/** 
 * interrupts from accelerometer will be coming in on this port due to
 * the int pin from the device being connected to a port b pin
 * */ 
ISR(PORTA_PORT_vect) {

  PORTA.INTFLAGS |= PIN4_bm;

  mcu_state = FALL_DETECTED;
}