/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "PWMServoDriver_t3.h"

// Set to true to print some debug messages, or false to disable them.
#define ENABLE_DEBUG_OUTPUT false

PWMServoDriverT3::PWMServoDriverT3(uint8_t addr, uint8_t bus) {
  _i2caddr = addr;
  _bus = bus;
  _pullups = I2C_PULLUP_EXT;
    #if defined(__MK20DX128__) // Teensy 3.0
        _pins = I2C_PINS_18_19;
        _bus = 0;
    #endif

    #if defined(__MK20DX256__) // Teensy 3.1/3.2
        if(_bus == 1) {
            _pins = I2C_PINS_29_30;
        }
        else{
            _pins = I2C_PINS_18_19;
            _bus = 0;
        }

    #endif

    #if defined(__MK64FX512__) // Teensy 3.5
        if(_bus == 2) {
            _pins = I2C_PINS_3_4;
        }
        else if(_bus == 1) {
            _pins = I2C_PINS_37_38;
        }
        else{
            _pins = I2C_PINS_18_19;
            _bus = 0;
        }

    #endif

    #if defined(__MK66FX1M0__) // Teensy 3.6
        if(_bus == 3) {
            _pins = I2C_PINS_56_57;
        }
        else if(_bus == 2) {
            _pins = I2C_PINS_3_4;
        }
        else if(_bus == 1) {
            _pins = I2C_PINS_37_38;
        }
        else{
            _pins = I2C_PINS_18_19;
            _bus = 0;
        }

    #endif

    #if defined(__MKL26Z64__) // Teensy LC
        if(_bus == 1) {
            _pins = I2C_PINS_22_23;
        }
        else{
            _pins = I2C_PINS_18_19;
            _bus = 0;
        }

    #endif
}

void PWMServoDriverT3::begin(void) {
    i2c_t3(_bus).begin(I2C_MASTER, 0, _pins, _pullups, _i2cRate);
    reset();
}


void PWMServoDriverT3::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}

void PWMServoDriverT3::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
  }
  uint8_t prescale = floor(prescaleval + 0.5);
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Final pre-scale: "); Serial.println(prescale);
  }
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void PWMServoDriverT3::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  // WIRE.beginTransmission(_i2caddr);
  // WIRE.write(LED0_ON_L+4*num);
  // WIRE.write(on);
  // WIRE.write(on>>8);
  // WIRE.write(off);
  // WIRE.write(off>>8);
  // WIRE.endTransmission();
  i2c_t3(_bus).beginTransmission(_i2caddr);
  i2c_t3(_bus).write(LED0_ON_L+4*num);
  i2c_t3(_bus).write(on);
  i2c_t3(_bus).write(on>>8);
  i2c_t3(_bus).write(off);
  i2c_t3(_bus).write(off>>8);
  i2c_t3(_bus).endTransmission();
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void PWMServoDriverT3::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

uint8_t PWMServoDriverT3::read8(uint8_t addr) {
  i2c_t3(_bus).beginTransmission(_i2caddr);
  i2c_t3(_bus).write(addr);
  i2c_t3(_bus).endTransmission();

  i2c_t3(_bus).requestFrom(_i2caddr, (uint8_t)1); // specify the number of bytes to receive
//  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return i2c_t3(_bus).readByte();
}

void PWMServoDriverT3::write8(uint8_t addr, uint8_t d) {
  // WIRE.beginTransmission(_i2caddr);
  // WIRE.write(addr);
  // WIRE.write(d);
  // WIRE.endTransmission();
    i2c_t3(_bus).beginTransmission(_i2caddr); // open the device
    i2c_t3(_bus).write(addr); // write the register address
    i2c_t3(_bus).write(d); // write the data
    i2c_t3(_bus).endTransmission();
}
