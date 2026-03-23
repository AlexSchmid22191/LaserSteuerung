#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>

// Prevent the ArduinoRS485 library from defining the default pins, since pin 2 (default DE) is used for the software enable
#define CUSTOM_RS485_DEFAULT_DE_PIN -1
#define CUSTOM_RS485_DEFAULT_RE_PIN -1

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Adafruit_MAX31856.h>

// Pin definitions
const uint8_t sw_enable = 2;
const uint8_t aiming_beam_enable = 3;
const uint8_t tc_fault_bypass = 4;
const uint8_t hw_enable_readback = 6;
const uint8_t MAX_DRDY = 7;
const uint8_t MAX_FAULT = 8;
const uint8_t pwr_control_pin = 9;
const uint8_t MAX_CS = 10;

// Used for the monitor photo diode readout via transimpedance amplifier expansion board
const uint8_t photodiode_readout = A1;

Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(MAX_CS);

// Register definitions
enum Register
{
  reg_working_process_variable,
  reg_target_setpoint,
  reg_manual_power,
  reg_working_power,
  reg_working_setpoint,
  reg_rate,
  reg_control_mode,
  reg_pid_p,
  reg_pid_i,
  reg_pid_d,
  reg_software_enable,
  reg_tc_type,
  reg_tc_error,
  reg_aiming_beam,
  reg_hardware_enable,
  SIZE_REGISTERS
};

// EEPROM storage locations
enum EEPROMAdress
{
  ee_rate = 10,
  ee_pid_p = 12,
  ee_pid_i = 14,
  ee_pid_d = 16,
  ee_tc_type = 18,
};

enum laser_safety_state
{
  disarmed,
  software_armed,
  fully_armed
};

void set_output_power(int power);
void working_setpoint_adjust();
void setup_timer();
void write_to_eeprom();
void read_from_eeprom();
void pid_calculation(bool reset = false);
void cut_power();
uint16_t sanitize(uint16_t value, uint16_t lower_bound, uint16_t upper_bound, uint16_t fallback);

// About units:
// All temperature like variables (process_variable, setpoint, error, sum_error and diff_error, pid_p) are stored in tenths of a degree C
// Integration and derivative time are stored in seconds
// The time interval is stored in milliseconds
// The output is stored in hundredths of percents

void setup()
{
  pinMode(sw_enable, OUTPUT);
  pinMode(aiming_beam_enable, OUTPUT);
  pinMode(tc_fault_bypass, INPUT_PULLUP);
  pinMode(hw_enable_readback, INPUT);
  pinMode(MAX_DRDY, INPUT);
  pinMode(MAX_FAULT, INPUT);
  pinMode(pwr_control_pin, OUTPUT);
  pinMode(photodiode_readout, INPUT);

  digitalWrite(sw_enable, LOW);
  digitalWrite(aiming_beam_enable, LOW);
  
  setup_timer();

  // Start the Modbus RTU server, with (slave) id 1
  Serial.begin(9600);
  if (!ModbusRTUServer.begin(1, 9600))
  {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  // Initialize to type K thermocouple
  maxthermo.begin();
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  maxthermo.setNoiseFilter(MAX31856_NOISE_FILTER_50HZ);

  // Everything will be handled by holding registers, all data is stored as 16 bit int (Like Eurotherms do)
  ModbusRTUServer.configureHoldingRegisters(0x00, SIZE_REGISTERS);
  read_from_eeprom();
}

void loop()
{
  static laser_safety_state safety_state = disarmed;
  static uint32_t last_arming = millis();

  ModbusRTUServer.poll();
  write_to_eeprom();

  // Switch the aiming pin
  digitalWrite(aiming_beam_enable, ModbusRTUServer.holdingRegisterRead(reg_aiming_beam));

  // Read the thermocouple temperature and write it to the holding register as tenths of degrees
  ModbusRTUServer.holdingRegisterWrite(reg_working_process_variable, static_cast<int>(maxthermo.readThermocoupleTemperature() * 10 + 0.5));


  // Safety logic follows a finite state machine with three states: disarmed, software armed and fully armed.
  // Refer to the state diagram in the documentation for details
  switch (safety_state)
  {
  case disarmed:
    if (ModbusRTUServer.holdingRegisterRead(reg_software_enable) == 1)
    {
      digitalWrite(sw_enable, HIGH);
      safety_state = software_armed;
    }
    break;
  case software_armed:
    // Trip disarm if thermocouple error is detected and the bypass is not set, otherwise continue
    // Trip disarm if software enable is cleared, otherwise continue
    // If the hardware enable is enabled, go to fully armed
    // Don't trip the power cut if the hardware enable is switched off, otherwise the system would constantly try to switch itself off when only the software enable is switched on but the hardware enable is still off, which would lead to an unstable situation
    ModbusRTUServer.holdingRegisterWrite(reg_tc_error, maxthermo.readFault());
    if (ModbusRTUServer.holdingRegisterRead(reg_tc_error) && digitalRead(tc_fault_bypass) == HIGH)
    {
      cut_power();
      safety_state = disarmed;
    }
    if (ModbusRTUServer.holdingRegisterRead(reg_software_enable) == 0)
    {
      cut_power();
      safety_state = disarmed;
    }
    if (digitalRead(hw_enable_readback) == HIGH)
    {
      ModbusRTUServer.holdingRegisterWrite(reg_hardware_enable, 1);
      safety_state = fully_armed;
      last_arming = millis();
    }
    break;
  case fully_armed:
    // Trip disarm if thermocouple error is detected and the bypass is not set
    // Trip disarm if software enable is cleared, otherwise continue
    // Trip disarm if hardware enable is switched off, otherwise continue
    ModbusRTUServer.holdingRegisterWrite(reg_tc_error, maxthermo.readFault());
    if (ModbusRTUServer.holdingRegisterRead(reg_tc_error) && digitalRead(tc_fault_bypass) == HIGH)
    {
      cut_power();
      safety_state = disarmed;
    }
    if (ModbusRTUServer.holdingRegisterRead(reg_software_enable) == 0)
    {
      cut_power();
      safety_state = disarmed;
    }
    if (digitalRead(hw_enable_readback) == LOW && millis() - last_arming > 500) // Add a delay to prevent immediate disarming if the hw_enable_readback takes a bit to read high
    {
      cut_power();
      safety_state = disarmed;
    }
    break;
  }

  // If the system is fully armed, perform control calculations and set output power 
  if (safety_state == fully_armed)
  {
    if(ModbusRTUServer.holdingRegisterRead(reg_control_mode))
    {
      // Reset the PID calculation to prevent integral windup while in automatic mode
      pid_calculation(true);
      ModbusRTUServer.holdingRegisterWrite(reg_working_power, ModbusRTUServer.holdingRegisterRead(reg_manual_power));
    }
    else
    {
      working_setpoint_adjust();
      pid_calculation();
    }
    set_output_power(ModbusRTUServer.holdingRegisterRead(reg_working_power)); 
  }
  else
  {
    set_output_power(0);
  }
}

void cut_power()
{
  // If any of the safety checks fail, cut the power to the laser and reset the controller
  // Switch to manual mode, set output power to 0 and reset the PID controller
  set_output_power(0);
  ModbusRTUServer.holdingRegisterWrite(reg_control_mode, 1);
  ModbusRTUServer.holdingRegisterWrite(reg_working_power, 0);
  ModbusRTUServer.holdingRegisterWrite(reg_manual_power, 0);
  ModbusRTUServer.holdingRegisterWrite(reg_software_enable, 0);
  ModbusRTUServer.holdingRegisterWrite(reg_hardware_enable, 0);
  digitalWrite(sw_enable, LOW);
  pid_calculation(true);
}

void set_output_power(int power)
{
  // Adjust duty cycle
  // Direct timer manipulation to access 16 bit resolution
  power = constrain(power, 0, 10000);
  OCR1A = power;
}

void setup_timer()
{
  // Setup timer 1 for PWM output on pin 9 (OC1A)
  // Phase and frequency correct pwm mode
  // No prescaler
  // Top value 10000
  // PWM frequency 800 Hz
  noInterrupts();
  TCCR1A = 0 | (1 << COM1A1);
  TCCR1B = 0 | (1 << WGM13) | (1 << CS10);
  ICR1 = 10000;
  OCR1A = 0;
  interrupts();
}

void write_to_eeprom()
{
  static uint16_t pid_p = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_p));
  static uint16_t pid_i = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_i));
  static uint16_t pid_d = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_d));
  static uint16_t rate = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_rate));
  static uint16_t tc_type = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_tc_type));

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_p)) != pid_p)
  {
    pid_p = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_p));
    EEPROM.put(ee_pid_p, pid_p);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_i)) != pid_i)
  {
    pid_i = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_i));
    EEPROM.put(ee_pid_i, pid_i);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_d)) != pid_d)
  {
    pid_d = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_d));
    EEPROM.put(ee_pid_d, pid_d);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_rate)) != rate)
  {
    rate = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_rate));
    EEPROM.put(ee_rate, rate);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_tc_type)) != tc_type)
  {
    tc_type = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_tc_type));
    EEPROM.put(ee_tc_type, tc_type);
  }
}

uint16_t sanitize(uint16_t value, uint16_t lower_bound, uint16_t upper_bound, uint16_t fallback)
{
  if (value < lower_bound || value > upper_bound)
    value = fallback;
  return value;
}

void read_from_eeprom()
{
  uint16_t temp;
  ModbusRTUServer.holdingRegisterWrite(reg_pid_p, sanitize(EEPROM.get(ee_pid_p, temp), 1, 10000, 750));
  ModbusRTUServer.holdingRegisterWrite(reg_pid_i, sanitize(EEPROM.get(ee_pid_i, temp), 1, 10000, 100));
  ModbusRTUServer.holdingRegisterWrite(reg_pid_d, sanitize(EEPROM.get(ee_pid_d, temp), 0, 10000, 0));
  ModbusRTUServer.holdingRegisterWrite(reg_rate, sanitize(EEPROM.get(ee_rate, temp), 1, 1200, 15));
    ModbusRTUServer.holdingRegisterWrite(reg_tc_type, EEPROM.get(ee_tc_type, temp));
}

void pid_calculation(bool reset)
{
  // About units:
  // All temperature like variables (process_variable, setpoint, error, sum_error and diff_error, pid_p) are stored in tenths of a degree C
  // Integration and derivative time are stored in seconds
  // The time interval is stored in milliseconds, hence the factors 1000
  // The output is stored in hundredths of percents
  // Shamelessly copied from the Arduino PID Library

  // Memory
  static double last_input = ModbusRTUServer.holdingRegisterRead(reg_working_process_variable);
  static double output_sum = 0;

  if(reset)
  {
    // Reset the error accumulation to prevent integral windup and large derivative kick when switching from manual to automatic mode or when a thermocouple error is detected and the controller is switched off
    output_sum = 0;
    return;
  }

  // Timing stuff
  const uint16_t interval = 100;
  static uint32_t last_time = millis();
  uint32_t now = millis();
  unsigned long timeChange = (now - last_time);

  // Control loop
  if (timeChange >= interval)
  {
    // Convert from parallel for to standard form
    // Guard against division by zero, by setting the k-terms to zero if the user entered the parallel parameters as zero
    double kp, ki, kd;
    kp = ModbusRTUServer.holdingRegisterRead(reg_pid_p) == 0 ? 0 : 10000 / ModbusRTUServer.holdingRegisterRead(reg_pid_p);
    ki = ModbusRTUServer.holdingRegisterRead(reg_pid_i) == 0 ? 0 : kp / ModbusRTUServer.holdingRegisterRead(reg_pid_i) * interval / 1000;
    kd = kp * ModbusRTUServer.holdingRegisterRead(reg_pid_d) * 1000 / interval;

    /*Compute all the working error variables*/
    double input = ModbusRTUServer.holdingRegisterRead(reg_working_process_variable);
    double error = ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) - input;
    double dInput = (input - last_input);

    // Integral term
    output_sum += (ki * error);
    output_sum = constrain(output_sum, 0, 10000);

    // Add integral, proportional and differential term term
    double output = output_sum + kp * error - kd * dInput;
    output = constrain(output, 0, 10000);

    last_input = input;
    last_time = now;

    ModbusRTUServer.holdingRegisterWrite(reg_working_power, output);
  }
}

void working_setpoint_adjust()
{
  // Adjust the working setpoint by an increment based on the given ramp or until the target SP is reached
  // Only perform the update every 100 milliseconds
  // The setpoint is written to and read from the Modbus register as 10ths of degrees (deci degrees), the ramp in deci degrees per minute
  // To minimize rounding errors, the increment calculations (increment and last_setpoint) are done in units of [deciDegrees times millisecond per minute]

  static uint32_t last_update = millis();
  uint32_t now = millis();
  if (now - last_update < 100)
    return;

  static int32_t last_setpoint = ModbusRTUServer.holdingRegisterRead(reg_working_setpoint);
  
  // Calculate the change of setpoint
  int32_t increment = ModbusRTUServer.holdingRegisterRead(reg_rate) * (now - last_update);

  // Check if the controller is ramping up or down and adjust setpoint by increment or until target SP is reached
  if (ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) < ModbusRTUServer.holdingRegisterRead(reg_target_setpoint))
  {
    last_setpoint = min(ModbusRTUServer.holdingRegisterRead(reg_target_setpoint) * 1000 * 60, last_setpoint + increment);
  }
  else if(ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) > ModbusRTUServer.holdingRegisterRead(reg_target_setpoint))
  {
    last_setpoint = max(ModbusRTUServer.holdingRegisterRead(reg_target_setpoint) * 1000 * 60, last_setpoint - increment);
  }
  ModbusRTUServer.holdingRegisterWrite(reg_working_setpoint, static_cast<int>(last_setpoint / 1000 / 60 + 0.5));
  last_update = now;
}