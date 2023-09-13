#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Adafruit_MAX31856.h>

// Pin definitions
const byte pwr_control_pin = 9;
const byte enable_pin = 2;
const byte MAX_CS = 10;

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
  SIZE_REGISTERS
};

// EEPROM storage locations
enum EEPROMAdress
{
  ee_rate = 0,
  ee_pid_p = 2,
  ee_pid_i = 4,
  ee_pid_d = 6
};

void set_output_power(int power);
void working_setpoint_adjust();
void setup_timer();
void write_to_eeprom();
void read_from_eeprom();
void pid_calculation();

// About units:
// All temperature like variables (process_variable, setpoint, error, sum_error and diff_error, pid_p) are stored in tenths of a degree C
// Integration and derivative time are stored in seconds
// The time interval is stored in milliseconds, hence the factors 1000 in line +24
// The output is stored in hundreths of percents

void setup()
{
  pinMode(enable_pin, OUTPUT);
  pinMode(pwr_control_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  digitalWrite(pwr_control_pin, LOW);
  setup_timer();

  // Start the Modbus RTU server, with (slave) id 1
  Serial.begin(9600);
  if (!ModbusRTUServer.begin(1, 9600))
  {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1)
      ;
  }

  // Initialize to type S thermocouple
  maxthermo.begin();
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_S);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  maxthermo.setNoiseFilter(MAX31856_NOISE_FILTER_50HZ);

  // Everything will be handled by holding registers, all data is stored as 16 bit int (Like Eurotherms do)
  ModbusRTUServer.configureHoldingRegisters(0x00, SIZE_REGISTERS);
  read_from_eeprom();
}

void loop()
{
  ModbusRTUServer.poll();

  // Read thermocouple error state and store it in the register, if errors are detected switch the enable register off
  // If no errors are detected, read temperature
  ModbusRTUServer.holdingRegisterWrite(reg_tc_error, maxthermo.readFault());
  if (ModbusRTUServer.holdingRegisterRead(reg_tc_error))
  {
    ModbusRTUServer.holdingRegisterWrite(reg_software_enable, 0);
  }
  else
  {
    ModbusRTUServer.holdingRegisterWrite(reg_working_process_variable, static_cast<int>(maxthermo.readThermocoupleTemperature() * 10 + 0.5));
  }

  if (ModbusRTUServer.holdingRegisterRead(reg_control_mode))
  {
    ModbusRTUServer.holdingRegisterWrite(reg_working_power, ModbusRTUServer.holdingRegisterRead(reg_manual_power));
  }
  else
  {
    working_setpoint_adjust();
    pid_calculation();
  }

  set_output_power(ModbusRTUServer.holdingRegisterRead(reg_working_power));
  digitalWrite(enable_pin, ModbusRTUServer.holdingRegisterRead(reg_software_enable));

  // TODO: Adjust PID parameters from registers
  // TODO: Write PID parameters to eeprom
  write_to_eeprom();
}

void set_output_power(int power)
{
  // Adjust duty cycle
  // Direct timer manipulation to access 16 birt resolution
  OCR1A = power;
}

void setup_timer()
{
  // Setup timer 1 for PWM output
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
  return;
  // TODO: Refactor this to updates
  EEPROM.put(ee_pid_p, ModbusRTUServer.holdingRegisterRead(reg_pid_p));
  EEPROM.put(ee_pid_i, ModbusRTUServer.holdingRegisterRead(reg_pid_i));
  EEPROM.put(ee_pid_d, ModbusRTUServer.holdingRegisterRead(reg_pid_d));
  EEPROM.put(ee_rate, ModbusRTUServer.holdingRegisterRead(reg_rate));
}

void read_from_eeprom()
{
  unsigned int temp;
  ModbusRTUServer.holdingRegisterWrite(reg_pid_p, EEPROM.get(ee_pid_p, temp));
  ModbusRTUServer.holdingRegisterWrite(reg_pid_i, EEPROM.get(ee_pid_i, temp));
  ModbusRTUServer.holdingRegisterWrite(reg_pid_d, EEPROM.get(ee_pid_d, temp));
  ModbusRTUServer.holdingRegisterWrite(reg_rate, EEPROM.get(ee_rate, temp));
}

void pid_calculation()
{
  // About units:
  // All temperature like variables (process_variable, setpoint, error, sum_error and diff_error, pid_p) are stored in tenths of a degree C
  // Integration and derivative time are stored in seconds
  // The time interval is stored in milliseconds, hence the factors 1000 in line +24
  // The output is stored in hundreths of percents

  const int interval = 100;

  static unsigned long last_update = millis();
  if (millis() - last_update < interval)
    return;

  last_update = millis();

  static long int last_error = 0;
  static long int error_sum = 0;

  long int error = ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) - ModbusRTUServer.holdingRegisterRead(reg_working_process_variable);

  // Over/underflow prevention for error integration
  const long int max_error_sum = 2147483647L;
  const long int min_error_sum = -2147483648L;
  if (max_error_sum - error_sum < error && error > 0)
    error_sum = max_error_sum;
  else if (min_error_sum - error_sum > error && error < 0)
    error_sum = min_error_sum;
  else
    error_sum += error;

  long int error_diff = error - last_error;
  last_error = error;

  long int output = (error + error_sum * interval / 1000 / ModbusRTUServer.holdingRegisterRead(reg_pid_i) + error_diff * ModbusRTUServer.holdingRegisterRead(reg_pid_d) / interval * 1000) / ModbusRTUServer.holdingRegisterRead(reg_pid_p);
  output = static_cast<int>(constrain(output, 0, 10000));

  ModbusRTUServer.holdingRegisterWrite(reg_working_power, output);
}

void working_setpoint_adjust()
{
  // Adjust the working setpoint with the given ramp
  // If the working setpoint already equals the target setpoint, do nothing
  // Every 100 milliseconds a new setpoint is calculated based on elapsed time since last update and ramp
  // The setpoint is written to the Modbus register as 10ths of degrees, but kept internally as [deciDegrees times millisecond per minute] to minimize rounding errors

  if (ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) == ModbusRTUServer.holdingRegisterRead(reg_target_setpoint))
    return;
  
  static unsigned long int last_update = millis();
  static long int last_setpoint = 0;
  unsigned long int now = millis();

  if (now - last_update > 100)
  {
    unsigned long int increment = ModbusRTUServer.holdingRegisterRead(reg_rate) * (now - last_update);
    if (last_setpoint/1000/60 < ModbusRTUServer.holdingRegisterRead(reg_target_setpoint))
    {
      last_setpoint += increment;
    }
    else
      last_setpoint -= increment;
    last_update = now;
    ModbusRTUServer.holdingRegisterWrite(reg_working_setpoint, static_cast<int>(last_setpoint/1000/60+0.5));
  }
}