#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <PID_v1.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Adafruit_MAX31856.h>

// Pin definitions
const byte pwr_control_pin = 9;
const byte enable_pin = 2;
const byte MAX_CS = 10;

Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(MAX_CS);

// Temporary variables because the PID algorithm works with doubles but the Modbus Server stores unsigned integers
double pid_setpoint;
double pid_input;
double pid_output;

PID pid_controller(&pid_input, &pid_output, &pid_setpoint, 1, 1, 1, DIRECT);


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
void setup_timer();
void write_to_eeprom();
void read_from_eeprom();
void set_pid_parameters();
void pid_reg_to_temp();
void pid_temp_to_reg();

void setup()
{
  pinMode(enable_pin, OUTPUT);
  pinMode(pwr_control_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  digitalWrite(pwr_control_pin, LOW);
  setup_timer();

  // Start the Modbus RTU server, with (slave) id 1
  Serial.begin(9600);
  if(!ModbusRTUServer.begin(1, 9600))
  {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  // Initialize to type S thermocouple
  maxthermo.begin();
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_S);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  maxthermo.setNoiseFilter(MAX31856_NOISE_FILTER_50HZ);

  // Everything will be handled by holding registers, all data is stored as 16 bit int (Like Eurotherms do)
  ModbusRTUServer.configureHoldingRegisters(0x00, SIZE_REGISTERS);
  read_from_eeprom();
  pid_controller.SetOutputLimits(0, 10000);
}

void loop()
{
  ModbusRTUServer.poll();

  // Read thermocouple error state and store it in the register, if errors are detected switch the enable register off
  // If no errors are detected, read temperature
  ModbusRTUServer.holdingRegisterWrite(reg_tc_error, maxthermo.readFault());
  if(ModbusRTUServer.holdingRegisterRead(reg_tc_error))
  {
    ModbusRTUServer.holdingRegisterWrite(reg_software_enable, 0);
  }
  else
  {
    ModbusRTUServer.holdingRegisterWrite(reg_working_process_variable, static_cast<int>(maxthermo.readThermocoupleTemperature() * 10 + 0.5));
  }

  // Control loop
  if(ModbusRTUServer.holdingRegisterRead(reg_control_mode))
  {
    //TODO: working_setpoint_adjust()
    pid_reg_to_temp();
    pid_controller.Compute();
    pid_temp_to_reg();

    set_output_power(ModbusRTUServer.holdingRegisterRead(reg_working_power));
  }
  else
  {
    set_output_power(ModbusRTUServer.holdingRegisterRead(reg_manual_power));
  }

  digitalWrite(enable_pin, ModbusRTUServer.holdingRegisterRead(reg_software_enable));

  set_pid_parameters();
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
  TCCR1B = 0 | (1<<WGM13) | (1<<CS10);
  ICR1 = 10000;
  OCR1A = 0;
  interrupts();
}

void write_to_eeprom()
{
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

void pid_reg_to_temp()
{
  pid_setpoint = static_cast<double>(ModbusRTUServer.holdingRegisterRead(reg_working_setpoint));
  pid_input = static_cast<double>(ModbusRTUServer.holdingRegisterRead(reg_working_process_variable));
}

void pid_temp_to_reg()
{
  ModbusRTUServer.holdingRegisterWrite(reg_working_power, static_cast<unsigned int>(pid_output));
}

void set_pid_parameters()
{
  // Read PID paramters from registers, convert to parallel form and set parameters
  double kp = 1 / ModbusRTUServer.holdingRegisterRead(reg_pid_p);
  double ki = kp / ModbusRTUServer.holdingRegisterRead(reg_pid_i);
  double kd = kp * ModbusRTUServer.holdingRegisterRead(reg_pid_d);
  pid_controller.SetTunings(kp, ki, kd);
}