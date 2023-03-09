#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

// Pin definitions
const byte pwr_control_pin = 4;
const byte enable_pin = 2;

// Globals
int working_output = 0;
int working_setpoint = 0;
int tc_readout = 0;

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
  reg_tc_error,
  SIZE_REGISTERS
};


void set_output_power(int power);
void update_working_registers();

void setup()
{
  Serial.begin(9600);

  Serial.println("Modbus RTU Server LED");

  // start the Modbus RTU server, with (slave) id 1
  if (!ModbusRTUServer.begin(1, 9600))
  {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  // Everything will be handled by holding registers, all data is stored as 16 bit int (Like Eurotherms do)
  ModbusRTUServer.configureHoldingRegisters(0x00, SIZE_REGISTERS);
}

void loop()
{
  // Read thermocouple and handle errors

  if(ModbusRTUServer.holdingRegisterRead(reg_control_mode))
  {
    //do automatic mode:
    //working_setpoint_adjust()
    //PID calculation

    set_output_power(working_output);
  }
  else
  {
    set_output_power(ModbusRTUServer.holdingRegisterRead(reg_manual_power));
  }
  update_working_registers();
}

void set_output_power(int power)
{
  // Adjust duty cycle
  // Direct timer manipulation to access 16 birt resolution
}

void update_working_registers()
{
  // Update the registers with the working setpoint and working output
  ModbusRTUServer.holdingRegisterWrite(reg_working_power, working_output);
  ModbusRTUServer.holdingRegisterWrite(reg_working_setpoint, working_setpoint);
  ModbusRTUServer.holdingRegisterWrite(reg_working_process_variable, tc_readout);
}