/*
 * user_code.c - All User Code should be applied here unless specified otherwise.
 *
 */

/* File Includes */
#include "user_code.h"
#include "backend_functions.h"
#include "main.h"
#include "snprintf.h"
#include <string.h>
#include "stm32g4xx_hal.h"
#include <stdlib.h>
/* End File Includes */

/* Variable Declarations */
uint32_t serialnumber;
CAN_ErrorCounts errors;
uint8_t countervaluea = 0;
uint8_t countervalueb = 0;
float vehicle_speed = 0;
float wheel_speed_fl = 0;
float wheel_speed_fr = 0;
float wheel_speed_rl = 0;
float wheel_speed_rr = 0;
float wheel_psi_fl = 0;
float wheel_psi_fr = 0;
float wheel_psi_rl = 0;
float wheel_psi_rr = 0;
uint32_t sport_state = 0;
uint32_t bc_state = 0;
uint32_t rw_state = 0;
uint32_t ccplus_state = 0;
uint32_t ccminus_state = 0;
uint32_t ccset_state = 0;
uint32_t ccresume_state = 0;
uint32_t radplus_state = 0;
uint32_t radminus_state = 0;
uint32_t radleft_state = 0;
uint32_t radright_state = 0;
uint32_t clutch_state = 0;
float brake_pressure = 0.0f;
float brake_state = 0.0f;
uint32_t dsc_state = 0;
float fuel_lev_r = 0;
float fuel_lev_l = 0;
float fuel_lev = 0;
uint32_t engine_run = 0;
uint32_t moving_forward = 0;
float engine_rpm = 0.0f;
float minirpm = 0.0f;
float vehicleSpeed = 0.0f;
float sa_temp = 0.0f;
uint8_t engineData[8];
uint8_t rpmData[8];
uint32_t trimPos = 0;
uint32_t trimMin = 0;
uint32_t trimMax = 5;
uint32_t lastTrim = 0;
uint32_t ac_button = 0;

char debug_buffer[100];

// Haltech IO Box A Variables
float haltech_IO_Box_A_AVI1 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_A_AVI2 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_A_AVI3 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_A_AVI4 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_A_DPI1_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPI2_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPI3_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPI4_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPI1_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPI2_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPI3_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPI4_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPO1_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPO2_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPO3_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DPO4_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_A_DP01_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPO2_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPO3_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_A_DPO4_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
uint8_t haltech_IO_Box_A_Status = 1;			 // 0=In bootmode,1=In firmware,2=Hardware failure,3=Firmware erased,4=Watchdog timeout,5=Illegal op-code //
uint8_t haltech_IO_Box_A_ID_Conflict = 0;		 // 0=No Conflict, 1=Conflict //
uint8_t haltech_IO_Box_A_Bootcode_Version = 31;	 // min 0, max 31, resolution 1, recommend highest value{31} //
uint8_t haltech_IO_Box_A_FW_Major_Version = 3;	 // min 0, max 3, resolution 1, recommend highest value{3} //
uint8_t haltech_IO_Box_A_FW_Minor_Version = 255; // min 0, max 255, resolution 1, recommend highest value{255} //
uint8_t haltech_IO_Box_A_FW_Bugfix_Version = 0;	 // min 0, max 255, resolution 1, currently unaffected by value //
uint8_t haltech_IO_Box_A_FW_Release_Type = 0;	 // min 0, max 255, resolution 1, currently unaffected by value //
uint8_t haltech_IO_Box_A_AVI1_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_A_DPO1_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_A_DPO1_Active = 0;		 // 0=Low, 1=High
uint8_t haltech_IO_Box_A_AVI2_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_A_DPO2_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_A_DPO2_Active = 0;		 // 0=Low, 1=High
uint8_t haltech_IO_Box_A_AVI3_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_A_DPO3_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_A_DPO3_Active = 0;		 // 0=Low, 1=High
uint8_t haltech_IO_Box_A_AVI4_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_A_DPO4_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_A_DPO4_Active = 0;		 // 0=Low, 1=High
// End HALTECH_IO_BOX_A variables

// Haltech IO Box B Variables
float haltech_IO_Box_B_AVI1 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_B_AVI2 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_B_AVI3 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_B_AVI4 = 0.0f;				 // min 0.00v, max 5.00v, resolution 0.001221'ish volts (4095 Steps..) //
float haltech_IO_Box_B_DPI1_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DPI2_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DPI3_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DPI4_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
uint32_t haltech_IO_Box_A_DPI1_Hz = 1;			 // min 0 Hz, max 20000 Hz, resolution Variable based on Pulse Width, min 1 Hz after 300 hz, resolution will start reducing.. //
uint32_t haltech_IO_Box_A_DPI2_Hz = 0;			 // min 0 Hz, max 20000 Hz, resolution Variable based on Pulse Width, min 1 Hz after 300 hz, resolution will start reducing.. //
uint32_t haltech_IO_Box_A_DPI3_Hz = 0;			 // min 0 Hz, max 20000 Hz, resolution Variable based on Pulse Width, min 1 Hz after 300 hz, resolution will start reducing.. //
uint32_t haltech_IO_Box_A_DPI4_Hz = 0;			 // min 0 Hz, max 20000 Hz, resolution Variable based on Pulse Width, min 1 Hz after 300 hz, resolution will start reducing.. //
float haltech_IO_Box_B_DPI1_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPI2_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPI3_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPI4_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPO1_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DPO2_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DPO3_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DPO4_Duty = 0.0f;		 // min 0.0%, max 100.0%, resolution 0.1% //
float haltech_IO_Box_B_DP01_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPO2_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPO3_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
float haltech_IO_Box_B_DPO4_Period = 0.0f;		 // min 0.00ms, max 50.00ms, resolution 0.01ms //
uint8_t haltech_IO_Box_B_Status = 1;			 // 0=In bootmode,1=In firmware,2=Hardware failure,3=Firmware erased,4=Watchdog timeout,5=Illegal op-code //
uint8_t haltech_IO_Box_B_ID_Conflict = 0;		 // 0=No Conflict, 1=Conflict //
uint8_t haltech_IO_Box_B_Bootcode_Version = 31;	 // min 0, max 31, resolution 1, recommend highest value{31} //
uint8_t haltech_IO_Box_B_FW_Major_Version = 3;	 // min 0, max 3, resolution 1, recommend highest value{3} //
uint8_t haltech_IO_Box_B_FW_Minor_Version = 255; // min 0, max 255, resolution 1, recommend highest value{255} //
uint8_t haltech_IO_Box_B_FW_Bugfix_Version = 0;	 // min 0, max 255, resolution 1, currently unaffected by value //
uint8_t haltech_IO_Box_B_FW_Release_Type = 0;	 // min 0, max 255, resolution 1, currently unaffected by value //
uint8_t haltech_IO_Box_B_AVI1_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_B_DPO1_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_B_DPO1_Active = 0;		 // 0=Low, 1=High
uint8_t haltech_IO_Box_B_AVI2_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_B_DPO2_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_B_DPO2_Active = 0;		 // 0=Low, 1=High
uint8_t haltech_IO_Box_B_AVI3_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_B_DPO3_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_B_DPO3_Active = 0;		 // 0=Low, 1=High
uint8_t haltech_IO_Box_B_AVI4_Pullup = 0;		 // 0=off, 1=on
uint8_t haltech_IO_Box_B_DPO4_Error = 0;		 // 0=Inactive, 1=Active
uint8_t haltech_IO_Box_B_DPO4_Active = 0;		 // 0=Low, 1=High

float flws_2_hz = 0.0f;
float frws_2_hz = 0.0f;
float rlws_2_hz = 0.0f;
float rrws_2_hz = 0.0f;

// End HALTECH_IO_BOX_B variables

// Continental RSX ABS Variables

float Conti_Steering_Angle;			   // 0.1 deg resolution. min -360 deg, -1000 to 1000 deg max range, -360 to 360 deg max table range
float Conti_Brake_Pressure_Front;	   // 0.1 bar resolution. min 0 bar, max 4095 bar, 0 to 2800 bar table range
float Conti_Brake_Pressure_Rear;	   // 0.1 bar resolution. min 0 bar, max 4095 bar, 0 to 2800 bar table range
float Conti_Wheel_Speed_FL;			   // roughly 0.25 km/h resolution. min 0 km/h, max 400 km/h table range
float Conti_Wheel_Speed_FR;			   // roughly 0.25 km/h resolution. min 0 km/h, max 400 km/h table range
float Conti_Wheel_Speed_RL;			   // roughly 0.25 km/h resolution. min 0 km/h, max 400 km/h table range
float Conti_Wheel_Speed_RR;			   // roughly 0.25 km/h resolution. min 0 km/h, max 400 km/h table range
float Conti_Yaw_Rate;				   // 0.1 deg/s resolution. min -163.83 deg/s, max 163.83 deg/s
float Conti_Lateral_Acceleration;	   // 0.1 m/s^2 resolution. min -89.5 m/s^2, max 12.8 m/s^2
float Conti_Longitudinal_Acceleration; // 0.1 m/s^2 resolution. min -12.8 m/s^2, max 89.5 m/s^2
uint8_t Conti_ABS_Mode;				   // 01 - 12, 0x5C0 Byte 5 high nibble...
uint8_t Conti_IMU_msg[8];
int16_t lateral_accel_output;
int16_t longitudinal_accel_output;
int16_t yaw_rate_output;
bool yaw_rate_sign_positive;

uint8_t Conti_Steering_Angle_msg[8];
int16_t steering_angle_output;

uint8_t Conti_Wheel_speed_msg[8];
uint16_t front_left_wheel_speed_output;
uint16_t front_right_wheel_speed_output;
uint16_t rear_left_wheel_speed_output;
uint16_t rear_right_wheel_speed_output;

uint8_t Conti_Brake_Pressure_msg[8];

// End Continental RSX ABS Variables

/* End Variable Declarations */

/* Startup Functions */
void events_Startup()
{
	setupCANbus(CAN_1, 1000000, NORMAL_MODE); // Haltech & RSX ABS - 1000KB
	setupCANbus(CAN_2, 500000, NORMAL_MODE);  // Mini PT - 500KB
	// setupCANbus(CAN_3, 1000000, NORMAL_MODE);
	setCAN_Termination(CAN_1, true);
	setCAN_Termination(CAN_2, true);
	// setCAN_Termination(CAN_3, true);
	startCANbus(CAN_1);
	startCANbus(CAN_2);
	// startCANbus(CAN_3);
}
/* End Startup Functions */

void onSerialReceive(uint8_t *serialMessage)
{
	// What do you want to do when you receive a UART message.. ?
	// printf("%07.4f message received...\r\n", getTimestamp());
}

void onReceive(CAN_Message Message)
{
	// char formatted_message[50];
	// format_CAN_message(&Message, formatted_message, sizeof(formatted_message));
	// printf("%s\r\n", formatted_message);

	// What do you want to do when you receive a CAN message.. ?
	if (Message.Bus == CAN_1) // Haltech & RSX
	{
		// serialPrint("CAN 1 - received a message..\r\n");
		// send_message(CAN_2, Message.is_extended_id, Message.arbitration_id, Message.dlc, Message.data);

		if (Message.arbitration_id == 0x360) // Engine Speed - 864 // DONE!
		{
			uint32_t engine_speed = (Message.data[0] << 8) | (Message.data[1]);
			engine_rpm = process_float_value(engine_speed, 0xFFFF, false, 1, 0, 0);
			if (engine_rpm > 10)
			{
				minirpm = engine_rpm;
			}

			// Set engine run to enable EPAS and disable starter

			if (engine_rpm > 900 && engine_run == 0)
			{
				engine_run = 1;
			}
		}
		if (Message.arbitration_id == 0x370) // Vehicle Speed - 880 // DONE!
		{
			uint32_t vehicle_speed_data = (Message.data[0] << 8) | (Message.data[1]);
			vehicleSpeed = process_float_value(vehicle_speed_data, 0xFFFF, false, 0.1, 0, 0);
			// Note: This is in KM/H
			// if (vehicleSpeed > 0)
			// {
			// 	snprintf(debug_buffer, sizeof(debug_buffer), "Haltech MPH: %2.1f\r\n", vehicleSpeed);
			// 	// snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f and hz == %2.2f \r\n", wheel_speed_fl, flws_2_hz);
			// 	serialPrint(debug_buffer);
			// }
		}
		if (Message.arbitration_id == 0x3e0) // Coolant Temp - 992
		{
			uint32_t haltech_temp = (Message.data[0] << 8) | (Message.data[1]);
			sa_temp = process_float_value(haltech_temp, 0xFFFF, false, 0.1, 0, 4);
			sa_temp -= 273.15; // conversion from kelvin to celcius
		}
	}
	if (Message.Bus == CAN_2) // MINI PT
	{
		if (Message.arbitration_id == 0x0CE) // Wheel Speeds - 206 // DONE!
		{
			uint32_t wheel_fl = (Message.data[1] << 8) | (Message.data[0]);
			Conti_Wheel_Speed_FL = process_float_value(wheel_fl, 0xFFFF, false, 0.0625, 0, 0);
			uint32_t wheel_fr = (Message.data[3] << 8) | (Message.data[2]);
			Conti_Wheel_Speed_FR = process_float_value(wheel_fr, 0xFFFF, false, 0.0625, 0, 0);
			uint32_t wheel_rl = (Message.data[5] << 8) | (Message.data[4]);
			Conti_Wheel_Speed_RL = process_float_value(wheel_rl, 0xFFFF, false, 0.0625, 0, 0);
			uint32_t wheel_rr = (Message.data[7] << 8) | (Message.data[6]);
			Conti_Wheel_Speed_RR = process_float_value(wheel_rr, 0xFFFF, false, 0.0625, 0, 0);
			// snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f--FRONT--%2.1f \r\n              %2.1f--REAR---%2.1f\r\n", Conti_Wheel_Speed_FL, Conti_Wheel_Speed_FR, Conti_Wheel_Speed_RL, Conti_Wheel_Speed_RR);
			// serialPrint(debug_buffer);
			// haltech_IO_Box_A_DPI1_Duty = map_float(wheel_speed_fl, 0, 255, 0, 1000);
			// haltech_IO_Box_A_DPI2_Duty = map_float(wheel_speed_fr, 0, 255, 0, 1000);
			// haltech_IO_Box_A_DPI3_Duty = map_float(wheel_speed_rl, 0, 255, 0, 1000);
			// haltech_IO_Box_A_DPI4_Duty = map_float(wheel_speed_rr, 0, 255, 0, 1000);
			// if (wheel_speed_fl < 0.1)
			// {
			// 	flws_2_hz = 0;
			// }
			// else
			// {
			// 	flws_2_hz = 10000 / wheel_speed_fl;
			// }

			// if (wheel_speed_fr < 0.1)
			// {
			// 	frws_2_hz = 0;
			// }
			// else
			// {
			// 	frws_2_hz = 10000 / wheel_speed_fr;
			// }

			// if (wheel_speed_rl < 0.1)
			// {
			// 	rlws_2_hz = 0;
			// }
			// else
			// {
			// 	rlws_2_hz = 10000 / wheel_speed_rl;
			// }

			// if (wheel_speed_rr < 0.1)
			// {
			// 	rrws_2_hz = 0;
			// }
			// else
			// {
			// 	rrws_2_hz = 10000 / wheel_speed_rr;
			// }

			// // haltech_IO_Box_A_DPI1_Period = map_float(wheel_speed_fl, 0, 255, 50000, 1);
			// // haltech_IO_Box_A_DPI2_Period = map_float(wheel_speed_fr, 0, 255, 50000, 1);
			// // haltech_IO_Box_A_DPI3_Period = map_float(wheel_speed_rl, 0, 255, 50000, 1);
			// // haltech_IO_Box_A_DPI4_Period = map_float(wheel_speed_rr, 0, 255, 50000, 1);
			// snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f--FRONT--%2.1f \r\n              %2.1f--REAR---%2.1f\r\n", wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr);
			// // snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f and hz == %2.2f \r\n", wheel_speed_fl, flws_2_hz);
			// serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x315) // Sport Button - 789 // DONE!
		{
			sport_state = process_raw_value(Message.data[1], 0x7F);
			if (sport_state == 114)
			{
				// snprintf(debug_buffer, sizeof(debug_buffer), "Sport State: %i\r\n", sport_state);
				// serialPrint(debug_buffer);
				haltech_IO_Box_B_AVI1 = 4;
			}
			else
			{
				haltech_IO_Box_B_AVI1 = 0;
			}
		}
		if (Message.arbitration_id == 0x316) // DSC Button - 789 // DONE!
		{
			dsc_state = process_raw_value(Message.data[0], 0x01);

			snprintf(debug_buffer, sizeof(debug_buffer), "DSC State: %i\r\n", dsc_state);
			serialPrint(debug_buffer);
			if (dsc_state == 1)
			{
				haltech_IO_Box_B_AVI4 = 4;
			}
			else
			{
				haltech_IO_Box_B_AVI4 = 0;
			}
			// haltech_IO_Box_B_AVI4
			// Trim = haltech_IO_Box_B_AVI3
		}
		if (Message.arbitration_id == 0x1ee) // BC Button - 494 // DONE!
		{
			bc_state = process_raw_value(Message.data[0], 0x40);
			// snprintf(debug_buffer, sizeof(debug_buffer), "BC    State: %i\r\n", bc_state);
			// serialPrint(debug_buffer);
		}
		// if (Message.arbitration_id == 0x2a6) // Rear Wiper Switch - 678
		// {
		// 	rw_state = process_raw_value(Message.data[0], 0x40);
		// 	snprintf(debug_buffer, sizeof(debug_buffer), "RW    State: %i\r\n", rw_state);
		// 	serialPrint(debug_buffer);
		// }
		if (Message.arbitration_id == 0x194) // Cruise Buttons - 404 // DONE!
		{
			ccplus_state = process_raw_value(Message.data[2], 0x01);
			ccminus_state = process_raw_value(Message.data[2], 0x04);
			ccresume_state = process_raw_value(Message.data[2], 0x10); // or 0x10 (cancel?)
			ccset_state = process_raw_value(Message.data[2], 0x40);

			if (bc_state == 0)
			{
				if (ccplus_state == 1)
				{
					// snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: + %i\r\n", ccplus_state);
					// serialPrint(debug_buffer);
					haltech_IO_Box_B_AVI2 = 0.25;
				}
				else if (ccminus_state == 1)
				{
					// snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: - %i\r\n", ccminus_state);
					// serialPrint(debug_buffer);
					haltech_IO_Box_B_AVI2 = 0.50;
				}
				else if (ccset_state == 1)
				{
					// snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: set %i\r\n", ccset_state);
					// serialPrint(debug_buffer);
					haltech_IO_Box_B_AVI2 = 0.75;
				}
				else if (ccresume_state == 1)
				{
					// snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: resume %i\r\n", ccresume_state);
					// serialPrint(debug_buffer);
					haltech_IO_Box_B_AVI2 = 1;
				}
				else
				{
					haltech_IO_Box_B_AVI2 = 0;
				}
			}
		}
		if (Message.arbitration_id == 0x19e) // Brake Pressure - 414 // DONE!
		{
			uint16_t brake_pressure_data = (Message.data[6]);
			uint16_t brake_state_data = (Message.data[5]);
			brake_pressure = process_float_value(brake_pressure_data, 0xFF, false, 1, 0, 0);
			brake_state = process_float_value(brake_state_data, 0xFF, false, 1, 0, 0);
			if (brake_pressure == 0 && brake_state == 77)
			{
				brake_pressure = 1;
			}
			// snprintf(debug_buffer, sizeof(debug_buffer), "Brake Pressure: %2.0f\r\n", brake_pressure);
			// serialPrint(debug_buffer);
			// snprintf(debug_buffer, sizeof(debug_buffer), "Brake State: %2.0f\r\n", brake_state);
			// serialPrint(debug_buffer);
			float brakePressureMapped = map_float(brake_pressure, 0, 252, 0, 4095); // 725
			Conti_Brake_Pressure_Front = brakePressureMapped;
			Conti_Brake_Pressure_Rear = brakePressureMapped;
		}
		if (Message.arbitration_id == 0x330) // Fuel Level - 816 // DONE!
		{
			uint32_t fuel_lev_sig = Message.data[3];
			fuel_lev = process_float_value(fuel_lev_sig, 0xFF, false, 1, 0, 3);
			uint32_t fuel_lev_l_sig = Message.data[4];
			fuel_lev_l = process_float_value(fuel_lev_l_sig, 0xFF, false, 0.5, 0, 3);
			uint32_t fuel_lev_r_sig = Message.data[5];
			fuel_lev_r = process_float_value(fuel_lev_r_sig, 0xFF, false, 0.5, 0, 3);
			// haltech_IO_Box_A_AVI1 = map_float(fuel_lev, 0, 50, 0, 1);
			haltech_IO_Box_A_AVI2 = map_float(fuel_lev_l, 0, 30, 0, 1);
			haltech_IO_Box_A_AVI3 = map_float(fuel_lev_r, 0, 30, 0, 1);
		}
		if (Message.arbitration_id == 0x1a0) // Speed(MovingForward) - 416 // DONE!
		{
			moving_forward = process_raw_value(Message.data[1], 0x10);
		}
		// if (Message.arbitration_id == 0x1d0) // EngineData - 464 // DONE!
		// {
		// 	float ST_ENG_RUN = process_raw_value(Message.data[2], 0x30);
		// 	printf("%2.1f\r\n", ST_ENG_RUN);
		// }
		if (Message.arbitration_id == 0x242) // A/C Button) - 578 // DONE!
		{
			ac_button = process_raw_value(Message.data[1], 0x01);
			// printf("%d\r\n", ac_button);
			if (ac_button == 1)
			{
				haltech_IO_Box_A_AVI1 = 4.75;
			}
			else
			{
				haltech_IO_Box_A_AVI1 = 0;
			}
		}
	}
	// if (Message.Bus == CAN_3)
	// {
	// }
}

/* Run 2000Hz Functions here */
void events_2000Hz()
{
}

/* Run 1000Hz Functions here */
void events_1000Hz()
{
}

/* Run 500Hz Functions here */
void events_500Hz()
{
}

/* Run 200Hz Functions here */
void events_200Hz()
{
	// Mini Tach Here
	if (engine_run == 1)
	{
		// TODO - Fix scaling for rpmtomini, it may be correct now. Easiest to compare in the car because desk haltech sends zero along with logs.
		int16_t rpmtomini = (uint16_t)map_float(minirpm, 0, 8400, 0, 32000); // scale in dbc is .25 so i multiplied 8k by 4
		// printf("Some shit: %d\r\n", rpmtomini);
		rpmData[0] = (uint8_t)0x7E;
		rpmData[1] = (uint8_t)0x74;
		rpmData[2] = (uint8_t)0xFF;
		rpmData[3] = (uint8_t)0x00;
		rpmData[4] = (uint8_t)(rpmtomini);
		rpmData[5] = (uint8_t)(rpmtomini >> 8);
		rpmData[6] = (uint8_t)0x84;
		rpmData[7] = (uint8_t)0x00;
		// printf("Some shit: %d\r\n", engineData);
		send_message(CAN_2, false, 0x0aa, 8, rpmData);
	}
}

/* Run 100Hz Functions here - 10ms */
void events_100Hz()
{
	// uint16_t forward = 1;
	// uint16_t wsfl = (uint16_t)roundfloat_to_int32(wheel_speed_fl, 0) * 10;
	// uint16_t wsfr = (uint16_t)roundfloat_to_int32(wheel_speed_fr, 0) * 10;
	// uint16_t wsrl = (uint16_t)roundfloat_to_int32(wheel_speed_rl, 0) * 10;
	// uint16_t wsrr = (uint16_t)roundfloat_to_int32(wheel_speed_rr, 0) * 10;

	// uint16_t wsbyte0 = (uint8_t)wsfl;
	// uint16_t wsbyte1 = (uint8_t)((wsfl >> 8) << 4) | (wsfr & 0x0F);
	// uint16_t wsbyte2 = (uint8_t)(wsfr >> 4);
	// uint16_t wsbyte3 = (uint8_t)wsrl;
	// uint16_t wsbyte4 = (uint8_t)((wsrl >> 8) << 4) | (wsrr & 0x0F);
	// uint16_t wsbyte5 = (uint8_t)(wsrr >> 4);
	// uint16_t wsbyte6 = (uint8_t)wsfl;
	// uint16_t wsbyte7 = (uint8_t)((wsfl >> 8) << 4) | (forward & 0x0F);

	// uint8_t wheelSpeedData[8] = {(uint8_t)wsbyte0, (uint8_t)wsbyte1, (uint8_t)wsbyte2, (uint8_t)wsbyte3, (uint8_t)wsbyte4, (uint8_t)wsbyte5, (uint8_t)wsbyte6, (uint8_t)wsbyte7};

	// send_message(CAN_1, false, 0x340, 8, wheelSpeedData);
	// printf("%08b\r\n", wsbyte6);

	// uint8_t wheelSpeedData[8] = {(uint8_t)wsbyte0, (uint8_t)wsbyte1, (uint8_t)wsbyte2, (uint8_t)wsbyte3, (uint8_t)wsbyte4, (uint8_t)wsbyte5, (uint8_t)wsbyte6, (uint8_t)wsbyte7};
	// send_message(CAN_1, false, 0x341, 8, wheelSpeedData);

	// snprintf(debug_buffer, sizeof(debug_buffer), "Test Byte: %u, %u, %u\r\n", (uint8_t)testbyte1, (uint8_t)testbyte2, (uint8_t)testbyte3);
	//  snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %u--FRONT--%2.1f \r\n              %2.1f--REAR---%2.1f\r\n", testbyte, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr);
	//  snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f and hz == %2.2f \r\n", wheel_speed_fl, flws_2_hz);
	// serialPrint(debug_buffer);

	lateral_accel_output = (uint16_t)clamped_map_float(Conti_Lateral_Acceleration, -89.5, 12.8, 4095, 0);
	longitudinal_accel_output = (uint16_t)map_float(Conti_Longitudinal_Acceleration, -12.8, 89.5, 0, 4095);
	if (Conti_Yaw_Rate < 0)
	{
		yaw_rate_sign_positive = false;
		yaw_rate_output = (uint16_t)(clamped_map_float((Conti_Yaw_Rate * -1), 0, 163.83, 0, 16383));
	}
	else
	{
		yaw_rate_sign_positive = true;
		yaw_rate_output = (uint16_t)clamped_map_float(Conti_Yaw_Rate, 0, 163.83, 0, 16383) | 0x4000;
	}

	Conti_IMU_msg[0] = (uint8_t)(lateral_accel_output >> 8);
	Conti_IMU_msg[1] = (uint8_t)(lateral_accel_output & 0xFF);
	Conti_IMU_msg[2] = (uint8_t)(longitudinal_accel_output >> 8);
	Conti_IMU_msg[3] = (uint8_t)(longitudinal_accel_output & 0xFF);
	Conti_IMU_msg[4] = (uint8_t)(yaw_rate_output >> 8);
	Conti_IMU_msg[5] = (uint8_t)(yaw_rate_output & 0xFF);
	Conti_IMU_msg[6] = 0;
	Conti_IMU_msg[7] = 0;
	send_message(CAN_1, false, 0x070, 8, Conti_IMU_msg);

	steering_angle_output = (int16_t)(Conti_Steering_Angle * 10);
	Conti_Steering_Angle_msg[0] = (uint8_t)(steering_angle_output >> 8);
	Conti_Steering_Angle_msg[1] = (uint8_t)(steering_angle_output & 0xFF);
	Conti_Steering_Angle_msg[2] = 0;
	Conti_Steering_Angle_msg[3] = 0;
	Conti_Steering_Angle_msg[4] = 0;
	Conti_Steering_Angle_msg[5] = 0;
	Conti_Steering_Angle_msg[6] = 0;
	Conti_Steering_Angle_msg[7] = 0;
	send_message(CAN_1, false, 0x321, 8, Conti_Steering_Angle_msg);

	// snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f--FRONT--%2.1f \r\n              %2.1f--REAR---%2.1f\r\n", Conti_Wheel_Speed_FL, Conti_Wheel_Speed_FR, Conti_Wheel_Speed_RL, Conti_Wheel_Speed_RR);
	// serialPrint(debug_buffer);
	front_left_wheel_speed_output = (uint16_t)(Conti_Wheel_Speed_FL * 9.768);  //* 4.085
	front_right_wheel_speed_output = (uint16_t)(Conti_Wheel_Speed_FR * 9.768); // I think this should be 9.579
	rear_left_wheel_speed_output = (uint16_t)(Conti_Wheel_Speed_RL * 9.768);
	rear_right_wheel_speed_output = (uint16_t)(Conti_Wheel_Speed_RR * 9.768);

	Conti_Wheel_speed_msg[0] = (uint8_t)(front_left_wheel_speed_output & 0xFF);
	Conti_Wheel_speed_msg[1] = (uint8_t)(((front_right_wheel_speed_output & 0x0F)) << 4) | ((front_left_wheel_speed_output >> 8) & 0x0F);
	Conti_Wheel_speed_msg[2] = (uint8_t)(front_right_wheel_speed_output >> 4);
	Conti_Wheel_speed_msg[3] = (uint8_t)(rear_left_wheel_speed_output & 0xFF);
	Conti_Wheel_speed_msg[4] = (uint8_t)(((rear_right_wheel_speed_output & 0x0F)) << 4) | ((rear_left_wheel_speed_output >> 8) & 0x0F);
	Conti_Wheel_speed_msg[5] = (uint8_t)(rear_right_wheel_speed_output >> 4);
	Conti_Wheel_speed_msg[6] = 0;
	Conti_Wheel_speed_msg[7] = 0;
	// printf("%d\r\n", Conti_Wheel_speed_msg);
	send_message(CAN_1, false, 0x340, 8, Conti_Wheel_speed_msg);

	Conti_Brake_Pressure_msg[0] = 0;
	Conti_Brake_Pressure_msg[1] = (uint8_t)(((uint16_t)(Conti_Brake_Pressure_Front * 10)) & 0xff);
	Conti_Brake_Pressure_msg[2] = (uint8_t)(((((uint16_t)(Conti_Brake_Pressure_Rear * 10)) & 0x0F) << 4) | ((((uint16_t)(Conti_Brake_Pressure_Front * 10)) >> 8) & 0x0F));
	Conti_Brake_Pressure_msg[3] = (uint8_t)(((uint16_t)(Conti_Brake_Pressure_Rear * 10)) >> 4);
	Conti_Brake_Pressure_msg[4] = 0;
	Conti_Brake_Pressure_msg[5] = 0;
	Conti_Brake_Pressure_msg[6] = 0;
	Conti_Brake_Pressure_msg[7] = 0;
	send_message(CAN_1, false, 0x342, 8, Conti_Brake_Pressure_msg);
}

/* Run 50Hz Functions here - 20ms */
void events_50Hz()
{
	// IO BOX A
	uint16_t ioBA_AVI1 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI1 * 819, 0);	 // A/C
	uint16_t ioBA_AVI2 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI2 * 4095, 0); // FLLV
	uint16_t ioBA_AVI3 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI3 * 4095, 0); // FLLV
	uint16_t ioBA_AVI4 = (uint16_t)roundfloat_to_int32((float)0, 0);					 // Clutch
	uint8_t msgIOBA_1[8] = {ioBA_AVI1 >> 8, ioBA_AVI1, ioBA_AVI2 >> 8, ioBA_AVI2, ioBA_AVI3 >> 8, ioBA_AVI3, ioBA_AVI4 >> 8, ioBA_AVI4};
	send_message(CAN_1, false, 0x2C0, 8, msgIOBA_1);

	uint16_t ioBA_DPI1_DC = (uint16_t)roundfloat(haltech_IO_Box_A_DPI1_Duty, 1);
	uint16_t ioBA_DPI1_Period = (uint16_t)roundfloat(flws_2_hz, 1);
	// uint16_t ioBA_DPI1_Period = (uint32_t)frequency_Hz_to_period_10uS(haltech_IO_Box_A_DPI1_Hz); // Can be edited to take in a general 10uS resolution value instead of frequency conversion function
	uint16_t ioBA_DPI2_DC = (uint16_t)roundfloat(haltech_IO_Box_A_DPI2_Duty, 1);
	uint16_t ioBA_DPI2_Period = (uint16_t)roundfloat(frws_2_hz, 1);
	uint8_t msgIOBA_2[8] = {(uint8_t)(ioBA_DPI1_DC >> 2), (uint8_t)((ioBA_DPI1_DC & 0x03) << 6), (uint8_t)(ioBA_DPI1_Period >> 8), (uint8_t)(ioBA_DPI1_Period), (uint8_t)(ioBA_DPI2_DC >> 2), (uint8_t)((ioBA_DPI2_DC & 0x03) << 6), (uint8_t)(ioBA_DPI2_Period >> 8), (uint8_t)(ioBA_DPI2_Period)};
	send_message(CAN_1, false, 0x2C2, 8, msgIOBA_2);

	uint16_t ioBA_DPI3_DC = (uint16_t)roundfloat(haltech_IO_Box_A_DPI3_Duty, 1);
	uint16_t ioBA_DPI3_Period = (uint16_t)roundfloat(rlws_2_hz, 1);
	uint16_t ioBA_DPI4_DC = (uint16_t)roundfloat(haltech_IO_Box_A_DPI4_Duty, 1);
	uint16_t ioBA_DPI4_Period = (uint16_t)roundfloat(rrws_2_hz, 1);
	uint8_t msgIOBA_3[8] = {(uint8_t)(ioBA_DPI3_DC >> 2), (uint8_t)((ioBA_DPI3_DC & 0x03) << 6), (uint8_t)(ioBA_DPI3_Period >> 8), (uint8_t)(ioBA_DPI3_Period), (uint8_t)(ioBA_DPI4_DC >> 2), (uint8_t)((ioBA_DPI4_DC & 0x03) << 6), (uint8_t)(ioBA_DPI4_Period >> 8), (uint8_t)(ioBA_DPI4_Period)};
	send_message(CAN_1, false, 0x2C4, 8, msgIOBA_3);

	uint8_t msgIOBA_4[8] = {(uint8_t)((haltech_IO_Box_A_Status << 4) + haltech_IO_Box_A_ID_Conflict), (uint8_t)((haltech_IO_Box_A_Bootcode_Version << 2) + haltech_IO_Box_A_FW_Major_Version), haltech_IO_Box_A_FW_Minor_Version, haltech_IO_Box_A_FW_Bugfix_Version, haltech_IO_Box_A_FW_Release_Type, 0, 0, 0};
	send_message(CAN_1, false, 0x2C6, 8, msgIOBA_4);

	// IO BOX B
	uint16_t ioBB_AVI1 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI1 * 819, 0); // Sport Button
	uint16_t ioBB_AVI2 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI2 * 819, 0); // Cruise Buttons
	uint16_t ioBB_AVI3 = (uint16_t)roundfloat_to_int32(trimPos * 819, 0);				// Trim
	uint16_t ioBB_AVI4 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI4 * 819, 0); // Traction Control
	uint8_t msgIOBB_1[8] = {ioBB_AVI1 >> 8, ioBB_AVI1, ioBB_AVI2 >> 8, ioBB_AVI2, ioBB_AVI3 >> 8, ioBB_AVI3, ioBB_AVI4 >> 8, ioBB_AVI4};
	// uint8_t msgIOBB_1[8] = {ioBB_AVI1 >> 8, ioBB_AVI1, ioBB_AVI2 >> 8, ioBB_AVI2, ioBB_AVI3 >> 8, ioBB_AVI3, ioBB_AVI4 >> 8, ioBB_AVI4};
	send_message(CAN_1, false, 0x2C1, 8, msgIOBB_1);

	uint16_t ioBB_DPI1_DC = (uint16_t)roundfloat(haltech_IO_Box_B_DPI1_Duty, 1);
	uint16_t ioBB_DPI1_Period = (uint16_t)roundfloat(haltech_IO_Box_B_DPI1_Period, 2);
	uint16_t ioBB_DPI2_DC = (uint16_t)roundfloat(haltech_IO_Box_B_DPI2_Duty, 1);
	uint16_t ioBB_DPI2_Period = (uint16_t)roundfloat(haltech_IO_Box_B_DPI2_Period, 2);
	uint8_t msgIOBB_2[8] = {(uint8_t)(ioBB_DPI1_DC >> 2), (uint8_t)((ioBB_DPI1_DC & 0x03) << 6), (uint8_t)(ioBB_DPI1_Period >> 8), (uint8_t)(ioBB_DPI1_Period), (uint8_t)(ioBB_DPI2_DC >> 2), (uint8_t)((ioBB_DPI2_DC & 0x03) << 6), (uint8_t)(ioBB_DPI2_Period >> 8), (uint8_t)(ioBB_DPI2_Period)};
	send_message(CAN_1, false, 0x2C3, 8, msgIOBB_2);

	uint16_t ioBB_DPI3_DC = (uint16_t)roundfloat(haltech_IO_Box_B_DPI3_Duty, 1);
	uint16_t ioBB_DPI3_Period = (uint16_t)roundfloat(haltech_IO_Box_B_DPI3_Period, 2);
	uint16_t ioBB_DPI4_DC = (uint16_t)roundfloat(haltech_IO_Box_B_DPI4_Duty, 1);
	uint16_t ioBB_DPI4_Period = (uint16_t)roundfloat(haltech_IO_Box_B_DPI4_Period, 2);
	uint8_t msgIOBB_3[8] = {(uint8_t)(ioBB_DPI3_DC >> 2), (uint8_t)((ioBB_DPI3_DC & 0x03) << 6), (uint8_t)(ioBB_DPI3_Period >> 8), (uint8_t)(ioBB_DPI3_Period), (uint8_t)(ioBB_DPI4_DC >> 2), (uint8_t)((ioBB_DPI4_DC & 0x03) << 6), (uint8_t)(ioBB_DPI4_Period >> 8), (uint8_t)(ioBB_DPI4_Period)};
	send_message(CAN_1, false, 0x2C5, 8, msgIOBB_3);

	uint8_t msgIOBB_4[8] = {(uint8_t)((haltech_IO_Box_B_Status << 4) + haltech_IO_Box_B_ID_Conflict), (uint8_t)((haltech_IO_Box_B_Bootcode_Version << 2) + haltech_IO_Box_B_FW_Major_Version), haltech_IO_Box_B_FW_Minor_Version, haltech_IO_Box_B_FW_Bugfix_Version, haltech_IO_Box_B_FW_Release_Type, 0, 0, 0};
	send_message(CAN_1, false, 0x2C7, 8, msgIOBB_4);
}

/* Run 20Hz Functions here - 50ms */
void events_20Hz()
{
	// Enable EPAS Here
	if (engine_run == 1)
	{
		// TODO - Temp Engine, byte 7
		// engineData[0] = (uint8_t)0x61;
		// engineData[1] = (uint8_t)0xff;
		engineData[2] = (uint8_t)0x6e;
		engineData[3] = (uint8_t)0xd2;
		engineData[4] = (uint8_t)0x24;
		engineData[5] = (uint8_t)0x78;
		engineData[6] = (uint8_t)0xcc;
		engineData[7] = (uint8_t)0xa4;
		//  printf("Some shit: %2.1f\r\n", minirpm);
		int16_t temptomini = (uint16_t)roundfloat_to_int32(sa_temp + 100 + 48, 0);

		engineData[0] = (uint8_t)temptomini;
		engineData[1] = (uint8_t)temptomini;
		// printf("Some shit: %d\r\n", temptomini);
		send_message(CAN_2, false, 0x1d0, 8, engineData);
	}

	if (bc_state == 1 && ccplus_state == 1 && lastTrim > 200)
	{
		if (trimPos != trimMax)
		{
			trimPos += 1;
		}
		lastTrim = 0;
	}
	if (bc_state == 1 && ccminus_state == 1 && lastTrim > 200)
	{
		if (trimPos != trimMin)
		{
			trimPos -= 1;
		}
		lastTrim = 0;
	}
}

/* Run 10Hz Functions here - 100ms */
void events_10Hz()
{
	if (lastTrim < 5000)
	{
		lastTrim += 100;
	}
}

/* Run 5Hz Functions here */
void events_5Hz()
{
	toggleLED(LED_1);
}

/* Run 2Hz Functions here */
void events_2Hz()
{
}

/* Run 1Hz Functions here */
void events_1Hz()
{
	// snprintf(debug_buffer, sizeof(debug_buffer), "Haltech Temp: %2.1f\r\n", sa_temp);

	// snprintf(debug_buffer, sizeof(debug_buffer), "Engine Run: %d\r\n", engine_run);
	// // snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f and hz == %2.2f \r\n", wheel_speed_fl, flws_2_hz);
	// snprintf(debug_buffer, sizeof(debug_buffer), "Haltech RPM: %2.1f\r\n", engine_rpm);
	// // snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f and hz == %2.2f \r\n", wheel_speed_fl, flws_2_hz);
	// serialPrint(debug_buffer);
}