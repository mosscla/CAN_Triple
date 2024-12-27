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
uint32_t brake_state = 0;
uint32_t clutch_state = 0;
float brake_pressure = 0;
uint32_t dsc_state = 0;
float fuel_lev_r = 0;
float fuel_lev_l = 0;
float fuel_lev = 0;

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

/* End Variable Declarations */

/* Startup Functions */
void events_Startup()
{
	setupCANbus(CAN_1, 1000000, NORMAL_MODE); // Haltech - 1mb
	setupCANbus(CAN_2, 500000, NORMAL_MODE);  // Mini PT - 500kb
	// setupCANbus(CAN_3, 1000000, NORMAL_MODE);
	setCAN_Termination(CAN_1, true);
	setCAN_Termination(CAN_2, false);
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
	if (Message.Bus == CAN_1) // Haltech
	{
		// serialPrint("CAN 1 - received a message..\r\n");
		// send_message(CAN_2, Message.is_extended_id, Message.arbitration_id, Message.dlc, Message.data);
	}

	if (Message.Bus == CAN_2) // MINI PT
	{
		if (Message.arbitration_id == 0x0CE) // Wheel Speeds - 206
		{
			uint32_t wheel_fl = (Message.data[1] << 8) | (Message.data[0]);
			wheel_speed_fl = process_float_value(wheel_fl, 0xFFFF, false, 0.0625, 0, 3);
			uint32_t wheel_fr = (Message.data[3] << 8) | (Message.data[2]);
			wheel_speed_fr = process_float_value(wheel_fr, 0xFFFF, false, 0.0625, 0, 3);
			uint32_t wheel_rl = (Message.data[5] << 8) | (Message.data[4]);
			wheel_speed_rl = process_float_value(wheel_rl, 0xFFFF, false, 0.0625, 0, 3);
			uint32_t wheel_rr = (Message.data[1] << 8) | (Message.data[6]);
			wheel_speed_rr = process_float_value(wheel_rr, 0xFFFF, false, 0.0625, 0, 3);
			haltech_IO_Box_A_DPI1_Duty = map_float(wheel_speed_fl, 0, 255, 0, 1000);
			haltech_IO_Box_A_DPI2_Duty = map_float(wheel_speed_fr, 0, 255, 0, 1000);
			haltech_IO_Box_A_DPI3_Duty = map_float(wheel_speed_rl, 0, 255, 0, 1000);
			haltech_IO_Box_A_DPI4_Duty = map_float(wheel_speed_rr, 0, 255, 0, 1000);
			if (wheel_speed_fl < 0.1)
			{
				flws_2_hz = 0;
			}
			else
			{
				flws_2_hz = 10000 / wheel_speed_fl;
			}

			if (wheel_speed_fr < 0.1)
			{
				frws_2_hz = 0;
			}
			else
			{
				frws_2_hz = 10000 / wheel_speed_fr;
			}

			if (wheel_speed_rl < 0.1)
			{
				rlws_2_hz = 0;
			}
			else
			{
				rlws_2_hz = 10000 / wheel_speed_rl;
			}

			if (wheel_speed_rr < 0.1)
			{
				rrws_2_hz = 0;
			}
			else
			{
				rrws_2_hz = 10000 / wheel_speed_rr;
			}

			// haltech_IO_Box_A_DPI1_Period = map_float(wheel_speed_fl, 0, 255, 50000, 1);
			// haltech_IO_Box_A_DPI2_Period = map_float(wheel_speed_fr, 0, 255, 50000, 1);
			// haltech_IO_Box_A_DPI3_Period = map_float(wheel_speed_rl, 0, 255, 50000, 1);
			// haltech_IO_Box_A_DPI4_Period = map_float(wheel_speed_rr, 0, 255, 50000, 1);
			snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f--FRONT--%2.1f \r\n              %2.1f--REAR---%2.1f\r\n", wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr);
			// snprintf(debug_buffer, sizeof(debug_buffer), "Wheel Speeds: %2.1f and hz == %2.2f \r\n", wheel_speed_fl, flws_2_hz);
			serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x315) // Sport Button - 789
		{
			sport_state = process_raw_value(Message.data[1], 0x7F);

			snprintf(debug_buffer, sizeof(debug_buffer), "Sport State: %i\r\n", sport_state);
			serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x1ee) // BC Button - 494
		{
			bc_state = process_raw_value(Message.data[0], 0x40);
			snprintf(debug_buffer, sizeof(debug_buffer), "BC    State: %i\r\n", bc_state);
			serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x2a6) // Rear Wiper Switch - 678
		{
			rw_state = process_raw_value(Message.data[0], 0x40);
			snprintf(debug_buffer, sizeof(debug_buffer), "RW    State: %i\r\n", rw_state);
			serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x194) // Cruise Buttons - 404
		{
			ccplus_state = process_raw_value(Message.data[2], 0x01);
			ccminus_state = process_raw_value(Message.data[2], 0x04);
			ccset_state = process_raw_value(Message.data[2], 0x80); // or 0x10 (cancel?)
			ccresume_state = process_raw_value(Message.data[2], 0x40);

			if (ccplus_state == 1)
			{
				snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: + %i\r\n", ccplus_state);
				serialPrint(debug_buffer);
				haltech_IO_Box_A_AVI2 = 1.0;
			}
			else if (ccminus_state == 1)
			{
				snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: - %i\r\n", ccminus_state);
				serialPrint(debug_buffer);
				haltech_IO_Box_A_AVI2 = 2.0;
			}
			else if (ccset_state == 1)
			{
				snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: set %i\r\n", ccset_state);
				serialPrint(debug_buffer);
				haltech_IO_Box_A_AVI2 = 3.0;
			}
			else if (ccresume_state == 1)
			{
				snprintf(debug_buffer, sizeof(debug_buffer), "Cruise: resume %i\r\n", ccresume_state);
				serialPrint(debug_buffer);
				haltech_IO_Box_A_AVI2 = 4.0;
			}
		}
		if (Message.arbitration_id == 0x19e) // Brake Pressure - 414
		{
			uint32_t brake_pressure_sig = (Message.data[6]);
			brake_pressure = process_float_value(brake_pressure_sig, 0xFF, true, 1, 0, 3);
			haltech_IO_Box_A_AVI1 = map_float(brake_pressure, 0, 255, 0, 5);
			snprintf(debug_buffer, sizeof(debug_buffer), "Brake Pressure: + %2.1f\r\n", brake_pressure);
			serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x19e) // Traction Control Button - 414
		{
			dsc_state = process_raw_value(Message.data[1], 0x10);
			snprintf(debug_buffer, sizeof(debug_buffer), "DSC   State: %i\r\n", dsc_state);
			serialPrint(debug_buffer);
		}
		if (Message.arbitration_id == 0x330) // Fuel Level - 816
		{
			uint32_t fuel_lev_sig = Message.data[3];
			fuel_lev = process_float_value(fuel_lev_sig, 0xFF, false, 1, 0, 3);
			uint32_t fuel_lev_l_sig = Message.data[4];
			fuel_lev_l = process_float_value(fuel_lev_l_sig, 0xFF, false, 0.5, 0, 3);
			uint32_t fuel_lev_r_sig = Message.data[5];
			fuel_lev_r = process_float_value(fuel_lev_r_sig, 0xFF, false, 0.5, 0, 3);
			haltech_IO_Box_A_AVI3 = map_float(fuel_lev, 0, 999, 0, 5);
			snprintf(debug_buffer, sizeof(debug_buffer), "Fuel Total: %2.1f Fuel Left:%2.1f Fuel Right:%2.1f\r\n", fuel_lev, fuel_lev_l, fuel_lev_r);
			serialPrint(debug_buffer);
		}
	}
	if (Message.Bus == CAN_3)
	{
	}
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
}

/* Run 100Hz Functions here */
void events_100Hz()
{
}

/* Run 50Hz Functions here - 20ms */
void events_50Hz()
{
	// IO BOX A
	uint16_t ioBA_AVI1 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI1 * 5 / 4095, 0);
	uint16_t ioBA_AVI2 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI2 * 5 / 4095, 0);
	uint16_t ioBA_AVI3 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI1 * 5 / 4095, 0);
	uint16_t ioBA_AVI4 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_A_AVI1 * 5 / 4095, 0);
	uint8_t msgIOBA_1[8] = {(uint8_t)ioBA_AVI1 >> 8, (uint8_t)ioBA_AVI1, (uint8_t)ioBA_AVI2 >> 8, (uint8_t)ioBA_AVI2, (uint8_t)ioBA_AVI3 >> 8, (uint8_t)ioBA_AVI3, (uint8_t)ioBA_AVI4 >> 8, (uint8_t)ioBA_AVI4};
	send_message(CAN_1, false, 0x2C0, 8, msgIOBA_1);

	uint16_t ioBA_DPI1_DC = (uint16_t)roundfloat(haltech_IO_Box_A_DPI1_Duty, 1);
	// uint16_t ioBA_DPI1_Period = (uint16_t)roundfloat(flws_2_hz, 1);
	uint16_t ioBA_DPI1_Period = (uint32_t)frequency_Hz_to_period_10uS(haltech_IO_Box_A_DPI1_Hz); // Can be edited to take in a general 10uS resolution value instead of frequency conversion function
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
	uint16_t ioBB_AVI1 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI1 * 5 / 4095, 0);
	uint16_t ioBB_AVI2 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI2 * 5 / 4095, 0);
	uint16_t ioBB_AVI3 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI1 * 5 / 4095, 0);
	uint16_t ioBB_AVI4 = (uint16_t)roundfloat_to_int32(haltech_IO_Box_B_AVI1 * 5 / 4095, 0);
	uint8_t msgIOBB_1[8] = {(uint8_t)ioBB_AVI1 >> 8, (uint8_t)ioBB_AVI1, (uint8_t)ioBB_AVI2 >> 8, (uint8_t)ioBB_AVI2, (uint8_t)ioBB_AVI3 >> 8, (uint8_t)ioBB_AVI3, (uint8_t)ioBB_AVI4 >> 8, (uint8_t)ioBB_AVI4};
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

	// uint8_t msgIOBB_4[8] = {(uint8_t)((haltech_IO_Box_B_Status << 4) + haltech_IO_Box_B_ID_Conflict), (uint8_t)((haltech_IO_Box_B_Bootcode_Version << 2) + haltech_IO_Box_B_FW_Major_Version), haltech_IO_Box_B_FW_Minor_Version, haltech_IO_Box_B_FW_Bugfix_Version, haltech_IO_Box_B_FW_Release_Type, 0, 0, 0};
	// send_message(CAN_1, false, 0x2C7, 8, msgIOBB_4);
}

/* Run 20Hz Functions here - 50ms */
void events_20Hz()
{
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
}