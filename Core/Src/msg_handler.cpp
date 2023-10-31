#include "msg_handler.hpp"

void MsgHandler::process_txclpt_callback()
{
	sending =false;
}

void MsgHandler::process_rxclpt_callback()
{
	receiving = false;
	received_motor_speeds = true;
}


void MsgHandler::send_motor_speeds(float* input_motor_speeds)
{
	while(sending || receiving);
	sending = true;
	HAL_UART_Transmit_IT(huart, (uint8_t*)input_motor_speeds, 4*sizeof(float));
}


void MsgHandler::prepare_receive_motor_speeds()
{
	while(sending || receiving);
	receiving = true;
	HAL_UART_Receive_IT(huart, (uint8_t*)motor_speeds, 4*sizeof(float));
}


bool MsgHandler::get_received_motor_speeds(float* to_fill_motor_speeds)
{
	if( received_motor_speeds )
	{
		if( motor_speeds != NULL)
		{
			memcpy(to_fill_motor_speeds,motor_speeds,4*sizeof(float));
			received_motor_speeds = false;
			return true;
		}
	}
	return false;
}
