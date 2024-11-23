#include "msg_handler.hpp"

/*
  * @brief  called upon completion of HAL_UART_TRANSMIT_IT
*/
void MsgHandler::process_txclpt_callback()
{
	sending =false;
}

/*
  * @brief  called upon completion of HAL_UART_RECEIVE_IT
*/
void MsgHandler::process_rxclpt_callback()
{
	receiving = false;
	received_motor_speeds = true;
}

/*
  * @brief  write input_motor_speeds to uart (ie measured motor speeds)
  * @param input_motor_speeds , tab of float to send in rad/s 
*/
void MsgHandler::send_motor_speeds(float* input_motor_speeds)
{
	while(sending || receiving);
	sending = true;
	HAL_UART_Transmit_IT(huart, (uint8_t*)input_motor_speeds, 4*sizeof(float));
}

/*
  * @brief  prepare the read of motor_speeds from uart (ie command motor speeds)
*/
void MsgHandler::prepare_receive_motor_speeds()
{
	while(sending || receiving);
	receiving = true;
	HAL_UART_Receive_IT(huart, (uint8_t*)motor_speeds, 4*sizeof(float));
}

/*
  * @brief  get the received motor speeds from uart 
  * @param to_fill_motor_speeds , tab of float in rad/s to fill with received_data
  * @retval bool                , True if to_fill_motor_speeds was filled
  *                               (only when received_motor_speeds was toggled from rxclpt_callback),
  * 							  False if not (you should call again get_received_motor_speeds again until motor_speeds is received)
*/
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
