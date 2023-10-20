#include "msg_handler.hpp"

void MsgHandler::process_txclpt_callback()
{

	if ( !tx_msg_list.empty() && ongoing_fetch == false)
	{
		check_tx_list();
	}

}

void MsgHandler::check_tx_list()
{
	if( tx_msg_list.front()->get_type() == msg_type::payload)
	{
		tx_msg_list.pop_front();
		if ( !tx_msg_list.empty())
		{
			if(tx_msg_list.front()->get_type() == msg_type::header)
			{
				transmit_front_msg();
			}
		}
		else
		{
			receive_data_header();
		}
	}
	else if ( tx_msg_list.front()->get_type() == msg_type::header)
	{
		receive_ack();
	}
}


void MsgHandler::process_rxclpt_callback()
{
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PinState::GPIO_PIN_RESET);
	if ( !tx_msg_list.empty() && ongoing_fetch == false)
	{
		if(rxSingleack.get_data()[0] == SerialID::MSG_ACK)
		{
			if( tx_msg_list.front()->get_type() == msg_type::header
				&& tx_msg_list.front()->get_data()[1] == rxSingleack.get_data()[1]
				&& tx_msg_list.front()->get_data()[2] == rxSingleack.get_data()[2]
			)
			{
				tx_msg_list.pop_front();
				if( tx_msg_list.front()->get_type() == msg_type::payload)
				{
					transmit_front_msg();
				}

			}
			rxSingleack = AckMsg();
			return;
		}
	}
	else if (rxHeader.get_data()[0] == SerialID::MSG_START && ongoing_fetch == false)
	{
		uint8_t msg_id = rxHeader.get_data()[1];
		uint8_t msg_len = rxHeader.get_data()[2];
		switch (msg_id) {
			case SerialID::MSG_PRINT:
				ongoing_fetch = true;
				ack_msg_print(msg_len);
				break;
			case SerialID::MSG_MOTOR_SPEEDS:
				if(msg_len == 4*sizeof(float))
				{
					ongoing_fetch = true;
					ack_msg_motor_speeds();
				}
				break;
			default:
				break;
		}
		rxHeader.get_data()[0] = SerialID::MSG_NO_START;
		rxHeader.get_data()[1] = SerialID::MSG_NO_ID;
		rxHeader.get_data()[2] = SerialID::MSG_NO_SIZE;
		if( ongoing_fetch == false)
		{
			if( !tx_msg_list.empty() )
			{
				check_tx_list();
			}
			else
			{
				receive_data_header();
			}
		}
	}
	else if ( ongoing_fetch == true)
	{
		uint8_t msg_id = txSingleack.get_data()[1];
		uint8_t msg_len = txSingleack.get_data()[2];
		switch (msg_id) {
			case SerialID::MSG_PRINT:
				process_received_msg_print(incoming_data->get_data(),msg_len);
				break;
			case SerialID::MSG_MOTOR_SPEEDS:
				process_received_msg_motor_speeds(incoming_data->get_data());
				break;
			default:
				break;
		}
		ongoing_fetch = false;
		txSingleack = AckMsg();
		if( tx_msg_list.empty())
		{
			receive_data_header();
		}
		else
		{
			check_tx_list();
		}

	}
	else
	{
		if( tx_msg_list.empty())
		{
			receive_data_header();
		}
		else
		{
			check_tx_list();
		}
	}


}


void MsgHandler::send_print(const char* msg)
{
	if( strlen(msg) < UINT8_MAX)
	{
		tx_msg_list.push_back(std::make_shared<HeaderClass>(SerialID::MSG_PRINT,strlen(msg)));
		tx_msg_list.push_back(std::make_shared<PayloadClass>(msg,strlen(msg)));
		if( tx_msg_list.size() == 2 && ongoing_fetch == false)
		{
			transmit_front_msg();
		}
	}

}

void MsgHandler::send_motor_speeds(float* motor_speeds)
{
	tx_msg_list.push_back(std::make_shared<HeaderClass>(SerialID::MSG_MOTOR_SPEEDS,4*sizeof(float)));
	tx_msg_list.push_back(std::make_shared<PayloadClass>(motor_speeds,4*sizeof(float)));
	if( tx_msg_list.size() == 2 && ongoing_fetch == false)
	{
		transmit_front_msg();
	}

}

void MsgHandler::transmit_front_msg()
{
	HAL_UART_Abort(huart);
	HAL_UART_Transmit_IT(huart, (uint8_t*)tx_msg_list.front()->get_data(), tx_msg_list.front()->get_data_size());
}

void MsgHandler::receive_ack()
{
	HAL_UART_Abort(huart);
	HAL_UART_Receive_IT(huart, rxSingleack.get_data(), rxSingleack.get_data_size());
}
void MsgHandler::receive_data_header()
{
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PinState::GPIO_PIN_SET);
	HAL_UART_Abort(huart);
	HAL_UART_Receive_IT(huart, rxHeader.get_data(), rxHeader.get_data_size());
}
void MsgHandler::ack_msg_motor_speeds()
{
	float  dummy_data_1[4] = {0.0,0.0,0.0,0.0};
	if( incoming_data != NULL)
	{
		delete incoming_data;
	}
	incoming_data = new PayloadClass(&dummy_data_1,4*sizeof(float));
	txSingleack.get_data()[0] = SerialID::MSG_ACK;
	txSingleack.get_data()[1] = SerialID::MSG_MOTOR_SPEEDS;
	txSingleack.get_data()[2] = 4*sizeof(float);
	HAL_UART_Abort(huart);
	HAL_UART_Receive_IT(huart, incoming_data->get_data(), incoming_data->get_data_size());
	HAL_UART_Transmit_IT(huart, txSingleack.get_data(), txSingleack.get_data_size());

}
void MsgHandler::ack_msg_print(uint8_t msg_len)
{
	char dummy_print_msg[msg_len];
	char *dummy_print_msg_ptr = dummy_print_msg;
	if( incoming_data != NULL)
	{
		delete incoming_data;
	}
	incoming_data = new PayloadClass(dummy_print_msg_ptr,msg_len);
	txSingleack.get_data()[0] = SerialID::MSG_ACK;
	txSingleack.get_data()[1] = SerialID::MSG_PRINT;
	txSingleack.get_data()[2] = msg_len;
	HAL_UART_Abort(huart);
	HAL_UART_Transmit_IT(huart, txSingleack.get_data(), txSingleack.get_data_size());

}

void MsgHandler::receive_data()
{
	HAL_UART_Abort(huart);
	HAL_UART_Receive_IT(huart, incoming_data->get_data(), incoming_data->get_data_size());
}

void MsgHandler::process_received_msg_motor_speeds(uint8_t * data)
{
	memcpy(motor_speeds,data,4*sizeof(float));
	received_motor_speeds = true;
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


void MsgHandler::process_received_msg_print(uint8_t * data, uint8_t msg_len)
{
    char* charPtr = reinterpret_cast<char*>(data);
 	uint8_t max_len = std::min(UINT8_MAX, 50+msg_len);
	char msg[max_len];
	sprintf(msg,"received : %s",charPtr);
	send_print(msg);

}

void MsgHandler::process_timeout(void)
{

	if( ! tx_msg_list.empty())
	{
		if( ongoing_fetch)
		{
			if(register_ongoing_fetch)
			{
				ongoing_fetch = false;
				register_ongoing_fetch = false;
				txSingleack = AckMsg();

			}
			else
			{
				register_ongoing_fetch = true;
			}
		}
		else
		{
			if(registered_msg.get() == tx_msg_list.front().get() )
			{
				tx_msg_list.clear();
				registered_msg = nullptr;
				receive_data_header();
			}
			else
			{
				registered_msg = tx_msg_list.front();
			}
		}

	}
}
