#include "msg_handler.hpp"

void MsgHandler::process_txclpt_callback()
{

	if ( !tx_msg_list.empty() && ongoing_fetch == false)
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
	else if (ongoing_fetch)
	{
		receive_data();
	}

}


void MsgHandler::process_rxclpt_callback()
{
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
				rxSingleack = AckMsg();

			}
			return;
		}
	}
	if(rxHeader.get_data()[0] == SerialID::MSG_START && ongoing_fetch == false)
	{
		uint8_t msg_id = rxHeader.get_data()[1];
		uint8_t msg_len = rxHeader.get_data()[2];
		switch (msg_id) {
			case SerialID::MSG_PRINT:
				ongoing_fetch = true;
				ack_msg_print(msg_len);
				break;
			case SerialID::MSG_DATA_1:
				if(msg_len == sizeof(float))
				{
					ongoing_fetch = true;
					ack_msg_data_1();
				}
				break;
			default:
				break;
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
			case SerialID::MSG_DATA_1:
				process_received_msg_data_1(incoming_data->get_data());
				break;
			default:
				break;
		}
		ongoing_fetch = false;

	}


}


void MsgHandler::send_print(const char* msg)
{
	if( strlen(msg) < UINT8_MAX)
	{
		tx_msg_list.push_back(std::make_shared<HeaderClass>(SerialID::MSG_PRINT,strlen(msg)));
		tx_msg_list.push_back(std::make_shared<PayloadClass>(msg,strlen(msg)));
		if( tx_msg_list.size() == 2)
		{
			transmit_front_msg();
		}
	}

}

void MsgHandler::send_float(float float_to_send)
{
	tx_msg_list.push_back(std::make_shared<HeaderClass>(SerialID::MSG_DATA_1,sizeof(float)));
	tx_msg_list.push_back(std::make_shared<PayloadClass>(&float_to_send,sizeof(float)));
	if( tx_msg_list.size() == 2)
	{
		transmit_front_msg();
	}

}

void MsgHandler::transmit_front_msg()
{
	if(ongoing_fetch == false)
	{
		HAL_UART_AbortReceive(huart);
	}
	HAL_UART_Transmit_DMA(huart, (uint8_t*)tx_msg_list.front()->get_data(), tx_msg_list.front()->get_data_size());
}

void MsgHandler::receive_ack()
{
	HAL_UART_Receive_DMA(huart, rxSingleack.get_data(), rxSingleack.get_data_size());
}
void MsgHandler::receive_data_header()
{
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PinState::GPIO_PIN_RESET);
	HAL_UART_Receive_DMA(huart, rxHeader.get_data(), rxHeader.get_data_size());
}
void MsgHandler::ack_msg_data_1()
{
	float dummy_data_1 = 0.0;
	incoming_data = std::make_unique<PayloadClass>(&dummy_data_1,sizeof(float));
	txSingleack.get_data()[0] = SerialID::MSG_ACK;
	txSingleack.get_data()[1] = SerialID::MSG_DATA_1;
	txSingleack.get_data()[2] = sizeof(float);
	HAL_UART_Transmit_DMA(huart, txSingleack.get_data(), txSingleack.get_data_size());

}
void MsgHandler::ack_msg_print(uint8_t msg_len)
{
	char dummy_print_msg[msg_len];
	char *dummy_print_msg_ptr = dummy_print_msg;
	incoming_data = std::make_unique<PayloadClass>(dummy_print_msg_ptr,msg_len);
	txSingleack.get_data()[0] = SerialID::MSG_ACK;
	txSingleack.get_data()[1] = SerialID::MSG_PRINT;
	txSingleack.get_data()[2] = msg_len;
	HAL_UART_Transmit_DMA(huart, txSingleack.get_data(), txSingleack.get_data_size());

}

void MsgHandler::receive_data()
{
	HAL_UART_Receive_DMA(huart, incoming_data->get_data(), incoming_data->get_data_size());
}

void MsgHandler::process_received_msg_data_1(uint8_t * data)
{
	float* float_ptr = reinterpret_cast<float*>(data);
	char msg[50];
	sprintf(msg,"received : %f",*float_ptr);
	send_print(msg);

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
		register_ongoing_fetch = false;
	}
}
