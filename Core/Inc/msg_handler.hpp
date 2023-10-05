#pragma once
#include <list>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include "stm32f4xx_hal.h"
#include "main.h"

#include "msg.hpp"



class MsgHandler
{
	public:
		MsgHandler(UART_HandleTypeDef *huart2) : huart(huart2), ongoing_fetch(false),register_ongoing_fetch(false), rxHeader(SerialID::MSG_NO_ID,3) {};
		~MsgHandler() = default;
		void process_rxclpt_callback();
		void process_txclpt_callback();
		void send_print(const char* msg);
		void send_float(float float_to_send);
		void process_timeout(void);
	private :
		UART_HandleTypeDef *huart;
		void transmit_front_msg();
		void receive_ack();
		void receive_data_header();
		void receive_data();
		void ack_msg_print(uint8_t msg_len);
		void ack_msg_data_1();
		void process_received_msg_print(uint8_t* data, uint8_t msg_len);
		void process_received_msg_data_1(uint8_t* data);
		bool ongoing_fetch;
		bool register_ongoing_fetch;
		AckMsg rxSingleack;
		AckMsg txSingleack;
		HeaderClass rxHeader;
		std::unique_ptr<PayloadClass> incoming_data;
		std::list<std::shared_ptr<AbstractMsg>> tx_msg_list;
		std::shared_ptr<AbstractMsg> registered_msg;
};
