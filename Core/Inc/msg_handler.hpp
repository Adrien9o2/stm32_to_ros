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
		MsgHandler(UART_HandleTypeDef *huart2) : huart(huart2),received_motor_speeds(false) ,ongoing_fetch(false),register_ongoing_fetch(false), rxHeader(SerialID::MSG_NO_ID,3)
		{
			motor_speeds = new float[4];
			launched = false;
		}
		~MsgHandler() { delete [] motor_speeds;}
		void process_rxclpt_callback();
		void process_txclpt_callback();
		void send_print(const char* msg);
		void send_motor_speeds(float* motor_speeds);
		void process_timeout(void);
		bool get_received_motor_speeds(float* to_fill_motor_speeds);
		void launch_handler() { if( launched == false){launched=true; receive_data_header();}};
	private :
		UART_HandleTypeDef *huart;
		void transmit_front_msg();
		void receive_ack();
		void receive_data_header();
		void receive_data();
		void ack_msg_print(uint8_t msg_len);
		void ack_msg_motor_speeds();
		void process_received_msg_print(uint8_t* data, uint8_t msg_len);
		void process_received_msg_motor_speeds(uint8_t* data);
		void check_tx_list();
		bool received_motor_speeds;
		bool ongoing_fetch;
		bool register_ongoing_fetch;
		AckMsg rxSingleack;
		AckMsg txSingleack;
		HeaderClass rxHeader;
		PayloadClass* incoming_data;
		std::list<std::shared_ptr<AbstractMsg>> tx_msg_list;
		std::shared_ptr<AbstractMsg> registered_msg;
		float* motor_speeds;
		bool launched;
		bool abort_clpt;
};
