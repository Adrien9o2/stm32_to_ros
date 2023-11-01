#pragma once
#include <list>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include "stm32f4xx_hal.h"
#include "main.h"



class MsgHandler
{
	public:
		/*
		* @brief Constructor of MsgHandler
		* @param huart2, pointer to huart wich MsgHandler will handle 
		*/
		MsgHandler(UART_HandleTypeDef *huart2) : huart(huart2)
		{
			motor_speeds = new float[4];
			sending = false;
			receiving = false;
			received_motor_speeds = false;
		}
		/*
		* @brief Destructor of MsgHandler
		*/
		~MsgHandler() { delete [] motor_speeds;}
		void process_rxclpt_callback();
		void process_txclpt_callback();
		void prepare_receive_motor_speeds();
		void send_motor_speeds(float* input_motor_speeds);
		bool get_received_motor_speeds(float* to_fill_motor_speeds);
		/*
		* @brief Reset flags that may lock functions waiting those flags
		*/
		bool unlock_timeout() { receiving = false; sending = false; };
	private :
		UART_HandleTypeDef *huart;
		float* motor_speeds;
		bool received_motor_speeds;
		bool sending;
		bool receiving;
};
