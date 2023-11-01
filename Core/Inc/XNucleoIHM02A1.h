/**
 ******************************************************************************
 *
 * COPYRIGHT(c) 2014 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef __X_NUCLEO_IHM02A1_CLASS_H
#define __X_NUCLEO_IHM02A1_CLASS_H


/* Includes ------------------------------------------------------------------*/

#include <stdexcept>
#include <type_traits>
#include "main.h"
#include "L6470.h"
#include "StepperMotor.h"


/* Classes -------------------------------------------------------------------*/
class XNucleoIHM02A1
{

public:
	/**
	 * @brief Constructor.
	 * @param init_0        pointer to the initialization structure of the first motor.
	 * @param init_1        pointer to the initialization structure of the second motor.
	 * @param spi           SPI handler used for communication with the shield.
	 * @param standby_reset_port port name of the STBY\RST pin of the xnucleo shield.
	 * @param standby_reset_pin pin of the STBY\RST pin of the xnucleo shield.
	 * @param ssel_port     port name of the SSEL pin used for SPI communication.
	 * @param ssel_pin     pin of the SSEL used for SPI communication.
	 */
	XNucleoIHM02A1(L6470_init_t *init_0, L6470_init_t *init_1, SPI_HandleTypeDef *spi, GPIO_TypeDef* standby_reset_port  ,uint16_t standby_reset_pin,GPIO_TypeDef* ssel_port , uint16_t ssel_pin) :spi(spi), standby_reset_port(standby_reset_port), standby_reset_pin(standby_reset_pin)
	{
	    HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_SET);


	    if( shield_1::count == 0)
	    {
	    	motor_drivers = new abstractL6470*[L6470DAISYCHAINSIZE];
	    	for( int i = 0; i < L6470DAISYCHAINSIZE; i ++)
	    	{
				motor_drivers[i] = new L6470<shield_1>(standby_reset_port, standby_reset_pin, ssel_port, ssel_pin, spi);
	    	}
	    }
	    else if( shield_2::count == 0)
	    {
	    	motor_drivers = new abstractL6470*[L6470DAISYCHAINSIZE];
	    	for( int i = 0; i < L6470DAISYCHAINSIZE; i ++)
	    	{
				motor_drivers[i] = new L6470<shield_2>(standby_reset_port, standby_reset_pin, ssel_port, ssel_pin, spi);
	    	}
	    }
	    else if( shield_3::count == 0)
	    {
	    	motor_drivers = new abstractL6470*[L6470DAISYCHAINSIZE];
	    	for( int i = 0; i < L6470DAISYCHAINSIZE; i ++)
	    	{
				motor_drivers[i] = new L6470<shield_3>(standby_reset_port, standby_reset_pin, ssel_port, ssel_pin, spi);
	    	}
	    }
	    else
	    {
	    	throw std::runtime_error("Cannot create more than 3 shields");
	    }




	    /* Initializing the motor_drivers. */
	    init_motor_drivers[0] = init_0;
	    init_motor_drivers[1] = init_1;
	    if (!init()) {
	        /*Do nothing*/
	    	throw std::runtime_error("init failed");
	    }
	}
    /**
     * @brief Destructor.
     */
    ~XNucleoIHM02A1(void) {
    	for( int i = 0; i < L6470DAISYCHAINSIZE; i ++)
    	{
    		delete motor_drivers[i];
    	}
    	delete motor_drivers;
    }

    /**
     * @brief Initializing the X_NUCLEO_IHM02A1 board.
     * @retval true if initialization is successful, false otherwise.
     */
    bool init(void) const noexcept
    {
        /* Disable the L6470. */

    	HAL_GPIO_WritePin(standby_reset_port, standby_reset_pin, GPIO_PinState::GPIO_PIN_RESET);

    	/* Wait for at least t_STBY,min */
    	HAL_Delay(1);

    	/* Enable the L6470. */
    	HAL_GPIO_WritePin(standby_reset_port, standby_reset_pin, GPIO_PinState::GPIO_PIN_SET);

    	/* Wait for at least t_logicwu */
    	HAL_Delay(1);



        return init_all_motor_drivers();
    }


    /**
     * @brief  Getting the array of motor_drivers.
     * @param  None.
     * @retval The array of motor_drivers.
     */
    abstractL6470 **get_motor_drivers(void) const noexcept
    {
        return motor_drivers;
    }

    /**
      * @brief  Performing the actions set on the motors with calls to a number of
      *         "Prepare<Action>()" methods, one for each motor of the daisy-chain.
      * @param  None.
      * @retval A pointer to the results returned by the motor_drivers, i.e. an
      *         integer value for each of them.
      */
    uint32_t* perform_prepared_actions(void) noexcept
    {
        /* Performing pre-actions, if needed. */
        for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
            /*
               "GetPosition()" is needed by "PrepareSetMark()" at the time when the
               prepared actions get performed.
            */
            if (motor_drivers[m]->get_prepared_action() == prepared_action_t::PREPARED_SET_MARK) {
                motor_drivers[m]->prepare_set_mark((uint32_t) motor_drivers[m]->get_position());
            }
        }
        
        /* Performing the prepared actions and getting back raw data. */
        uint8_t *raw_data = motor_drivers[0]->perform_prepared_actions();

        /* Processing raw data. */
        for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
            results[m] = motor_drivers[m]->get_result(raw_data);
        }

        /* Returning results. */
        return results;
    }

protected:

    /**
     * @brief  Initialize the L6470 motor drivers.
     * @retval true if initialization is successful, false otherwise.
     */
    bool init_all_motor_drivers() const noexcept
    {
    	for( int i = 0; i < L6470DAISYCHAINSIZE; i ++)
    	{
    		if(motor_drivers[i]->init((void*) init_motor_drivers[i]) != COMPONENT_OK)
    		{
    			return false;
    		}
    	}
    	return true;

    }


    /*** Component's Instance Variables ***/

    /* IO Device. */
    SPI_HandleTypeDef *spi;

    /* motor_drivers. */
    abstractL6470 **motor_drivers;
    
    /* motor_drivers' initialization. */
    L6470_init_t *init_motor_drivers[L6470DAISYCHAINSIZE];

    /* Results of prepared actions. */
    uint32_t results[L6470DAISYCHAINSIZE];

    /* Standby/reset pin. */
    GPIO_TypeDef* standby_reset_port;
    uint16_t standby_reset_pin;
};

#endif /* __X_NUCLEO_IHM02A1_CLASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
