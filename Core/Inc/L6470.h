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


#ifndef __L6470_CLASS_H
#define __L6470_CLASS_H

/* Includes -------------------------------------------------------------------*/
#include <cstring>
#include <type_traits>
#include <stdexcept>

#include "main.h"

#include "L6470_def.h"

#include "StepperMotor.h"
using namespace StepperMotor;

/* Classes -------------------------------------------------------------------*/


//We use dummy class shield_1, shield_2, shield_3 in order to template a L6470 class
//The action of templating allows to separate static variables shared among instances of the same class
//Indeed L6470 class used by st  with XNucleoIHM02A1 was not meant to use more than one shield even though it has the hardware capabilities.
class shield_1
{
	public:
		shield_1(){count++;}
		~shield_1() = default;
		static const uint8_t shield_nbr = 1;
		static uint8_t count;
};

class shield_2
{
public:
	shield_2(){count++;}
	~shield_2() = default;
	static const uint8_t shield_nbr = 2;
	static uint8_t count;
};

class shield_3
{
public:
	shield_3(){count++;}
	~shield_3() = default;
	static const uint8_t shield_nbr = 3;
	static uint8_t count;
};

typedef enum
{
    PREPARED_NO_ACTION = 0,
    PREPARED_GET_POSITION,
    PREPARED_GET_MARK,
    PREPARED_GET_SPEED,
    PREPARED_GET_MAX_SPEED,
    PREPARED_GET_MIN_SPEED,
    PREPARED_GET_ACCELERATION,
    PREPARED_GET_DECELERATION,
    PREPARED_GET_DIRECTION,
    PREPARED_SET_MARK
} prepared_action_t;

//In order to manipulate each templated L6470 class with the same pointer we define an absract class (polymorphism)
class abstractL6470
{
	public:
		abstractL6470();
		virtual ~abstractL6470();
	    virtual int init(void *init) = 0;
	    virtual unsigned int get_status(void) = 0;
	    virtual unsigned int get_parameter(unsigned int parameter) = 0;
	    virtual signed int get_position(void) = 0;
	    virtual signed int get_mark(void) = 0;
	    virtual unsigned int get_speed(void) = 0;
	    virtual unsigned int get_max_speed(void) = 0;
	    virtual unsigned int get_min_speed(void) = 0;
	    virtual unsigned int get_acceleration(void) = 0;
	    virtual unsigned int get_deceleration(void) = 0;
	    virtual StepperMotor::direction_t get_direction(void) = 0;
	    virtual void set_parameter(unsigned int parameter, unsigned int value) = 0;
	    virtual void set_home(void) = 0;
	    virtual void set_mark(void) = 0;
	    virtual void set_mark(signed int position) = 0;
	    virtual bool set_max_speed(unsigned int speed) = 0;
	    virtual bool set_min_speed(unsigned int speed) = 0;
	    virtual bool set_acceleration(unsigned int acceleration) = 0;
	    virtual bool set_deceleration(unsigned int deceleration) = 0;
	    virtual bool set_step_mode(StepperMotor::step_mode_t step_mode) = 0;
	    virtual void go_to(signed int position) = 0;
	    virtual void go_to(signed int position, StepperMotor::direction_t direction) = 0;
	    virtual void go_home(void) = 0;
	    virtual void go_mark(void) = 0;
	    virtual void go_until(eL6470_ActId_t action, StepperMotor::direction_t direction, unsigned int speed) = 0;
	    virtual void run(StepperMotor::direction_t direction) = 0;
	    virtual void run(StepperMotor::direction_t direction, unsigned int speed) = 0;
	    virtual void move(StepperMotor::direction_t direction, unsigned int steps) = 0;
	    virtual void soft_stop(void) = 0;
	    virtual void hard_stop(void) = 0;
	    virtual void soft_hiz(void) = 0;
	    virtual void hard_hiz(void) = 0;
	    virtual void wait_while_active(void) = 0;
	    virtual void step_clock(StepperMotor::direction_t direction) = 0;
	    virtual void release_sw(eL6470_ActId_t action, StepperMotor::direction_t direction) = 0;
	    virtual void reset_device(void) = 0;
	    virtual void prepare_get_status(void) = 0;
	    virtual void prepare_get_parameter(unsigned int parameter) = 0;
	    virtual void prepare_get_position(void) = 0;
	    virtual void prepare_get_mark(void) = 0;
	    virtual void prepare_get_speed(void) = 0;
	    virtual void prepare_get_max_speed(void) = 0;
	    virtual void prepare_get_min_speed(void) = 0;
	    virtual void prepare_get_acceleration(void) = 0;
	    virtual void prepare_get_deceleration(void) = 0;
	    virtual void prepare_get_direction(void) = 0;
	    virtual void prepare_set_parameter(unsigned int parameter, unsigned int value) = 0;
	    virtual void prepare_set_home(void) = 0;
	    virtual void prepare_set_mark(void) = 0;
	    virtual void prepare_set_mark(signed int position) = 0;
	    virtual void prepare_set_speed(unsigned int speed) = 0;
	    virtual void prepare_set_max_speed(unsigned int speed) = 0;
	    virtual void prepare_set_min_speed(unsigned int speed) = 0;
	    virtual void prepare_set_acceleration(unsigned int acceleration) = 0;
	    virtual void prepare_set_deceleration(unsigned int deceleration) = 0;
	    virtual void prepare_go_to(signed int position) = 0;
	    virtual void prepare_go_to(signed int position, StepperMotor::direction_t direction) = 0;
	    virtual void prepare_go_home(void) = 0;
	    virtual void prepare_go_mark(void) = 0;
	    virtual void prepare_go_until(eL6470_ActId_t L6470_ActId, StepperMotor::direction_t direction, unsigned int speed) = 0;
	    virtual void prepare_run(StepperMotor::direction_t direction) = 0;
	    virtual void prepare_run(StepperMotor::direction_t direction, unsigned int speed) = 0;
	    virtual void prepare_move(StepperMotor::direction_t direction, unsigned int steps) = 0;
	    virtual void prepare_soft_stop(void) = 0;
	    virtual void prepare_hard_stop(void) = 0;
	    virtual void prepare_soft_hiz(void) = 0;
	    virtual void prepare_hard_hiz(void) = 0;
	    virtual void prepare_step_clock(StepperMotor::direction_t direction) = 0;
	    virtual void prepare_release_sw(eL6470_ActId_t action, StepperMotor::direction_t direction) = 0;
	    virtual void prepare_reset_device(void) = 0;
	    virtual uint8_t* perform_prepared_actions(void) = 0;
	    virtual prepared_action_t get_prepared_action(void) = 0;
	    virtual int32_t get_result(uint8_t *raw_data) = 0;
	    virtual void L6470_DISABLE(void) = 0;
	protected:
	    virtual int32_t  L6470_AbsPos_2_Position(uint32_t AbsPos) = 0;
	    virtual uint32_t L6470_Position_2_AbsPos(int32_t Position) = 0;
	    virtual float    L6470_Speed_2_Step_s(uint32_t Speed) = 0;
	    virtual uint32_t L6470_Step_s_2_Speed(float Step_s) = 0;
	    virtual float    L6470_Acc_2_Step_s2(uint16_t Acc) = 0;
	    virtual uint16_t L6470_Step_s2_2_Acc(float Step_s2) = 0;
	    virtual float    L6470_Dec_2_Step_s2(uint16_t Dec) = 0;
	    virtual uint16_t L6470_Step_s2_2_Dec(float Step_s2) = 0;
	    virtual float    L6470_MaxSpeed_2_Step_s(uint16_t MaxSpeed) = 0;
	    virtual uint16_t L6470_Step_s_2_MaxSpeed(float Step_s) = 0;
	    virtual float    L6470_MinSpeed_2_Step_s(uint16_t MinSpeed) = 0;
	    virtual uint16_t L6470_Step_s_2_MinSpeed(float Step_s) = 0;
	    virtual float    L6470_FsSpd_2_Step_s(uint16_t FsSpd) = 0;
	    virtual uint16_t L6470_Step_s_2_FsSpd(float Step_s) = 0;
	    virtual float    L6470_IntSpeed_2_Step_s(uint16_t IntSpeed) = 0;
	    virtual uint16_t L6470_Step_s_2_IntSpeed(float Step_s) = 0;
	    virtual float    L6470_StSlp_2_s_Step(uint8_t StSlp) = 0;
	    virtual uint8_t  L6470_s_Step_2_StSlp(float s_Step) = 0;
	    virtual float    L6470_FnSlpAcc_2_s_Step(uint8_t FnSlpAcc) = 0;
	    virtual uint8_t  L6470_s_Step_2_FnSlpAcc(float s_Step) = 0;
	    virtual float    L6470_FnSlpDec_2_s_Step(uint8_t FnSlpDec) = 0;
	    virtual uint8_t  L6470_s_Step_2_FnSlpDec(float s_Step) = 0;
	    virtual float    L6470_OcdTh_2_mA(uint8_t OcdTh) = 0;
	    virtual uint8_t  L6470_mA_2_OcdTh(float mA) = 0;
	    virtual float    L6470_StallTh_2_mA(uint8_t StallTh) = 0;
	    virtual uint8_t  L6470_mA_2_StallTh(float mA) = 0;
	    virtual status_t L6470_Config(void *init) = 0;
	    virtual void     L6470_SetParam(eL6470_RegId_t L6470_RegId, uint32_t Value) = 0;
	    virtual uint32_t L6470_GetParam(eL6470_RegId_t L6470_RegId) = 0;
	    virtual void     L6470_Run(eL6470_DirId_t L6470_DirId, uint32_t Speed) = 0;
	    virtual void     L6470_StepClock(eL6470_DirId_t L6470_DirId) = 0;
	    virtual void     L6470_Move(eL6470_DirId_t L6470_DirId, uint32_t N_Step) = 0;
	    virtual void     L6470_GoTo(uint32_t AbsPos) = 0;
	    virtual void     L6470_GoToDir(eL6470_DirId_t L6470_DirId, uint32_t AbsPos) = 0;
	    virtual void     L6470_GoUntil(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed) = 0;
	    virtual void     L6470_ReleaseSW(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId) = 0;
	    virtual void     L6470_GoHome(void) = 0;
	    virtual void     L6470_GoMark(void) = 0;
	    virtual void     L6470_ResetPos(void) = 0;
	    virtual void     L6470_ResetDevice(void) = 0;
	    virtual void     L6470_SoftStop(void) = 0;
	    virtual void     L6470_HardStop(void) = 0;
	    virtual void     L6470_SoftHiZ(void) = 0;
	    virtual void     L6470_HardHiZ(void) = 0;
	    virtual uint16_t L6470_GetStatus(void) = 0;
	    virtual void     L6470_PrepareSetParam(eL6470_RegId_t L6470_RegId, uint32_t Value) = 0;
	    virtual void     L6470_PrepareGetParam(eL6470_RegId_t L6470_RegId) = 0;
	    virtual void     L6470_PrepareRun(eL6470_DirId_t L6470_DirId, uint32_t Speed) = 0;
	    virtual void     L6470_PrepareStepClock(eL6470_DirId_t L6470_DirId) = 0;
	    virtual void     L6470_PrepareMove(eL6470_DirId_t L6470_DirId, uint32_t N_Step) = 0;
	    virtual void     L6470_PrepareGoTo(uint32_t AbsPos) = 0;
	    virtual void     L6470_PrepareGoToDir(eL6470_DirId_t L6470_DirId, uint32_t AbsPos) = 0;
	    virtual void     L6470_PrepareGoUntil(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed) = 0;
	    virtual void     L6470_PrepareReleaseSW(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId) = 0;
	    virtual void     L6470_PrepareGoHome(void) = 0;
	    virtual void     L6470_PrepareGoMark(void) = 0;
	    virtual void     L6470_PrepareResetPos(void) = 0;
	    virtual void     L6470_PrepareResetDevice(void) = 0;
	    virtual void     L6470_PrepareSoftStop(void) = 0;
	    virtual void     L6470_PrepareHardStop(void) = 0;
	    virtual void     L6470_PrepareSoftHiZ(void) = 0;
	    virtual void     L6470_PrepareHardHiZ(void) = 0;
	    virtual void     L6470_PrepareGetStatus(void) = 0;
	    virtual uint8_t* L6470_PerformPreparedApplicationCommand(void) = 0;
	    virtual void     L6470_DaisyChainCommand(uint8_t* pL6470_DaisyChainSpiTxStruct, uint8_t* pL6470_DaisyChainSpiRxStruct) = 0;
	    virtual uint32_t L6470_ExtractReturnedData(uint8_t* pL6470_DaisyChainSpiRxStruct, uint8_t LengthByte) = 0;
	    virtual uint8_t  L6470_CheckStatusRegisterFlag(uint8_t L6470_StatusRegisterFlagId) = 0;
	    virtual uint8_t* L6470_GetRegisterName(uint8_t id) = 0;
	    virtual void     L6470_ResetAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg) = 0;
	    virtual void     L6470_FillAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3) = 0;
	    virtual void     L6470_PrepareAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3) = 0;
	    virtual void     L6470_PrepareDaisyChainCommand(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, uint8_t* pL6470_DaisyChainSpiTxStruct) = 0;
	    virtual int L6470_round(float f) = 0;
	    virtual status_t Read(uint8_t* pBuffer, uint16_t NumBytesToRead) = 0;
	    virtual status_t Write(uint8_t* pBuffer, uint16_t NumBytesToWrite) = 0;
	    virtual status_t ReadWrite(uint8_t* pBufferToRead, uint8_t* pBufferToWrite, uint16_t NumBytes) = 0;
	    virtual void L6470_SPI_Communication(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) = 0;


		uint8_t shield_id;

};


/**
 * @brief Class representing a L6470 component.
 */
template <class shield_x>
class L6470 : public abstractL6470
{
public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param standby_reset port name of the STBY\RST pin of the component.
     * @param standby_reset pin name of the STBY\RST pin of the component.
     * @param ssel_port     port name of the SSEL pin of the SPI device to be used for communication.
     * @param ssel_pin      pin name of the SSEL pin of the SPI device to be used for communication.
     * @param spi           SPI handler to be used for communication.
     */
    L6470(GPIO_TypeDef* standby_reset_port, uint16_t standby_reset_pin,GPIO_TypeDef* ssel_port , uint16_t ssel_pin, SPI_HandleTypeDef *spi) : standby_reset_port(standby_reset_port), standby_reset_pin(standby_reset_pin), ssel_port(ssel_port),ssel_pin(ssel_pin) , spi(spi)
    {

    	if(std::is_same<shield_x,shield_1>::value)
    	{
    		shield_id = 1;
    		shield_1::count++;
    	}
    	else if (std::is_same<shield_x,shield_2>::value)
    	{
    		shield_id = 2;
    		shield_2::count++;
    	}
    	else if (std::is_same<shield_x,shield_3>::value)
    	{
    		shield_id = 3;
    		shield_3::count++;
    	}
    	else
    	{
    		throw std::runtime_error("template of L6470 must be shield_1, shield_2 or shield_3");
    	}
        L6470_Register = &_L6470_Register[0];
        L6470_ApplicationCommand = &_L6470_ApplicationCommand[0];
        L6470_Direction = &_L6470_Direction[0];
        L6470_ACT = &_L6470_ACT[0];
        pL6470_StatusRegister = &L6470_StatusRegister;
        prepared_action = PREPARED_NO_ACTION;
        L6470_Id = number_of_devices++;
        L6470_DaisyChain_HalfPrepared = ZERO_F;
        memset(L6470_AppCmdPkg, 0, L6470DAISYCHAINSIZE * sizeof(sL6470_AppCmdPkg_t));
        memset(L6470_DaisyChainSpiTxStruct, 0, L6470MAXSPICMDBYTESIZE * L6470DAISYCHAINSIZE * sizeof(uint8_t));
        memset(L6470_DaisyChainSpiRxStruct, 0, L6470MAXSPICMDBYTESIZE * L6470DAISYCHAINSIZE * sizeof(uint8_t));
    }

    /**
     * @brief Destructor.
     */
    ~L6470(void) = default;


    /*** Public Component Related Methods ***/

    /**
     * @brief  Initializing the component.
     * @param  init Pointer to device specific initalization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int init(void *init)
    {
        return (int) L6470_Config((void *) init);
    }

    /**
     * @brief  Getting the status.
     * @param  None.
     * @retval The status.
     */
    virtual unsigned int get_status(void)
    {
        return (unsigned int) L6470_GetStatus();
    }

    /**
     * @brief  Getting a parameter.
     * @param  parameter A parameter's register address.
     * @retval The parameter's value.
     * @note   The parameter can be one of the following:
     *           + L6470_ABS_POS_ID
     *           + L6470_EL_POS_ID
     *           + L6470_MARK_ID
     *           + L6470_SPEED_ID
     *           + L6470_ACC_ID
     *           + L6470_DEC_ID
     *           + L6470_MAX_SPEED_ID
     *           + L6470_MIN_SPEED_ID
     *           + L6470_FS_SPD_ID
     *           + L6470_KVAL_HOLD_ID
     *           + L6470_KVAL_RUN_ID
     *           + L6470_KVAL_ACC_ID
     *           + L6470_KVAL_DEC_ID
     *           + L6470_INT_SPEED_ID
     *           + L6470_ST_SLP_ID
     *           + L6470_FN_SLP_ACC_ID
     *           + L6470_FN_SLP_DEC_ID
     *           + L6470_K_THERM_ID
     *           + L6470_ADC_OUT_ID
     *           + L6470_OCD_TH_ID
     *           + L6470_STALL_TH_ID
     *           + L6470_STEP_MODE_ID
     *           + L6470_ALARM_EN_ID
     *           + L6470_CONFIG_ID
     *           + L6470_STATUS_ID
     */
    virtual unsigned int get_parameter(unsigned int parameter)
    {
        return (unsigned int) L6470_GetParam((eL6470_RegId_t) parameter);
    }

    /**
     * @brief  Getting the position.
     * @param  None.
     * @retval The position.
     */
    virtual signed int get_position(void)
    {
        return (signed int) L6470_AbsPos_2_Position((uint32_t) L6470_GetParam((eL6470_RegId_t) L6470_ABS_POS_ID));
    }

    /**
     * @brief  Getting the marked position.
     * @param  None.
     * @retval The marked position.
     */
    virtual signed int get_mark(void)
    {
        return (signed int) L6470_AbsPos_2_Position((uint32_t) L6470_GetParam((eL6470_RegId_t) L6470_MARK_ID));
    }

    /**
     * @brief  Getting the current speed in pps.
     * @param  None.
     * @retval The current speed in pps.
     */
    virtual unsigned int get_speed(void)
    {
        return L6470_round(L6470_Speed_2_Step_s((unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_SPEED_ID)));
    }

    /**
     * @brief  Getting the maximum speed in pps.
     * @param  None.
     * @retval The maximum speed in pps.
     */
    virtual unsigned int get_max_speed(void)
    {
        return L6470_round(L6470_MaxSpeed_2_Step_s((unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_MAX_SPEED_ID)));
    }

    /**
     * @brief  Getting the minimum speed in pps.
     * @param  None.
     * @retval The minimum speed in pps.
     */
    virtual unsigned int get_min_speed(void)
    {
        return L6470_round(L6470_MinSpeed_2_Step_s((unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_MIN_SPEED_ID)));
    }

    /**
     * @brief  Getting the acceleration in pps^2.
     * @param  None.
     * @retval The acceleration in pps^2.
     */
    virtual unsigned int get_acceleration(void)
    {
        return L6470_round(L6470_Acc_2_Step_s2((unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_ACC_ID)));
    }

    /**
     * @brief  Getting the deceleration in pps^2.
     * @param  None.
     * @retval The deceleration in pps^2.
     */
    virtual unsigned int get_deceleration(void)
    {
        return L6470_round(L6470_Dec_2_Step_s2((unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_DEC_ID)));
    }

    /**
     * @brief  Getting the direction of rotation.
     * @param  None.
     * @retval The direction of rotation.
     */
    virtual StepperMotor::direction_t get_direction(void)
    {
        return (StepperMotor::direction_t) (L6470_CheckStatusRegisterFlag((eL6470_StatusRegisterFlagId_t) DIR_ID) == 1 ? StepperMotor::FWD : StepperMotor::BWD);
    }

    /**
     * @brief   Setting a parameter.
     * @param   parameter A parameter's register address.
     * @param   value The parameter's value.
     * @retval  None.
     * @note    The parameter can be one of the following:
     *           + L6470_ABS_POS_ID
     *           + L6470_EL_POS_ID
     *           + L6470_MARK_ID
     *           + L6470_SPEED_ID
     *           + L6470_ACC_ID
     *           + L6470_DEC_ID
     *           + L6470_MAX_SPEED_ID
     *           + L6470_MIN_SPEED_ID
     *           + L6470_FS_SPD_ID
     *           + L6470_KVAL_HOLD_ID
     *           + L6470_KVAL_RUN_ID
     *           + L6470_KVAL_ACC_ID
     *           + L6470_KVAL_DEC_ID
     *           + L6470_INT_SPEED_ID
     *           + L6470_ST_SLP_ID
     *           + L6470_FN_SLP_ACC_ID
     *           + L6470_FN_SLP_DEC_ID
     *           + L6470_K_THERM_ID
     *           + L6470_ADC_OUT_ID
     *           + L6470_OCD_TH_ID
     *           + L6470_STALL_TH_ID
     *           + L6470_STEP_MODE_ID
     *           + L6470_ALARM_EN_ID
     *           + L6470_CONFIG_ID
     *           + L6470_STATUS_ID
     * @warning Some registers can only be written in particular conditions (see L6470's datasheet).
     *          Any attempt to write one of those registers when the conditions are not satisfied
     *          causes the command to be ignored and the NOTPERF_CMD flag to rise at the end of the
     *          last argument byte. Any attempt to set an inexistent register (wrong address value)
     *          causes the command to be ignored and the WRONG_CMD flag to rise.
     *          For example, setting some parameters requires first to disable the power bridge;
     *          this can be done through the soft_hiz() method.
     *          They are the following:
     *           + L6470_ABS_POS_ID
     *           + L6470_EL_POS_ID
     *           + L6470_SPEED_ID
     *           + L6470_ACC_ID
     *           + L6470_DEC_ID
     *           + L6470_MAX_SPEED_ID
     *           + L6470_MIN_SPEED_ID
     *           + L6470_INT_SPEED_ID
     *           + L6470_ST_SLP_ID
     *           + L6470_FN_SLP_ACC_ID
     *           + L6470_FN_SLP_DEC_ID
     *           + L6470_ADC_OUT_ID
     *           + L6470_STEP_MODE_ID
     *           + L6470_CONFIG_ID
     *           + L6470_STATUS_ID
     */
    virtual void set_parameter(unsigned int parameter, unsigned int value)
    {
        L6470_SetParam((eL6470_RegId_t) parameter, (uint32_t) value);
    }

    /**
     * @brief  Setting the current position to be the home position.
     * @param  None.
     * @retval None.
     */
    virtual void set_home(void)
    {
        L6470_ResetPos();
    }

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void set_mark(void)
    {
        L6470_SetParam((eL6470_RegId_t) L6470_MARK_ID, (uint32_t) L6470_GetParam((eL6470_RegId_t) L6470_ABS_POS_ID));
    }

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  position The given position.
     * @retval None.
     */
    virtual void set_mark(signed int position)
    {
        L6470_SetParam((eL6470_RegId_t) L6470_MARK_ID, (uint32_t) L6470_Position_2_AbsPos((int32_t) position));
    }

    /**
     * @brief  Setting the maximum speed in pps.
     * @param  speed The maximum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_max_speed(unsigned int speed)
    {
        L6470_SetParam((eL6470_RegId_t) L6470_MAX_SPEED_ID, (uint32_t) L6470_Step_s_2_MaxSpeed((float) speed));
        return true;
    }

    /**
     * @brief  Setting the minimum speed in pps.
     * @param  speed The minimum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_min_speed(unsigned int speed)
    {
        L6470_SetParam((eL6470_RegId_t) L6470_MIN_SPEED_ID, (uint32_t) L6470_Step_s_2_MinSpeed((float) speed));
        return true;
    }

    /**
     * @brief  Setting the acceleration in pps^2.
     * @param  acceleration The acceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_acceleration(unsigned int acceleration)
    {
        L6470_SetParam((eL6470_RegId_t) L6470_ACC_ID, (uint32_t) L6470_Step_s2_2_Acc((float) acceleration));
        return true;
    }

    /**
     * @brief  Setting the deceleration in pps^2.
     * @param  deceleration The deceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_deceleration(unsigned int deceleration)
    {
        L6470_SetParam((eL6470_RegId_t) L6470_DEC_ID, (uint32_t) L6470_Step_s2_2_Dec((float) deceleration));
        return true;
    }

    /**
     * @brief   Setting the Step Mode.
     * @param   step_mode The Step Mode.
     * @retval "true" in case of success, "false" otherwise.
     * @warning Setting the step mode implies first disabling the power bridge through
     *          the soft_hiz() method.
     * @warning Every time step mode is changed, the values of the home
     *          and mark positions lose meaning and are reset.
     */
    virtual bool set_step_mode(step_mode_t step_mode)
    {
        if ((eMotorStepMode_t) step_mode > MICROSTEP_1_128) {
            return false;
        }
        soft_hiz();
        L6470_SetParam((eL6470_RegId_t) L6470_STEP_MODE_ID, (eMotorStepMode_t) step_mode);
        return true;
    }

    /**
     * @brief  Going to a specified position through the shortest path.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void go_to(signed int position)
    {
        L6470_GoTo((uint32_t) L6470_Position_2_AbsPos((int32_t) position));
    }

    /**
     * @brief  Going to a specified position imposing the desired direction.
     * @param  position The desired position.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void go_to(signed int position, StepperMotor::direction_t direction)
    {
        L6470_GoToDir((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (uint32_t) L6470_Position_2_AbsPos((int32_t) position));
    }

    /**
     * @brief  Going to the home position.
     * @param  None.
     * @retval None.
     */
    virtual void go_home(void)
    {
        L6470_GoHome();
    }

    /**
     * @brief  Going to the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void go_mark(void)
    {
        L6470_GoMark();
    }

    /**
     * @brief  Running at the given speed imposing the desired direction until
     *         an external switch turn-on event occurs.
     * @param  action The identifier of the action about the absolute position.
     * @param  position The desired position.
     * @param  speed The speed value in pps.
     * @retval None.
     * @note   The identifier of the action about the absolute position can be
     *         one of the following:
     *           + L6470_ACT_RST_ID: the absolute position is reset;
     *           + L6470_ACT_CPY_ID: the absolute position is set as the marked position.
     */
    virtual void go_until(eL6470_ActId_t action, StepperMotor::direction_t direction, unsigned int speed)
    {
        L6470_GoUntil((eL6470_ActId_t) action, (eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) L6470_Step_s_2_Speed((float) speed));
    }

    /**
     * @brief  Running towards a specified direction.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(StepperMotor::direction_t direction)
    {
        L6470_Run((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_MAX_SPEED_ID));
    }

    /**
     * @brief  Running towards a specified direction at the given speed.
     * @param  direction The direction of rotation.
     * @param  speed The speed value in pps.
     * @retval None.
     */
    virtual void run(StepperMotor::direction_t direction, unsigned int speed)
    {
        L6470_Run((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) L6470_Step_s_2_Speed((float) speed));
    }

    /**
     * @brief  Moving towards a specified direction for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     */
    virtual void move(StepperMotor::direction_t direction, unsigned int steps)
    {
        L6470_Move((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) steps);
    }

    /**
     * @brief  Stopping the motor through an immediate deceleration up to zero speed.
     *         The used deceleration value is the one stored in the "DECEL" register.
     * @param  None.
     * @retval None.
     */
    virtual void soft_stop(void)
    {
        L6470_SoftStop();
    }

    /**
     * @brief  Stopping the motor through an immediate infinite deceleration.
     * @param  None.
     * @retval None.
     */
    virtual void hard_stop(void)
    {
        L6470_HardStop();
    }

    /**
     * @brief  Disabling the power bridge after performing a deceleration to zero.
     *         The used deceleration value is the one stored in the "DECEL" register.
     * @param  None.
     * @retval None.
     */
    virtual void soft_hiz(void)
    {
        L6470_SoftHiZ();
    }

    /**
     * @brief  Disabling the power bridge immediately.
     * @param  None.
     * @retval None.
     */
    virtual void hard_hiz(void)
    {
        L6470_HardHiZ();
    }

    /**
     * @brief  Waiting while the motor is active.
     * @param  None.
     * @retval None.
     */
    virtual void wait_while_active(void)
    {
        while (L6470_CheckStatusRegisterFlag(BUSY_ID) == 0);
    }

    /**
     * @brief   Switching to step-clock mode.
     * @param   direction The direction of rotation.
     * @retval  None.
     * @warning Setting the step-clock mode implies first disabling the power bridge through
     *          the soft_hiz() method.
     */
    virtual void step_clock(StepperMotor::direction_t direction)
    {
        soft_hiz();
        L6470_StepClock((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID));
    }

    /**
      * @brief  Doing a motion at minimum speed imposing a specified direction
      *         until the SW is released.
      * @param  action The identifier of the action about the absolute position.
      * @param  direction The direction of rotation.
      * @note   The identifier of the action about the absolute position can be
      *         one of the following:
      *           + L6470_ACT_RST_ID: the absolute position is reset;
      *           + L6470_ACT_CPY_ID: the absolute position is set as the marked position.
      */
    virtual void release_sw(eL6470_ActId_t action, StepperMotor::direction_t direction)
    {
        L6470_ReleaseSW((eL6470_ActId_t) action, (eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID));
    }

    /**
      * @brief  Resetting the device to power-up conditions.
      * @param  None.
      * @retval None.
      */
    virtual void reset_device(void)
    {
        L6470_ResetDevice();
    }

    /**
     * @brief  Preparing the command to get the status.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_status(void)
    {
        L6470_PrepareGetStatus();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to get a parameter.
     * @param  parameter A parameter's register address.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     *         The parameter can be one of the following:
     *           + L6470_ABS_POS_ID
     *           + L6470_EL_POS_ID
     *           + L6470_MARK_ID
     *           + L6470_SPEED_ID
     *           + L6470_ACC_ID
     *           + L6470_DEC_ID
     *           + L6470_MAX_SPEED_ID
     *           + L6470_MIN_SPEED_ID
     *           + L6470_FS_SPD_ID
     *           + L6470_KVAL_HOLD_ID
     *           + L6470_KVAL_RUN_ID
     *           + L6470_KVAL_ACC_ID
     *           + L6470_KVAL_DEC_ID
     *           + L6470_INT_SPEED_ID
     *           + L6470_ST_SLP_ID
     *           + L6470_FN_SLP_ACC_ID
     *           + L6470_FN_SLP_DEC_ID
     *           + L6470_K_THERM_ID
     *           + L6470_ADC_OUT_ID
     *           + L6470_OCD_TH_ID
     *           + L6470_STALL_TH_ID
     *           + L6470_STEP_MODE_ID
     *           + L6470_ALARM_EN_ID
     *           + L6470_CONFIG_ID
     *           + L6470_STATUS_ID
     */
    virtual void prepare_get_parameter(unsigned int parameter)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) parameter);
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to get the position.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_position(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_ABS_POS_ID);
        prepared_action = PREPARED_GET_POSITION;
    }

    /**
     * @brief  Preparing the command to get the marked position.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_mark(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_MARK_ID);
        prepared_action = PREPARED_GET_MARK;
    }

    /**
     * @brief  Preparing the command to get the current speed in pps.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_speed(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_SPEED_ID);
        prepared_action = PREPARED_GET_SPEED;
    }

    /**
     * @brief  Preparing the command to get the maximum speed in pps.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_max_speed(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_MAX_SPEED_ID);
        prepared_action = PREPARED_GET_MAX_SPEED;
    }

    /**
     * @brief  Preparing the command to get the minimum speed in pps.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_min_speed(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_MIN_SPEED_ID);
        prepared_action = PREPARED_GET_MIN_SPEED;
    }

    /**
     * @brief  Preparing the command to get the acceleration in pps^2.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_acceleration(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_ACC_ID);
        prepared_action = PREPARED_GET_ACCELERATION;
    }

    /**
     * @brief  Preparing the command to get the deceleration in pps^2.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_deceleration(void)
    {
        L6470_PrepareGetParam((eL6470_RegId_t) L6470_DEC_ID);
        prepared_action = PREPARED_GET_DECELERATION;
    }

    /**
     * @brief  Preparing the command to get the direction of rotation.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_get_direction(void)
    {
        if (!L6470_DaisyChain_HalfPrepared) {
            /* To avoid deleting the previous entered command. */
            L6470_DaisyChain_HalfPrepared = ONE_F;
            /* Resetting commands. */
            L6470_ResetAppCmdPkg(L6470_AppCmdPkg);
        }
        prepared_action = PREPARED_GET_DIRECTION;
    }

    /**
     * @brief  Preparing the command to set a parameter.
     * @param  parameter A parameter's register address.
     * @param  value The parameter's value.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     *         The parameter can be one of the following:
     *           + L6470_ABS_POS_ID
     *           + L6470_EL_POS_ID
     *           + L6470_MARK_ID
     *           + L6470_SPEED_ID
     *           + L6470_ACC_ID
     *           + L6470_DEC_ID
     *           + L6470_MAX_SPEED_ID
     *           + L6470_MIN_SPEED_ID
     *           + L6470_FS_SPD_ID
     *           + L6470_KVAL_HOLD_ID
     *           + L6470_KVAL_RUN_ID
     *           + L6470_KVAL_ACC_ID
     *           + L6470_KVAL_DEC_ID
     *           + L6470_INT_SPEED_ID
     *           + L6470_ST_SLP_ID
     *           + L6470_FN_SLP_ACC_ID
     *           + L6470_FN_SLP_DEC_ID
     *           + L6470_K_THERM_ID
     *           + L6470_ADC_OUT_ID
     *           + L6470_OCD_TH_ID
     *           + L6470_STALL_TH_ID
     *           + L6470_STEP_MODE_ID
     *           + L6470_ALARM_EN_ID
     *           + L6470_CONFIG_ID
     *           + L6470_STATUS_ID
     * @warning Some registers can only be written in particular conditions (see L6470's datasheet).
     *          Any attempt to write one of those registers when the conditions are not satisfied
     *          causes the command to be ignored and the NOTPERF_CMD flag to rise at the end of the
     *          last argument byte. Any attempt to set an inexistent register (wrong address value)
     *          causes the command to be ignored and the WRONG_CMD flag to rise.
     *          For example, setting some parameters requires first to disable the power bridge;
     *          this can be done through the soft_hiz() method.
     *          They are the following:
     *           + L6470_ABS_POS_ID
     *           + L6470_EL_POS_ID
     *           + L6470_SPEED_ID
     *           + L6470_ACC_ID
     *           + L6470_DEC_ID
     *           + L6470_MAX_SPEED_ID
     *           + L6470_MIN_SPEED_ID
     *           + L6470_INT_SPEED_ID
     *           + L6470_ST_SLP_ID
     *           + L6470_FN_SLP_ACC_ID
     *           + L6470_FN_SLP_DEC_ID
     *           + L6470_ADC_OUT_ID
     *           + L6470_STEP_MODE_ID
     *           + L6470_CONFIG_ID
     *           + L6470_STATUS_ID
     */
    virtual void prepare_set_parameter(unsigned int parameter, unsigned int value)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) parameter, (uint32_t) value);
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the current position to be
     *         the home position.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_home(void)
    {
        L6470_PrepareResetPos();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the current position to be
     *         the marked position.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_mark(void)
    {
        /*
           Set "0" now as marked position. This value will be replaced by the
           actual position read at the time when the prepared actions will be
           performed.
        */
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_MARK_ID, 0);
        prepared_action = PREPARED_SET_MARK;
    }

    /**
     * @brief  Preparing the command to set the given position to be
     *         the marked position.
     * @param  position The given position.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_mark(signed int position)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_MARK_ID, (uint32_t) L6470_Position_2_AbsPos((int32_t) position));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the current speed in pps.
     * @param  speed The current speed in pps.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_speed(unsigned int speed)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_SPEED_ID, (uint32_t) L6470_Step_s_2_Speed((float) speed));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the maximum speed in pps.
     * @param  speed The maximum speed in pps.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_max_speed(unsigned int speed)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_MAX_SPEED_ID, (uint32_t) L6470_Step_s_2_MaxSpeed((float) speed));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the minimum speed in pps.
     * @param  speed The minimum speed in pps.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_min_speed(unsigned int speed)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_MIN_SPEED_ID, (uint32_t) L6470_Step_s_2_MinSpeed((float) speed));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the acceleration in pps^2.
     * @param  acceleration The acceleration in pps^2.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_acceleration(unsigned int acceleration)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_ACC_ID, (uint32_t) L6470_Step_s2_2_Acc((float) acceleration));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to set the deceleration in pps^2.
     * @param  deceleration The deceleration in pps^2.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_set_deceleration(unsigned int deceleration)
    {
        L6470_PrepareSetParam((eL6470_RegId_t) L6470_DEC_ID, (uint32_t) L6470_Step_s2_2_Dec((float) deceleration));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to go to a specified position.
     * @param  position The desired position.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_go_to(signed int position)
    {
        L6470_PrepareGoTo((uint32_t) L6470_Position_2_AbsPos((int32_t) position));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to go to a specified position
     *         imposing the desired direction.
     * @param  position The desired position.
     * @param  direction The direction of rotation.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_go_to(signed int position, StepperMotor::direction_t direction)
    {
        L6470_PrepareGoToDir((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (uint32_t) L6470_Position_2_AbsPos((int32_t) position));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to go to the home position.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_go_home(void)
    {
        L6470_PrepareGoHome();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to go to the marked position.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_go_mark(void)
    {
        L6470_PrepareGoMark();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to run at the given speed imposing the desired
     *         direction until an external switch turn-on event occurs.
     * @param  action The identifier of the action about the absolute position.
     * @param  position The desired position.
     * @param  speed The speed value in pps.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     *         The identifier of the action about the absolute position can be
     *         one of the following:
     *           + L6470_ACT_RST_ID: the absolute position is reset;
     *           + L6470_ACT_CPY_ID: the absolute position is set as the marked position.
     */
    virtual void prepare_go_until(eL6470_ActId_t L6470_ActId, StepperMotor::direction_t direction, unsigned int speed)
    {
        L6470_PrepareGoUntil((eL6470_ActId_t) L6470_ActId, (eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) L6470_Step_s_2_Speed((float) speed));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to run towards a specified direction
     *         at the maximum speed.
     * @param  direction The direction of rotation.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_run(StepperMotor::direction_t direction)
    {
        L6470_PrepareRun((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) L6470_GetParam((eL6470_RegId_t) L6470_MAX_SPEED_ID));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to run towards a specified direction
     *         at the given speed.
     * @param  direction The direction of rotation.
     * @param  speed The speed value in pps.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_run(StepperMotor::direction_t direction, unsigned int speed)
    {
        L6470_PrepareRun((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) L6470_Step_s_2_Speed((float) speed));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to move towards a specified direction
     *         for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_move(StepperMotor::direction_t direction, unsigned int steps)
    {
        L6470_PrepareMove((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID), (unsigned int) steps);
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to stop the motor.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_soft_stop(void)
    {
        L6470_PrepareSoftStop();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to stop the motor and disabling the power bridge.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_hard_stop(void)
    {
        L6470_PrepareHardStop();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to disable the power bridge after performing
     *         a deceleration to zero.
     *         The used deceleration value is the one stored in the "DECEL" register.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_soft_hiz(void)
    {
        L6470_PrepareSoftHiZ();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief  Preparing the command to disable the power bridge immediately.
     * @param  None.
     * @retval None.
     * @note   The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_hard_hiz(void)
    {
        L6470_PrepareHardHiZ();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
     * @brief   Preparing the command to switch to step-clock mode.
     * @param   direction The direction of rotation.
     * @retval  None.
     * @warning Setting the step-clock mode requires an explicit action by the user to first
     *          disable the power bridge through the soft_hiz() method.
     * @note    The command will be sent by issuing "perform_action()".
     */
    virtual void prepare_step_clock(StepperMotor::direction_t direction)
    {
        L6470_PrepareStepClock((eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
      * @brief  Preparing the command to do a motion at minimum speed
      *         imposing a specified direction until the SW is released.
      * @param  action The identifier of the action about the absolute position.
      * @param  direction The direction of rotation.
      * @retval None.
      * @note   The command will be sent by issuing "perform_action()".
      *         The identifier of the action about the absolute position can be
      *         one of the following:
      *           + L6470_ACT_RST_ID: the absolute position is reset;
      *           + L6470_ACT_CPY_ID: the absolute position is set as the marked position.
      */
    virtual void prepare_release_sw(eL6470_ActId_t action, StepperMotor::direction_t direction)
    {
        L6470_PrepareReleaseSW((eL6470_ActId_t) action, (eL6470_DirId_t) (direction == StepperMotor::FWD ? L6470_DIR_FWD_ID : L6470_DIR_REV_ID));
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
      * @brief  Preparing the command to reset the device to power-up conditions.
      * @param  None.
      * @retval None.
      * @note   The command will be sent by issuing "perform_action()".
      */
    virtual void prepare_reset_device(void)
    {
        L6470_PrepareResetDevice();
        prepared_action = PREPARED_NO_ACTION;
    }

    /**
      * @brief    Performing all the actions prepared on the components
      *           of the daisy-chain.
      * @param    None.
      * @retval   The raw data returned by the components.
      */
    uint8_t* perform_prepared_actions(void)
    {
        return L6470_PerformPreparedApplicationCommand();
    }

    /**
      * @brief    Getting the prepared action.
      * @param    None
      * @retval   The prepared action.
      */
    prepared_action_t get_prepared_action(void)
    {
        return prepared_action;
    }

    /**
      * @brief    Converting the raw data received by the component according to
      *           the action performed on it.
      * @param    raw_data The received raw data.
      * @retval   The result of the action performed.
      */
    int32_t get_result(uint8_t *raw_data)
    {
        switch (prepared_action) {
            case PREPARED_GET_POSITION:
                return L6470_AbsPos_2_Position(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_ABS_POS_ID].LengthByte));

            case PREPARED_GET_MARK:
                return L6470_AbsPos_2_Position(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_MARK_ID].LengthByte));

            case PREPARED_GET_SPEED:
                return L6470_round(L6470_Speed_2_Step_s(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_SPEED_ID].LengthByte)));
    
            case PREPARED_GET_MAX_SPEED:
                return L6470_round(L6470_MaxSpeed_2_Step_s(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_MAX_SPEED_ID].LengthByte)));
    
            case PREPARED_GET_MIN_SPEED:
                return L6470_round(L6470_MinSpeed_2_Step_s(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_MIN_SPEED_ID].LengthByte)));
    
            case PREPARED_GET_ACCELERATION:
                return L6470_round(L6470_Acc_2_Step_s2(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_ACC_ID].LengthByte)));
    
            case PREPARED_GET_DECELERATION:
                return L6470_round(L6470_Dec_2_Step_s2(L6470_ExtractReturnedData(raw_data, L6470_Register[L6470_DEC_ID].LengthByte)));

            case PREPARED_GET_DIRECTION:
                return (int32_t) (StepperMotor::direction_t) (L6470_CheckStatusRegisterFlag((eL6470_StatusRegisterFlagId_t) DIR_ID) == 1 ? StepperMotor::FWD : StepperMotor::BWD);

            default:
            case PREPARED_NO_ACTION:
                return 0;
        }
    }


    /*
     * Puts the device in standby mode.
     */
    void L6470_ENABLE(void)
    {
    	HAL_GPIO_WritePin(standby_reset_port, standby_reset_pin, GPIO_PinState::GPIO_PIN_SET);
    }

    /*
     * Puts the device in reset mode.
     */
    void L6470_DISABLE(void)
    {
    	HAL_GPIO_WritePin(standby_reset_port, standby_reset_pin, GPIO_PinState::GPIO_PIN_RESET);
    }

protected:


    int32_t  L6470_AbsPos_2_Position(uint32_t AbsPos);
    uint32_t L6470_Position_2_AbsPos(int32_t Position);
    float    L6470_Speed_2_Step_s(uint32_t Speed);
    uint32_t L6470_Step_s_2_Speed(float Step_s);
    float    L6470_Acc_2_Step_s2(uint16_t Acc);
    uint16_t L6470_Step_s2_2_Acc(float Step_s2);
    float    L6470_Dec_2_Step_s2(uint16_t Dec);
    uint16_t L6470_Step_s2_2_Dec(float Step_s2);
    float    L6470_MaxSpeed_2_Step_s(uint16_t MaxSpeed);
    uint16_t L6470_Step_s_2_MaxSpeed(float Step_s);
    float    L6470_MinSpeed_2_Step_s(uint16_t MinSpeed);
    uint16_t L6470_Step_s_2_MinSpeed(float Step_s);
    float    L6470_FsSpd_2_Step_s(uint16_t FsSpd);
    uint16_t L6470_Step_s_2_FsSpd(float Step_s);
    float    L6470_IntSpeed_2_Step_s(uint16_t IntSpeed);
    uint16_t L6470_Step_s_2_IntSpeed(float Step_s);
    float    L6470_StSlp_2_s_Step(uint8_t StSlp);
    uint8_t  L6470_s_Step_2_StSlp(float s_Step);
    float    L6470_FnSlpAcc_2_s_Step(uint8_t FnSlpAcc);
    uint8_t  L6470_s_Step_2_FnSlpAcc(float s_Step);
    float    L6470_FnSlpDec_2_s_Step(uint8_t FnSlpDec);
    uint8_t  L6470_s_Step_2_FnSlpDec(float s_Step);
    float    L6470_OcdTh_2_mA(uint8_t OcdTh);
    uint8_t  L6470_mA_2_OcdTh(float mA);
    float    L6470_StallTh_2_mA(uint8_t StallTh);
    uint8_t  L6470_mA_2_StallTh(float mA);
    status_t L6470_Config(void *init);
    void     L6470_SetParam(eL6470_RegId_t L6470_RegId, uint32_t Value);
    uint32_t L6470_GetParam(eL6470_RegId_t L6470_RegId);
    void     L6470_Run(eL6470_DirId_t L6470_DirId, uint32_t Speed);
    void     L6470_StepClock(eL6470_DirId_t L6470_DirId);
    void     L6470_Move(eL6470_DirId_t L6470_DirId, uint32_t N_Step);
    void     L6470_GoTo(uint32_t AbsPos);
    void     L6470_GoToDir(eL6470_DirId_t L6470_DirId, uint32_t AbsPos);
    void     L6470_GoUntil(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed);
    void     L6470_ReleaseSW(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId);
    void     L6470_GoHome(void);
    void     L6470_GoMark(void);
    void     L6470_ResetPos(void);
    void     L6470_ResetDevice(void);
    void     L6470_SoftStop(void);
    void     L6470_HardStop(void);
    void     L6470_SoftHiZ(void);
    void     L6470_HardHiZ(void);
    uint16_t L6470_GetStatus(void);
    void     L6470_PrepareSetParam(eL6470_RegId_t L6470_RegId, uint32_t Value);
    void     L6470_PrepareGetParam(eL6470_RegId_t L6470_RegId);
    void     L6470_PrepareRun(eL6470_DirId_t L6470_DirId, uint32_t Speed);
    void     L6470_PrepareStepClock(eL6470_DirId_t L6470_DirId);
    void     L6470_PrepareMove(eL6470_DirId_t L6470_DirId, uint32_t N_Step);
    void     L6470_PrepareGoTo(uint32_t AbsPos);
    void     L6470_PrepareGoToDir(eL6470_DirId_t L6470_DirId, uint32_t AbsPos);
    void     L6470_PrepareGoUntil(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed);
    void     L6470_PrepareReleaseSW(eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId);
    void     L6470_PrepareGoHome(void);
    void     L6470_PrepareGoMark(void);
    void     L6470_PrepareResetPos(void);
    void     L6470_PrepareResetDevice(void);
    void     L6470_PrepareSoftStop(void);
    void     L6470_PrepareHardStop(void);
    void     L6470_PrepareSoftHiZ(void);
    void     L6470_PrepareHardHiZ(void);
    void     L6470_PrepareGetStatus(void);
    uint8_t* L6470_PerformPreparedApplicationCommand(void);
    void     L6470_DaisyChainCommand(uint8_t* pL6470_DaisyChainSpiTxStruct, uint8_t* pL6470_DaisyChainSpiRxStruct);
    uint32_t L6470_ExtractReturnedData(uint8_t* pL6470_DaisyChainSpiRxStruct, uint8_t LengthByte);
    uint8_t  L6470_CheckStatusRegisterFlag(uint8_t L6470_StatusRegisterFlagId);
    uint8_t* L6470_GetRegisterName(uint8_t id);
    void     L6470_ResetAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg);
    void     L6470_FillAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3);
    void     L6470_PrepareAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3);
    void     L6470_PrepareDaisyChainCommand(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, uint8_t* pL6470_DaisyChainSpiTxStruct);

    /**
     * @brief  Rounding a floating point number to the nearest unsigned integer number.
     * @param  f The floating point number.
     * @retval The nearest unsigned integer number.
     */
    int L6470_round(float f)
    {
        if (f >= 0) {
            return (int) f + (f - (int) f < 0.5f ? 0 : 1);
        } else {
            return (int) f - (f - (int) f < -0.5f ? 1 : 0);
        }
    }


    /*** Component's I/O Methods ***/

    /**
     * @brief      Utility function to read data from L6470.
     * @param[out] pBuffer pointer to the buffer to read data into.
     * @param[in]  NumBytesToRead number of bytes to read.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    status_t Read(uint8_t* pBuffer, uint16_t NumBytesToRead)
    {
        HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_Receive(spi, (uint8_t *)pBuffer, NumBytesToRead, 100);
        HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_SET);
        if( status != HAL_StatusTypeDef::HAL_OK)
        {
        	return COMPONENT_ERROR;
        }
        return COMPONENT_OK;
    }
    
    /**
     * @brief      Utility function to write data to L6470.
     * @param[in]  pBuffer pointer to the buffer of data to send.
     * @param[in]  NumBytesToWrite number of bytes to write.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    status_t Write(uint8_t* pBuffer, uint16_t NumBytesToWrite)
    {
        HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, (uint8_t *)pBuffer, NumBytesToWrite, 100);
        HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_SET);
        if( status != HAL_StatusTypeDef::HAL_OK)
        {
        	return COMPONENT_ERROR;
        }
        return COMPONENT_OK;
    }

    /**
     * @brief      Utility function to read and write data from/to L6470 at the same time.
     * @param[out] pBufferToRead pointer to the buffer to read data into.
     * @param[in]  pBufferToWrite pointer to the buffer of data to send.
     * @param[in]  NumBytes number of bytes to read and write.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    status_t ReadWrite(uint8_t* pBufferToRead, uint8_t* pBufferToWrite, uint16_t NumBytes)
    {
        HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_RESET);
        HAL_StatusTypeDef status_transmit = HAL_SPI_TransmitReceive(spi, pBufferToWrite, pBufferToRead , NumBytes, 1000);
        if( status_transmit != HAL_StatusTypeDef::HAL_OK)
        {
        	HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_SET);
        	return COMPONENT_ERROR;
        }
        HAL_GPIO_WritePin(ssel_port, ssel_pin, GPIO_PinState::GPIO_PIN_SET);
        return COMPONENT_OK;
    }


    void L6470_SPI_Communication(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
    {
        ReadWrite(pRxData, pTxData, Size);
    }



    GPIO_TypeDef* standby_reset_port;
    uint16_t standby_reset_pin;

    GPIO_TypeDef* ssel_port;
    uint16_t ssel_pin;

    SPI_HandleTypeDef *spi;


    uint8_t L6470_Id;
    const sL6470_Register_t *L6470_Register;
    const sL6470_ApplicationCommand_t *L6470_ApplicationCommand;
    const sL6470_Direction_t *L6470_Direction;
    const sL6470_ACT_t *L6470_ACT;
    sL6470_StatusRegister_t L6470_StatusRegister;
    sL6470_StatusRegister_t *pL6470_StatusRegister; 
    StepperMotorRegister_t StepperMotorRegister;
    prepared_action_t prepared_action;

    /* Static data. */
    static uint8_t number_of_devices;
    static const sL6470_Register_t _L6470_Register[L6470REGIDSIZE];
    static const sL6470_ApplicationCommand_t _L6470_ApplicationCommand[L6470APPCMDIDSIZE];
    static const sL6470_Direction_t _L6470_Direction[L6470DIRIDSIZE];
    static const sL6470_ACT_t _L6470_ACT[L6470ACTIDSIZE];
    static eFlagStatus_t L6470_DaisyChain_HalfPrepared;
    static sL6470_AppCmdPkg_t L6470_AppCmdPkg[L6470DAISYCHAINSIZE];
    static uint8_t L6470_DaisyChainSpiTxStruct[L6470MAXSPICMDBYTESIZE][L6470DAISYCHAINSIZE];
    static uint8_t L6470_DaisyChainSpiRxStruct[L6470MAXSPICMDBYTESIZE][L6470DAISYCHAINSIZE];
};

#include "L6470.tpp"

#endif /* __L6470_CLASS_H */
