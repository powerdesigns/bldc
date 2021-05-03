/*
	Copyright 2020 Martin Paz mpaz@paltatech.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "stm32f4xx_conf.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "hw.h"
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "commands.h"
#include "datatypes.h"
#include "mempools.h"
#include "terminal.h"
#include "timeout.h"
#include "utils.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "built_in_test.h"

// Private variables
volatile float V_Lx_temp [3] = {0};
volatile float I_Lx_temp [3] = {0};

volatile float V_Lx [3] = {0};
volatile float I_Lx [3] = {0};

volatile float curr0_offset = 0.0;
volatile float curr1_offset = 0.0;
volatile float curr2_offset = 0.0;

volatile uint32_t sample = 0;

volatile float V_temp [100] = {0};
volatile float I_temp [100] = {0};

volatile float V_t = 0;
volatile float I_t = 0;

volatile BIT_state_t BIT_state;

// Private functions
static void terminal_cmd_BIT_multiple_pulse_test(int argc, const char **argv);
static void terminal_cmd_BIT_check_all_legs(int argc, const char **argv);

static void multi_pulse_test (uint32_t tot_pulse, float pulse_dwell_us, float pulse_off_us, char leg[32], char side[32], char supply_leg[32], FlagStatus visibility);

static void BIT_mcpwm_foc_timer_reinit(int f_sw);
static void BIT_mcpwm_timer_reinit(int f_sw);

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

#define TIMER_UPDATE_SAMP(samp) \
		TIM2->CCR2 = (samp / 2);


void built_in_test_init(void){

	BIT_state = BIT_IDLE;

	terminal_register_command_callback(
			"BIT_multiple_pulse_test",
			"Trigger a multiple pulse test - For example: BIT_multiple_pulse_test 5 10.2 2.3 A TOP --supply-with C FALSE",
			"[total_pulse] [pulse_dwell_us] [pulse_off_us] [leg] [side] [argument(optional)] [supply_leg(optional)] [visibility]",
			terminal_cmd_BIT_multiple_pulse_test);

	terminal_register_command_callback(
			"BIT_check_all_legs",
			"Check all legs with a multiple pulse test for each leg combination - For example: BIT_check_all_legs 10 10.2 2.3 FALSE",
			"[total_pulse] [pulse_dwell_us] [pulse_off_us] [visibility]",
			terminal_cmd_BIT_check_all_legs);
}


static void terminal_cmd_BIT_multiple_pulse_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	int total_pulse = 0;
	float pulse_dwell_us = 0;
	float pulse_off_us = 0;
	char leg[10];
	char supply_leg[10];
	char side[10];
	char argument[32];
	char visibility[10];
	FlagStatus visib_state = RESET;


	mc_interface_select_motor_thread(1);
	mc_state m_state = mc_interface_get_state();

	if (m_state == MC_STATE_RUNNING){
		commands_printf("Motor Control is Running \n");
		return;
	}

	if( (argc == 7) || (argc == 9) ) {
		sscanf(argv[1], "%d", &total_pulse);
		sscanf(argv[2], "%f", &pulse_dwell_us);
		sscanf(argv[3], "%f", &pulse_off_us);
		sscanf(argv[4], "%s", leg);
		sscanf(argv[5], "%s", side);

		if ( !((leg[0] == 'A') || (leg[0] == 'B') || (leg[0] == 'C'))){
			commands_printf("Invalid leg. Use A, B or C \n");
			return;
		}
		if ( !((side[0] == 'T') || (side[0] == 'B'))){
			commands_printf("Invalid side. Use T, TOP, B or BOTTOM \n");
			return;
		}

		if(argc == 9) {
			sscanf(argv[6], "%s", argument);
			sscanf(argv[7], "%s", supply_leg);
			sscanf(argv[8], "%s", visibility);

			if ( !((supply_leg[0] == 'A') || (supply_leg[0] == 'B') || (supply_leg[0] == 'C'))){
				commands_printf("Invalid supply leg. Use A, B or C \n");
				return;
			}
			if(leg[0] == supply_leg[0]) {
				commands_printf("Supply with a different leg. Aborting.");
				return;
			}
			if( strcmp(argument, "--supply-with") != 0 ){
				commands_printf("Invalid argument. Use ""--supply-with"" \n");
				return;
			}
			if ( !((visibility[0] == 'T') || (visibility[0] == 'F'))){
				commands_printf("Invalid side. Use T, TRUE, F or FALSE \n");
				return;
			}
		}
		else{
			sscanf(argv[6], "%s", visibility);

			if ( !((visibility[0] == 'T') || (visibility[0] == 'F'))){
				commands_printf("Invalid side. Use T, TRUE, F or FALSE \n");
				return;
			}

			argument[0] = 0;
			supply_leg[0] = 0;
		}

		if( !strcmp(visibility, "TRUE") || !strcmp(visibility, "T") ){
			visib_state = SET;
		}

		BIT_state = BIT_RUN;

		/**************************************************************************************/
		utils_sys_lock_cnt();

		mc_configuration *mcconf = mempools_alloc_mcconf();
		*mcconf = *mc_interface_get_configuration();

		int motor_old = mc_interface_get_motor_thread();

		mc_interface_select_motor_thread(1);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		mc_interface_select_motor_thread(2);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		// Disable timeout
		systime_t tout = timeout_get_timeout_msec();
		float tout_c = timeout_get_brake_current();
		timeout_reset();
		timeout_configure(65000, 0.0);
		timeout_feed_WDT(THREAD_MCPWM);
		timeout_configure_IWDT_slowest();

		/**************************************************************************************/
		float foc_f_sw_old = mcconf->foc_f_sw;
		float period_old = 1.0/foc_f_sw_old;
		float period_new = pulse_dwell_us/1000000.0 + pulse_off_us/1000000.0;

		if( period_new > period_old) {
			commands_printf("Period too long, must be less than %.2f usec. Aborting.", (double)(period_old * 1000000.0));
		}
		else{

			float m_I_max_set = mcconf->l_current_max;
			float m_V_max_set = mcconf->l_max_vin;
			float m_V_in = GET_INPUT_VOLTAGE();

			commands_printf("I Max set: %.2f A \tV Max set: %.2f V \tV in: %.2f V \n", (double)m_I_max_set,(double)m_V_max_set,(double)m_V_in);

			if(argc == 7) {
				commands_printf("%s %d %.2f %.2f %s %s %s", argv[0], total_pulse, (double)pulse_dwell_us, (double)pulse_off_us, leg, side, visibility);
			}
			else {
				commands_printf("%s %d %.2f %.2f %s %s supply with %s %s", argv[0], total_pulse, (double)pulse_dwell_us, (double)pulse_off_us, leg, side, supply_leg, visibility);
			}

			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, leg, side, supply_leg, visib_state);

			switch (mcconf->motor_type) {
			case MOTOR_TYPE_BLDC:
			case MOTOR_TYPE_DC:
				BIT_mcpwm_timer_reinit(mcconf->m_bldc_f_sw_max);
				break;

			case MOTOR_TYPE_FOC:
				BIT_mcpwm_foc_timer_reinit(mcconf->foc_f_sw);
				break;

			case MOTOR_TYPE_GPD:
				break;

			default:
				break;
			}
		}

		/**************************************************************************************/
		timeout_configure_IWDT();
		timeout_feed_WDT(THREAD_MCPWM);
		timeout_configure(tout, tout_c);

		mc_interface_set_configuration(mcconf);
		mempools_free_mcconf(mcconf);

		mc_interface_select_motor_thread(1);
		mc_interface_release_motor();
		mc_interface_unlock();
		mc_interface_select_motor_thread(2);
		mc_interface_release_motor();
		mc_interface_unlock();

		mc_interface_select_motor_thread(motor_old);
		mc_interface_release_motor();

		utils_sys_unlock_cnt();

		/**************************************************************************************/
		BIT_state = BIT_IDLE;

		commands_printf("Fired!");
	}
	else {
		commands_printf("7 or 8 arguments required. For example: BIT_multiple_pulse_test 5 10.2 2.3 A TOP --supply-with C FALSE");
	}
	commands_printf(" ");
}

static void terminal_cmd_BIT_check_all_legs(int argc, const char **argv){
	(void)argc;
	(void)argv;

	int total_pulse = 0;
	float pulse_dwell_us = 0;
	float pulse_off_us = 0;
	char visibility[10];
	FlagStatus visib_state = RESET;

	mc_interface_select_motor_thread(1);
	mc_state m_state = mc_interface_get_state();

	if (m_state == MC_STATE_RUNNING){
		commands_printf("Motor Control is Running \n");
		return;
	}

	if (argc == 5) {
		sscanf(argv[1], "%d", &total_pulse);
		sscanf(argv[2], "%f", &pulse_dwell_us);
		sscanf(argv[3], "%f", &pulse_off_us);
		sscanf(argv[4], "%s", visibility);

		if ( !((visibility[0] == 'T') || (visibility[0] == 'F'))){
			commands_printf("Invalid side. Use T, TRUE, F or FALSE \n");
			return;
		}

		if( !strcmp(visibility, "TRUE") || !strcmp(visibility, "T") ){
			visib_state = SET;
		}

		BIT_state = BIT_RUN;

		/**************************************************************************************/
		utils_sys_lock_cnt();

		mc_configuration *mcconf = mempools_alloc_mcconf();
		*mcconf = *mc_interface_get_configuration();

		int motor_old = mc_interface_get_motor_thread();

		mc_interface_select_motor_thread(1);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		mc_interface_select_motor_thread(2);
		mc_interface_unlock();
		mc_interface_release_motor();
		mc_interface_lock();

		// Disable timeout
		systime_t tout = timeout_get_timeout_msec();
		float tout_c = timeout_get_brake_current();
		timeout_reset();
		timeout_configure(65000, 0.0);
		timeout_feed_WDT(THREAD_MCPWM);
		timeout_configure_IWDT_slowest();

		/**************************************************************************************/
		float foc_f_sw_old = mcconf->foc_f_sw;
		float period_old = 1.0/foc_f_sw_old;
		float period_new = pulse_dwell_us/1000000.0 + pulse_off_us/1000000.0;

		if( period_new > period_old) {
			commands_printf("Period too long, must be less than %.2f usec. Aborting.", (double)(period_old * 1000000.0));
		}
		else{

			float m_I_max_set = mcconf->l_current_max;
			float m_V_max_set = mcconf->l_max_vin;
			float m_V_in = GET_INPUT_VOLTAGE();

			commands_printf("I Max set: %.2f A \tV Max set: %.2f V \tV in: %.2f V \n", (double)m_I_max_set,(double)m_V_max_set,(double)m_V_in);

			commands_printf("%s %s %s %s %s %s supply with %s  %s", argv[0], argv[1], argv[2], argv[3], "A", "TOP", "B", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","T","B",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "A", "TOP", "C", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","T","C",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "B", "TOP", "A", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","T","A",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "B", "TOP", "C", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","T","C",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "C", "TOP", "A", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","T","A",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "C", "TOP", "B", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","T","B",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "A", "BOTTOM", "B", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","B","B",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "A", "BOTTOM", "C", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","B","C",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "B", "BOTTOM", "A", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","B","A",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "B", "BOTTOM", "C", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","B","C",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "C", "BOTTOM", "A", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","B","A",visib_state);

			commands_printf("%s %s %s %s %s %s supply with %s %s", argv[0], argv[1], argv[2], argv[3], "C", "BOTTOM", "B", argv[4]);
			multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","B","B",visib_state);

			switch (mcconf->motor_type) {
			case MOTOR_TYPE_BLDC:
			case MOTOR_TYPE_DC:
				BIT_mcpwm_timer_reinit(mcconf->m_bldc_f_sw_max);
				break;

			case MOTOR_TYPE_FOC:
				BIT_mcpwm_foc_timer_reinit(mcconf->foc_f_sw);
				break;

			case MOTOR_TYPE_GPD:
				break;

			default:
				break;
			}
		}

		/**************************************************************************************/
		timeout_configure_IWDT();
		timeout_feed_WDT(THREAD_MCPWM);
		timeout_configure(tout, tout_c);

		mc_interface_set_configuration(mcconf);
		mempools_free_mcconf(mcconf);

		mc_interface_select_motor_thread(1);
		mc_interface_release_motor();
		mc_interface_unlock();
		mc_interface_select_motor_thread(2);
		mc_interface_release_motor();
		mc_interface_unlock();

		mc_interface_select_motor_thread(motor_old);
		mc_interface_release_motor();

		utils_sys_unlock_cnt();
		/**************************************************************************************/

		BIT_state = BIT_IDLE;
	}
	else {
		commands_printf("4 arguments required. For example: BIT_check_all_legs 15 10.2 2.3 FALSE");
	}

	commands_printf(" ");
}


int BIT_pulse_test(int total_pulse, float pulse_dwell_us, float pulse_off_us){
	BIT_state = BIT_RUN;

	/**************************************************************************************/
	utils_sys_lock_cnt();

	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	int motor_old = mc_interface_get_motor_thread();

	mc_interface_select_motor_thread(1);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_lock();

	mc_interface_select_motor_thread(2);
	mc_interface_unlock();
	mc_interface_release_motor();
	mc_interface_lock();

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	timeout_reset();
	timeout_configure(65000, 0.0);
	timeout_feed_WDT(THREAD_MCPWM);
	timeout_configure_IWDT_slowest();

	/**************************************************************************************/
	float foc_f_sw_old = mcconf->foc_f_sw;
	float period_old = 1.0/foc_f_sw_old;
	float period_new = pulse_dwell_us/1000000.0 + pulse_off_us/1000000.0;

	if( period_new > period_old) {
		BIT_state = BIT_IDLE;
		return -1;
	}
	else{

		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","T","B",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","T","C",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","T","A",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","T","C",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","T","A",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","T","B",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","B","B",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "A","B","C",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","B","A",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "B","B","C",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","B","A",FALSE);
		multi_pulse_test(total_pulse, pulse_dwell_us, pulse_off_us, "C","B","B",FALSE);

		switch (mcconf->motor_type) {
		case MOTOR_TYPE_BLDC:
		case MOTOR_TYPE_DC:
			BIT_mcpwm_timer_reinit(mcconf->m_bldc_f_sw_max);
			break;

		case MOTOR_TYPE_FOC:
			BIT_mcpwm_foc_timer_reinit(mcconf->foc_f_sw);
			break;

		case MOTOR_TYPE_GPD:
			break;

		default:
			break;
		}
	}

	/**************************************************************************************/
	timeout_configure_IWDT();
	timeout_feed_WDT(THREAD_MCPWM);
	timeout_configure(tout, tout_c);

	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);

	mc_interface_select_motor_thread(1);
	mc_interface_release_motor();
	mc_interface_unlock();
	mc_interface_select_motor_thread(2);
	mc_interface_release_motor();
	mc_interface_unlock();

	mc_interface_select_motor_thread(motor_old);
	mc_interface_release_motor();

	utils_sys_unlock_cnt();
	/**************************************************************************************/
	BIT_state = BIT_IDLE;
	return 0;
}

FlagStatus BIT_get_state(void)
{
	if ( BIT_state != BIT_IDLE )
	return SET;

	return RESET;
}

static void multi_pulse_test (uint32_t tot_pulse, float pulse_dwell_us, float pulse_off_us, char leg[10], char side[10], char supply_leg[10], FlagStatus visibility){

	/**************************************************************************************/
	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	timeout_feed_WDT(THREAD_MCPWM);

	/**************************************************************************************/
	if(leg[0] == supply_leg[0]) {
		commands_printf("Supply with a different leg. Aborting.");
		return;
	}

	pulse_dwell_us /= 1000000.0;
	pulse_off_us /= 1000000.0;

	float period_new = pulse_dwell_us + pulse_off_us;
	uint32_t f_sw_new= 1.0 / period_new;
	float top_new = SYSTEM_CORE_CLOCK / (int)f_sw_new;
	float duty_new =  pulse_dwell_us / period_new;
	uint32_t compare_new = (uint32_t)(top_new * duty_new);

	//pulse test Bottom mosfets
	if(side[0] == 'B') {
		// clear pin top mosfets
		palClearPad(GPIOA, 8);
		palClearPad(GPIOA, 9);
		palClearPad(GPIOA, 10);
		// TOP  Re-Configuration: Channel 1 to 3 as gpio function push-pull
		palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	}

	//pulse test Top mosfets
	else if(side[0] == 'T') {
		// clear bottom mosfets
		palClearPad(GPIOB, 13);
		palClearPad(GPIOB, 14);
		palClearPad(GPIOB, 15);
		// BOTTOM Re-Configuration: Channel 1 to 3 as gpio function push-pull
		palSetPadMode(GPIOB, 13, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
		palSetPadMode(GPIOB, 15, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

		// when powerstage is bootstrapped, we need to pre-charge the top gate driver by driving the leg output
		// to zero volts.
		switch (leg[0]) {
		case 'A':
			palSetPad(GPIOB, 13);
			chThdSleepMicroseconds(500);
			palClearPad(GPIOB, 13);
			break;
		case 'B':
			palSetPad(GPIOB, 14);
			chThdSleepMicroseconds(500);
			palClearPad(GPIOB, 14);
			break;
		case 'C':
			palSetPad(GPIOB, 15);
			chThdSleepMicroseconds(500);
			palClearPad(GPIOB, 15);
			break;
		}
	}
	else
		return;

	switch (supply_leg[0]) {
	case 'A':
		if(side[0] == 'B')
			palSetPad(GPIOA, 8);
		else if(side[0] == 'T')
			palSetPad(GPIOB, 13);
		break;

	case 'B':
		if(side[0] == 'B')
			palSetPad(GPIOA, 9);
		else if(side[0] == 'T')
			palSetPad(GPIOB, 14);
		break;

	case 'C':
		if(side[0] == 'B')
			palSetPad(GPIOA, 10);
		else if(side[0] == 'T')
			palSetPad(GPIOB, 15);
		break;
	}

	chThdSleepMicroseconds(500);	// wait for mosfet to fully turn ON

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	switch (mcconf->motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		TIM_DeInit(TIM8);
		TIM8->CNT = 0;

		// Time Base configuration
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)f_sw_new;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 2;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM8, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM8, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM8, ENABLE);
		TIM_CCPreloadControl(TIM8, ENABLE);

		// PWM outputs have to be enabled in order to trigger ADC on CCx
		TIM_CtrlPWMOutputs(TIM8, ENABLE);

		// TIM1 Master and TIM8 slave
		TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
		TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
		TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
		TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

		TIM_Cmd(TIM8, ENABLE);

		break;

	case MOTOR_TYPE_FOC:
		TIM_DeInit(TIM2);
		TIM2->CNT = 0;

		// Time Base configuration
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)f_sw_new;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 250;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
		TIM_OC1Init(TIM2, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM2, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM2, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM2, ENABLE);
		TIM_CCPreloadControl(TIM2, ENABLE);

		// PWM outputs have to be enabled in order to trigger ADC on CCx
		TIM_CtrlPWMOutputs(TIM2, ENABLE);

		// TIM1 Master and TIM2 slave
		TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
		TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
		TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
		TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

		TIM_Cmd(TIM2, ENABLE);

		// Sample intervals
		TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

		// Enable CC2 interrupt, which will be fired in V0 and V7
		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

		nvicEnableVector(TIM2_IRQn, 6);

		break;

	case MOTOR_TYPE_GPD:
		break;

	default:
		break;
	}

	TIM_DeInit(TIM1);
	TIM1->CNT = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)f_sw_new;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = compare_new;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	if(side[0] == 'B') {
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	}
	else if(side[0] == 'T') {
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	}

	switch (leg[0]) {
	case 'A':
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		break;
	case 'B':
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		break;
	case 'C':
		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
		break;
	}

	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);

	volatile float m_V_in = GET_INPUT_VOLTAGE();

	V_Lx_temp[0] = 0;
	V_Lx_temp[1] = 0;
	V_Lx_temp[2] = 0;

	I_Lx_temp[0] = 0;
	I_Lx_temp[1] = 0;
	I_Lx_temp[2] = 0;

	V_Lx[0] = 0;
	V_Lx[1] = 0;
	V_Lx[2] = 0;

	I_Lx[0] = 0;
	I_Lx[1] = 0;
	I_Lx[2] = 0;

	if(mcconf->motor_type == MOTOR_TYPE_FOC) {
		mcpwm_foc_get_current_offsets(&curr0_offset, &curr1_offset, &curr2_offset, false);

		I_Lx_temp[0] = curr0_offset;
		I_Lx_temp[1] = curr1_offset;
#ifdef HW_HAS_3_SHUNTS
		I_Lx_temp[2] = curr2_offset;
#endif
	}

	switch (leg[0]) {
	case 'A':

		TIMER_UPDATE_DUTY(compare_new, 0, 0);
		for(sample=0;sample<tot_pulse;sample++){
			while( !(TIM1->SR & (TIM_FLAG_CC1)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC1);

			V_Lx_temp[0] += (float)ADC_Value[ADC_IND_SENS1];
			I_Lx_temp[0] += (float)ADC_Value[ADC_IND_CURR1];
			V_Lx_temp[1] += (float)ADC_Value[ADC_IND_SENS2];
			I_Lx_temp[1] += (float)ADC_Value[ADC_IND_CURR2];
			V_Lx_temp[2] += (float)ADC_Value[ADC_IND_SENS3];
#ifdef HW_HAS_3_SHUNTS
			I_Lx_temp[2] += (float)ADC_Value[ADC_IND_CURR3];
#endif

			V_temp[sample] = (float)ADC_Value[ADC_IND_SENS1];
			I_temp[sample] = (float)ADC_Value[ADC_IND_CURR1];
		}

		break;

	case 'B':

		TIMER_UPDATE_DUTY(0, compare_new, 0);
		for(sample=0;sample<tot_pulse;sample++){
			while( !(TIM1->SR & (TIM_FLAG_CC2)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC2);

			V_Lx_temp[0] += (float)ADC_Value[ADC_IND_SENS1];
			I_Lx_temp[0] += (float)ADC_Value[ADC_IND_CURR1];
			V_Lx_temp[1] += (float)ADC_Value[ADC_IND_SENS2];
			I_Lx_temp[1] += (float)ADC_Value[ADC_IND_CURR2];
			V_Lx_temp[2] += (float)ADC_Value[ADC_IND_SENS3];
#ifdef HW_HAS_3_SHUNTS
			I_Lx_temp[2] += (float)ADC_Value[ADC_IND_CURR3];
#endif

			V_temp[sample] = (float)ADC_Value[ADC_IND_SENS2];
			I_temp[sample] = (float)ADC_Value[ADC_IND_CURR2];
		}

		break;

	case 'C':

		TIMER_UPDATE_DUTY(0, 0, compare_new);
		for(sample=0;sample<tot_pulse;sample++){
			while( !(TIM1->SR & (TIM_FLAG_CC3)) );
			TIM_ClearFlag(TIM1, TIM_FLAG_CC3);

			V_Lx_temp[0] += (float)ADC_Value[ADC_IND_SENS1];
			I_Lx_temp[0] += (float)ADC_Value[ADC_IND_CURR1];
			V_Lx_temp[1] += (float)ADC_Value[ADC_IND_SENS2];
			I_Lx_temp[1] += (float)ADC_Value[ADC_IND_CURR2];
			V_Lx_temp[2] += (float)ADC_Value[ADC_IND_SENS3];
#ifdef HW_HAS_3_SHUNTS
			I_Lx_temp[2] += (float)ADC_Value[ADC_IND_CURR3];
#endif

			V_temp[sample] = (float)ADC_Value[ADC_IND_SENS3];
			I_temp[sample] = (float)ADC_Value[ADC_IND_CURR3];
		}

		break;
	}

	TIMER_UPDATE_DUTY(0, 0, 0);
	TIM_Cmd(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	TIM_ClearFlag(TIM1, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);
	TIM_DeInit(TIM1);

	// turn off all mosfets
	palClearPad(GPIOA, 8);
	palClearPad(GPIOA, 9);
	palClearPad(GPIOA, 10);

	palClearPad(GPIOB, 13);
	palClearPad(GPIOB, 14);
	palClearPad(GPIOB, 15);

	// GPIO Configuration: Channel 1 to 6 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLDOWN);

	V_Lx_temp[0] /= (float)sample;
	I_Lx_temp[0] -= curr0_offset;
	I_Lx_temp[0] /= (float)sample;

	V_Lx_temp[1] /= (float)sample;
	I_Lx_temp[1] -= curr1_offset;
	I_Lx_temp[1] /= (float)sample;

	V_Lx_temp[2] /= (float)sample;
#ifdef HW_HAS_3_SHUNTS
	I_Lx_temp[2] -= curr2_offset;
	I_Lx_temp[2] /= (float)sample;
#endif

	V_Lx[0] = ((float)ADC_Value[ADC_IND_SENS1] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	I_Lx[0] = (((float)ADC_Value[ADC_IND_CURR1] - (float)curr0_offset) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
	V_Lx[1] = ((float)ADC_Value[ADC_IND_SENS2] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	I_Lx[1] = (((float)ADC_Value[ADC_IND_CURR2] - (float)curr1_offset) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
	V_Lx[2] = ((float)ADC_Value[ADC_IND_SENS3] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#ifdef HW_HAS_3_SHUNTS
	I_Lx[2] = (((float)ADC_Value[ADC_IND_CURR3] - (float)curr2_offset) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
#endif

	commands_printf("V_L1_last: %.2f V\t V_L2_last: %.2f V\t V_L3_last: %.2f V", (double)V_Lx[0], (double)V_Lx[1], (double)V_Lx[2]);
	commands_printf("I_L1_last: %.2f A\t I_L2_last: %.2f A\t I_L3_last: %.2f A", (double)I_Lx[0], (double)I_Lx[1], (double)I_Lx[2]);

	switch (leg[0]) {
	case 'A':
		if(side[0] == 'T')	{
			if ((m_V_in - V_Lx[0]) > 1.0)	commands_printf("Fault in V sensor of leg A");
		}
		if (fabsf(I_Lx[0]) < 1.0)			commands_printf("Fault in I sensor of leg A");
		break;
	case 'B':
		if(side[0] == 'T')	{
			if ((m_V_in - V_Lx[1]) > 1.0)	commands_printf("Fault in V sensor of leg B");
		}
		if (fabsf(I_Lx[1]) < 1.0)			commands_printf("Fault in I sensor of leg B");
		break;
	case 'C':
		if(side[0] == 'T')	{
			if ((m_V_in - V_Lx[2]) > 1.0)	commands_printf("Fault in V sensor of leg C");
		}
		if (fabsf(I_Lx[2]) < 1.0)			commands_printf("Fault in I sensor of leg C");
		break;
	}

	if (visibility){
		V_Lx[0] = (V_Lx_temp [0] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		I_Lx[0] = (((I_Lx_temp [0] - (float)curr0_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
		V_Lx[1] = (V_Lx_temp [1] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		I_Lx[1] = (((I_Lx_temp [1] - (float)curr1_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
		V_Lx[2] = (V_Lx_temp [2] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	#ifdef HW_HAS_3_SHUNTS
		I_Lx[2] = (((I_Lx_temp [2] - (float)curr2_offset)) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
	#endif

		commands_printf(" ");
		commands_printf("V_L1_prom: %.2f V\t V_L2_prom: %.2f V\t V_L3_prom: %.2f V", (double)V_Lx[0], (double)V_Lx[1], (double)V_Lx[2]);
		commands_printf("I_L1_prom: %.2f A\t I_L2_prom: %.2f A\t I_L3_prom: %.2f A", (double)I_Lx[0], (double)I_Lx[1], (double)I_Lx[2]);

		for(sample=0;sample<tot_pulse;sample++){
			V_t = (V_temp [sample] * V_REG / 4096.0) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
			I_t = (((I_temp [sample] - ((float)curr0_offset))) * V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
			commands_printf("[%03d] V_temp: %.2f V\t I_temp: %.2f A", sample, (double)V_t, (double)I_t);
		}
	}

	/**************************************************************************************/
	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);

	mempools_free_mcconf(mcconf);

	/**************************************************************************************/
	commands_printf(" ");
}


static void BIT_mcpwm_foc_timer_reinit(int f_sw) {
	utils_sys_lock_cnt();

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM2);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM1->CNT = 0;
	TIM2->CNT = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_sw);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime =  conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

#ifdef HW_USE_BRK
	// Enable BRK function. Hardware will asynchronously stop any PWM activity upon an
	// external fault signal. PWM outputs remain disabled until MCU is reset.
	// software will catch the BRK flag to report the fault code
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	// ------------- Timer2 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CCPreloadControl(TIM2, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM2, ENABLE);

	// TIM1 Master and TIM2 slave
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	// Prevent all low side FETs from switching on
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	// Sample intervals
	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

	// Enable CC2 interrupt, which will be fired in V0 and V7
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	utils_sys_unlock_cnt();

	nvicEnableVector(TIM2_IRQn, 6);
}


static void BIT_mcpwm_timer_reinit(int f_sw) {
	utils_sys_lock_cnt();

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM1->CNT = 0;
	TIM8->CNT = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_sw);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime =  conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CCPreloadControl(TIM8, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// TIM1 Master and TIM8 slave
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

	// Enable TIM1 and TIM8
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);

	// Prevent all low side FETs from switching on
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	utils_sys_unlock_cnt();

}
