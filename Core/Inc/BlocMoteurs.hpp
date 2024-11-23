// Version récupérée sur le code des robots de la coupe 2022

#ifndef BLOCMOTEURS_H
#define BLOCMOTEURS_H

#include <cmath>
#include <vector>
#include "XNucleoIHM02A1.h"
#include "config.hpp"


#define NMOTEURS 4
#define XMECANUM_SHAPE




enum motor_location
{
	left,
	right
};

typedef enum _id_moteurs
{
	front_left,
	front_right,
	back_left,
	back_right
}id_moteurs;

class BlocMoteurs
{
	public:
		// Constructeur
		BlocMoteurs(SPI_HandleTypeDef *spi,
				GPIO_TypeDef* standby_reset_port_shield_1  ,uint16_t standby_reset_pin_shield_1 ,GPIO_TypeDef* ssel_port_shield_1 , uint16_t ssel_pin_shield_1,
				GPIO_TypeDef* standby_reset_port_shield_2  ,uint16_t standby_reset_pin_shield_2 ,GPIO_TypeDef* ssel_port_shield_2 , uint16_t ssel_pin_shield_2
		);
		~BlocMoteurs();

		/* Mouvements */

		void commande_vitesses_normalisees(float vitesse_normalisee_FL, float vitesse_normalisee_FR, float vitesse_normalisee_BL, float vitesse_normalisee_BR );
		void commande_vitesses_absolues(float vitesse_absolue_FL, float vitesse_absolue_FR, float vitesse_absolue_BL, float vitesse_absolue_BR );

		void commande_step(unsigned int number_of_step, direction_t dir_FL ,direction_t dir_FR, direction_t dir_BL, direction_t dir_BR);

		void avancer_step( unsigned int number_of_step);
		void reculer_step( unsigned int number_of_step);
		void gauche_step( unsigned int number_of_step);
		void droite_step( unsigned int number_of_step);

		void avancer_vitesse_abs(float vitesses_roues_rad_par_sec);
		void reculer_vitesse_abs(float vitesses_roues_rad_par_sec);
		void gauche_vitesse_abs(float vitesses_roues_rad_par_sec);
		void droite_vitesse_abs(float vitesses_roues_rad_par_sec);

		void avancer_vitesse_normalisee(float vitesse_normalisee);
		void reculer_vitesse_normalisee(float vitesse_normalisee);
		void gauche_vitesse_normalisee(float vitesse_normalisee);
		void droite_vitesse_normalisee(float vitesse_normalisee);


		/* Enable / Disable */
		void motors_on();
		void motors_stop_soft_hiz();
		void motors_stop_hard_hiz();
		void motors_stop_soft();
		void motors_stop_hard();


		/* Configuration */
		bool set_microstepping_mode(step_mode_t step_mode);
		step_mode_t get_microstepping_mode() const {return (step_mode_t) initShield1[0].step_sel;}

		void set_max_speed_moteurs(float radian_par_seconde);
		void set_min_speed_moteurs(float radian_par_seconde);
		void set_max_acc_moteurs(float radian_par_seconde2);
		void set_max_dec_moteurs(float radian_par_seconde2);

		/* Mesure */
		float* mesure_vitesses_rad();
		uint32_t * mesure_vitesses_step();
		uint32_t * mesure_pas_ecoule();


		/* Misc */

		void step_ceil();

		float rad_to_step( float rad);
		float step_to_rad( unsigned int step);
	private:
		void set_vitesse_moteur(unsigned int vitesse, StepperMotor::direction_t dir, id_moteurs id);
		void set_step_moteur(unsigned int steps, StepperMotor::direction_t dir, id_moteurs id);


		bool moteurs_arret;

		// vitesse_max en pps
		unsigned int max_vitesse;

		XNucleoIHM02A1 *shield_1;  // pointeur d'une entité shield_moteur pour la carte de contôle des pas à pas
		XNucleoIHM02A1 *shield_2;  // pointeur d'une entité shield_moteur pour la carte de contôle des pas à pas
		abstractL6470 **moteurs;   // tableau de pointeur de moteur

		uint32_t* last_read_value;

		L6470_init_t initShield1[L6470DAISYCHAINSIZE] = { // init parameters for the motors
		/* First Motor.G */
		{
			22.0,							   /* Motor supply voltage in V. */
			200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
			1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
			3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
			0,							   /* Motor initial speed [step/s]. Seems logic at 0*/
			1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode) */
			1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode)*/
			3000,						   /* Motor maximum speed [step/s] */
			0.0,						   /* Motor minimum speed [step/s]*/
			1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
			5.3,							   /* Holding kval [V]. */
			5.3,							   /* Constant speed kval [V]. */
			5.3,							   /* Acceleration starting kval [V]. */
			5.3,							   /* Deceleration starting kval [V]. */
			269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
			0.00072448,						   /* Start slope [s/step]. */
			0.0016,						   /* Acceleration final slope [s/step]. */
			0.0016,						   /* Deceleration final slope [s/step]. */
			0,							   /* Thermal compensation factor (range [0, 15]). */
			3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). Calculated with Kval*/
			3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). Calculated Kval */
			StepperMotor::STEP_MODE_FULL, /* Step mode selection. */
			0xFF,						   /* Alarm conditions enable. */
			0x2E88						   /* Ic configuration. */
		},

		/* Second Motor. */
		{
			22.0,							   /* Motor supply voltage in V. */
			200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
			1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
			3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
			0,							   /* Motor initial speed [step/s]. */
			1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
			1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
			3000,						   /* Motor maximum speed [step/s]. */
			0.0,						   /* Motor minimum speed [step/s]. */
			1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
			5.3,							   /* Holding kval [V]. */
			5.3,							   /* Constant speed kval [V]. */
			5.3,							   /* Acceleration starting kval [V]. */
			5.3,							   /* Deceleration starting kval [V]. */
			269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
			0.00072448,						   /* Start slope [s/step]. */
			0.0016,						   /* Acceleration final slope [s/step]. */
			0.0016,						   /* Deceleration final slope [s/step]. */
			0,							   /* Thermal compensation factor (range [0, 15]). */
			3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
			3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
			StepperMotor::STEP_MODE_FULL, /* Step mode selection. */
			0xFF,						   /* Alarm conditions enable. */
			0x2E88						   /* Ic configuration. */
		}};

	L6470_init_t initShield2[L6470DAISYCHAINSIZE] = { // init parameters for the motors
		/* First Motor.G */
		{
			22.0,						    /* Motor supply voltage in V. */
			200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
			1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
			3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
			0,							   /* Motor initial speed [step/s]. Seems logic at 0*/
			1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode).*/
			1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode).*/
			3000,						   /* Motor maximum speed [step/s]. */
			0.0,						   /* Motor minimum speed [step/s].*/
			1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
			5.3,							   /* Holding kval [V]. */
			5.3,							   /* Constant speed kval [V]. */
			5.3,							   /* Acceleration starting kval [V]. */
			5.3,							   /* Deceleration starting kval [V]. */
			269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
			0.00072448,						   /* Start slope [s/step]. */
			0.0016,						   /* Acceleration final slope [s/step]. */
			0.0016,						   /* Deceleration final slope [s/step]. */
			0,							   /* Thermal compensation factor (range [0, 15]). */
			3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). Calculated with Kval*/
			3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). Calculated Kval */
			StepperMotor::STEP_MODE_FULL, /* Step mode selection. */
			0xFF,						   /* Alarm conditions enable. */
			0x2E88						   /* Ic configuration. */
		},

		/* Second Motor. */
		{
			22.0,							   /* Motor supply voltage in V. */
			200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
			1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
			3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
			0,							   /* Motor initial speed [step/s]. */
			1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
			1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
			3000,						   /* Motor maximum speed [step/s]. */
			0.0,						   /* Motor minimum speed [step/s]. */
			1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
			5.3,							   /* Holding kval [V]. */
			5.3,							   /* Constant speed kval [V]. */
			5.3,							   /* Acceleration starting kval [V]. */
			5.3,							   /* Deceleration starting kval [V]. */
			269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
			0.00072448,						   /* Start slope [s/step]. */
			0.0016,						   /* Acceleration final slope [s/step]. */
			0.0016,						   /* Deceleration final slope [s/step]. */
			0,							   /* Thermal compensation factor (range [0, 15]). */
			3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
			3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
			StepperMotor::STEP_MODE_FULL, /* Step mode selection. */
			0xFF,						   /* Alarm conditions enable. */
			0x2E88						   /* Ic configuration. */
		}};

};





#endif
