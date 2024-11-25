#include "BlocMoteurs.hpp"


/**
  * @brief  Constructeur de classe, l'ensemble des GPIO et peripherique SPI est suposse init dans le main grâce a la generation de code auto
  * @param spi , Handler du SPI
  * @param standby_reset_port_shield_1 , port du GPIO du signal de reset du shield 1
  * @param standby_reset_pin_shield_1  , pin  du GPIO du signal de reset du shield 1
  * @param ssel_port_shield_1          , port du GPIO de commmunication SPI de selection pour le shield 1
  * @param ssel_port_shield_1          , pin  du GPIO de commmunication SPI de selection pour le shield 1
  * @param standby_reset_port_shield_2 , port du GPIO du signal de reset du shield 2
  * @param standby_reset_pin_shield_2  , pin  du GPIO du signal de reset du shield 2
  * @param ssel_port_shield_2          , port du GPIO de commmunication SPI pour le shield 2
  * @param ssel_port_shield_2          , pin  du GPIO de commmunication SPI pour le shield 2
  */
BlocMoteurs::BlocMoteurs(SPI_HandleTypeDef *spi,
		GPIO_TypeDef* standby_reset_port_shield_1  ,uint16_t standby_reset_pin_shield_1 ,GPIO_TypeDef* ssel_port_shield_1 , uint16_t ssel_pin_shield_1,
		GPIO_TypeDef* standby_reset_port_shield_2  ,uint16_t standby_reset_pin_shield_2 ,GPIO_TypeDef* ssel_port_shield_2 , uint16_t ssel_pin_shield_2):
    index_to_enum{back_right, back_left, front_left, front_right}, //Wiring dependant
    motor_direction_inverter{1.0, -1.0, 1.0, -1.0}
{
	//moteurs à l'arret par défaut
	moteurs_arret = 0;
	//vitesse par défaut
	max_vitesse = MAX_VITESSE;

  shield_1 = new XNucleoIHM02A1(&initShield1[0], &initShield2[1], spi, standby_reset_port_shield_1, standby_reset_pin_shield_1, ssel_port_shield_1, ssel_pin_shield_1);
  shield_2 = new XNucleoIHM02A1(&initShield1[0], &initShield2[1], spi, standby_reset_port_shield_2, standby_reset_pin_shield_2, ssel_port_shield_2, ssel_pin_shield_2);
  abstractL6470** moteurs_shield_1 = shield_1->get_motor_drivers();
  abstractL6470** moteurs_shield_2 = shield_2->get_motor_drivers();
  moteurs = new abstractL6470*[NMOTEURS];



  moteurs[front_left] = moteurs_shield_2[0]; //Wiring dependant
  moteurs[front_right] = moteurs_shield_2[1]; //Wiring dependant
  moteurs[back_left] = moteurs_shield_1[1];  //Wiring dependant
  moteurs[back_right] = moteurs_shield_1[0]; //Wiring dependant





  //Identify which motor is on which shield, which motor is which

  // moteurs_shield_1[0]->prepare_run(StepperMotor::FWD, 50);
  // moteurs_shield_1[1]->prepare_run(StepperMotor::FWD, 0);
  // shield_1->perform_prepared_actions();
  // HAL_Delay(4000);
  // moteurs_shield_1[0]->prepare_run(StepperMotor::FWD, 0);
  // moteurs_shield_1[1]->prepare_run(StepperMotor::FWD, 50);
  // shield_1->perform_prepared_actions();
  // HAL_Delay(4000);
  // motors_stop_hard();
  // moteurs_shield_2[0]->prepare_run(StepperMotor::FWD, 50);
  // moteurs_shield_2[1]->prepare_run(StepperMotor::FWD, 0);
  // shield_2->perform_prepared_actions();
  // HAL_Delay(4000);
  // moteurs_shield_2[0]->prepare_run(StepperMotor::FWD, 0);
  // moteurs_shield_2[1]->prepare_run(StepperMotor::FWD, 50);
  // shield_2->perform_prepared_actions();
  // HAL_Delay(4000);
  // motors_stop_hard();


}

constexpr float RAD_PER_FULL_STEP = (DEG_PER_FULL_STEP*M_PI)/180.0;

float BlocMoteurs::rad_to_step( float rad)
{
	return  (rad/RAD_PER_FULL_STEP);
}
float BlocMoteurs::step_to_rad( unsigned int step)
{
	return (RAD_PER_FULL_STEP*step);
}


/**
  * @brief  Destructeur de classe
  */
BlocMoteurs::~BlocMoteurs()
{
    delete [] moteurs;
}


/**
  * @brief  Applique une commande de vitesse normalisee par rapport au maximum de vitesse
  * @param  vitesse_normalisee_FL vitesse du moteur Avant gauche
  * @param  vitesse_normalisee_FR vitesse du moteur Avant droit
  * @param  vitesse_normalisee_BL vitesse du moteur Arriere gauche
  * @param  vitesse_normalisee_BR vitesse du moteur Arriere droit
  */
void BlocMoteurs::commande_vitesses_normalisees(float vitesse_normalisee_FL, float vitesse_normalisee_FR, float vitesse_normalisee_BL, float vitesse_normalisee_BR )
{
    /////////////////////////////////////////////////////////////
    // Assurance que les parametres sont entre -1 et 1 ////////////////
    vitesse_normalisee_FL = std::min(1.0f, vitesse_normalisee_FL);
    vitesse_normalisee_FL = std::max(-1.0f, vitesse_normalisee_FL);

    vitesse_normalisee_FR = std::min(1.0f, vitesse_normalisee_FR);
    vitesse_normalisee_FR = std::max(-1.0f, vitesse_normalisee_FR);

    vitesse_normalisee_BL = std::min(1.0f, vitesse_normalisee_BL);
    vitesse_normalisee_BL = std::max(-1.0f, vitesse_normalisee_BL);

    vitesse_normalisee_BR = std::min(1.0f, vitesse_normalisee_BR);
    vitesse_normalisee_BR = std::max(-1.0f, vitesse_normalisee_BR);
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    // Détermination du signe des vitesses (logique inversée entre les deux moteurs droits/gauches car placés symétriquement sur le robot et branchement identique)
    StepperMotor::direction_t sens_FL = vitesse_normalisee_FL*motor_direction_inverter[front_left] >= 0.0f ? StepperMotor::FWD : StepperMotor::BWD;
    
    StepperMotor::direction_t sens_FR = vitesse_normalisee_FR*motor_direction_inverter[front_right] >= 0.0f ? StepperMotor::FWD : StepperMotor::BWD;

    StepperMotor::direction_t sens_BL = vitesse_normalisee_BL*motor_direction_inverter[back_left] >= 0.0f ? StepperMotor::FWD : StepperMotor::BWD;

    StepperMotor::direction_t sens_BR = vitesse_normalisee_BR*motor_direction_inverter[back_right] >= 0.0f ? StepperMotor::FWD : StepperMotor::BWD;

    // Détermination des valeurs de vitesses réelles à envoyer au shield
    unsigned int vitesse_FL = ( (float) fabs(vitesse_normalisee_FL) * max_vitesse);
    unsigned int vitesse_FR = ( (float) fabs(vitesse_normalisee_FR) * max_vitesse);
    unsigned int vitesse_BL = ( (float) fabs(vitesse_normalisee_BL) * max_vitesse);
    unsigned int vitesse_BR = ( (float) fabs(vitesse_normalisee_BR) * max_vitesse);

    set_vitesse_moteur(vitesse_FL, sens_FL, id_moteurs::front_left);
    set_vitesse_moteur(vitesse_FR, sens_FR, id_moteurs::front_right);
    set_vitesse_moteur(vitesse_BL, sens_BL, id_moteurs::back_left);
    set_vitesse_moteur(vitesse_BR, sens_BR, id_moteurs::back_right);
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();

}


/**
  * @brief  Set la vitesse d'un moteur (sans appliquer la commande)
  * @param vitesse , vitesse absolue en step/s
  * @param dir , sens de rotation (FWD / BWD)
  * @param id , id du moteur (front_left, front_right, back_left, back_right)
  */
void BlocMoteurs::set_vitesse_moteur(unsigned int vitesse, StepperMotor::direction_t dir, id_moteurs id)
{
    if (!moteurs_arret)
    {
        moteurs[id]->prepare_run(dir, vitesse);
    }
    else
    {
    	moteurs[id]->prepare_hard_hiz(); // mode haute impédence pour pouvoir déplacer le robot à la main
    }
}


/**
  * @brief  Applique une commande de vitesse aux roues en rad/s
  * @param  vitesse_absolue_FL vitesse du moteur Avant gauche
  * @param  vitesse_absolue_FR vitesse du moteur Avant droit
  * @param  vitesse_absolue_BL vitesse du moteur Arriere gauche
  * @param  vitesse_absolue_BR vitesse du moteur Arriere droit
  */
void BlocMoteurs::commande_vitesses_absolues(float vitesse_absolue_FL, float vitesse_absolue_FR, float vitesse_absolue_BL, float vitesse_absolue_BR )
{
	commande_vitesses_normalisees(rad_to_step(vitesse_absolue_FL)/max_vitesse,rad_to_step(vitesse_absolue_FR)/max_vitesse,
								  rad_to_step(vitesse_absolue_BL)/max_vitesse,rad_to_step(vitesse_absolue_BR)/max_vitesse);

}

/**
  * @brief  Applique une consigne de pas aux 4 moteurs
  * !!!!!!!!!!!!!!!!!!!!!!!!! LE NOMBRE DE PAS EFFECTUE DEPEND DU MICROSTEPPING -> 200 EN FULL STEP = 400 EN HALF, 800 EN 1/4.... !!!!!!!!!!!!!!!!!!!!!!
  * @param  nuber_of_step_FL : nombre de pas à effectuer > 0 nombre de pas positif dans le sens du robot
  * @param  nuber_of_step_FR : nombre de pas à effectuer > 0 nombre de pas positif dans le sens du robot
  * @param  nuber_of_step_BL : nombre de pas à effectuer > 0 nombre de pas positif dans le sens du robot
  * @param  nuber_of_step_BR : nombre de pas à effectuer > 0 nombre de pas positif dans le sens du robot 
  */
void BlocMoteurs::commande_step(int number_of_step_FL, int number_of_step_FR, int number_of_step_BL, int number_of_step_BR)
{
  StepperMotor::direction_t dir_FL = (StepperMotor::direction_t) ( StepperMotor::direction_t::FWD*( number_of_step_FL*motor_direction_inverter[id_moteurs::front_left] > 0)
                                   + StepperMotor::direction_t::BWD*( number_of_step_FL*motor_direction_inverter[id_moteurs::front_left] < 0));

  StepperMotor::direction_t dir_FR = (StepperMotor::direction_t) (StepperMotor::direction_t::FWD*( number_of_step_FR*motor_direction_inverter[id_moteurs::front_right] > 0)
                                   + StepperMotor::direction_t::BWD*( number_of_step_FR*motor_direction_inverter[id_moteurs::front_right] < 0));
  
  StepperMotor::direction_t dir_BL = (StepperMotor::direction_t) (StepperMotor::direction_t::FWD*( number_of_step_BL*motor_direction_inverter[id_moteurs::back_left] > 0)
                                   + StepperMotor::direction_t::BWD*( number_of_step_BL*motor_direction_inverter[id_moteurs::back_left] < 0));

  StepperMotor::direction_t dir_BR = (StepperMotor::direction_t) (StepperMotor::direction_t::FWD*( number_of_step_BR*motor_direction_inverter[id_moteurs::back_right] > 0)
                                   + StepperMotor::direction_t::BWD*( number_of_step_BR*motor_direction_inverter[id_moteurs::back_right] < 0));


	set_step_moteur(abs(number_of_step_FL), dir_FL, id_moteurs::front_left);
	set_step_moteur(abs(number_of_step_FR), dir_FR, id_moteurs::front_right);
	set_step_moteur(abs(number_of_step_BL), dir_BL, id_moteurs::back_left);
	set_step_moteur(abs(number_of_step_BR), dir_BR ,id_moteurs::back_right);
  shield_1->perform_prepared_actions();
  shield_2->perform_prepared_actions();

}


/**
  * @brief  Set la vitesse d'un moteur (sans appliquer la commande)
  * @param step , nombre de step
  * @param dir , sens de rotation (FWD / BWD)
  * @param id , id du moteur (front_left, front_right, back_left, back_right)
  */
void BlocMoteurs::set_step_moteur(unsigned int steps, StepperMotor::direction_t dir, id_moteurs id)
{
    if (!moteurs_arret)
    {
        moteurs[id]->prepare_move(dir, steps);
    }
    else
    {
    	moteurs[id]->prepare_hard_hiz(); // mode haute impédence pour pouvoir déplacer le robot à la main
    }
}



/**
  * @brief  autorise les commandes de vitesse jusqu'a l'appel des fonctions motors_stop
  */
void BlocMoteurs::motors_on()
{
    moteurs_arret = 0;
}


/**
  * @brief  stop les roues et les laisse libres par la suite
  * Le passage a la vitesse a zero respecte les parametre de deceleration maximale du robot
  * (contrairement a la methode motors_stop_hard_hiz)
  */
void BlocMoteurs::motors_stop_soft_hiz()
{

	for(int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_soft_hiz();
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();

    moteurs_arret = true;
}

/**
  * @brief  arrête la commande en cours et laisse les roues libres
  */
void BlocMoteurs::motors_stop_hard_hiz()
{

	for(int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_hard_hiz();
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();

    moteurs_arret = true;
}


/**
  * @brief  Stop la consigne des moteurs et les bloques à zero
  * Le passage a la vitesse a zero respecte les parametre de deceleration maximale du robot
  * et se realise au mieux des capacites du driver
  * (contrairement a la methode motors_stop_soft)
  */
void BlocMoteurs::motors_stop_soft()
{

	for(int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_soft_stop();
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();

    moteurs_arret = true;
}

/**
  * @brief  Stop la consigne des moteurs et les bloques à zero
  * Le passage a la vitesse a zero ne respecte pas les parametre de deceleration maximale du robot
  * et se realise au mieux des capacites du driver
  * (contrairement a la methode motors_stop_soft)
  */
void BlocMoteurs::motors_stop_hard()
{

	for(int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_hard_stop();
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();

    moteurs_arret = true;
}


/**
  * @brief  Set la vitesse maximale des moteurs
  *
  * @param  vitesse_rad_s_FL vitesse max (en valeur absolue) du moteur FL
  * @param  vitesse_rad_s_FR vitesse max (en valeur absolue) du moteur FR
  * @param  vitesse_rad_s_BL vitesse max (en valeur absolue) du moteur BL
  * @param  vitesse_rad_s_BR vitesse max (en valeur absolue) du moteur BR
  */
void BlocMoteurs::set_max_speed_moteurs(float vitesse_rad_s_FL, float vitesse_rad_s_FR, float vitesse_rad_s_BL, float vitesse_rad_s_BR)
{
	unsigned int pps[4] = {rad_to_step(fabs(vitesse_rad_s_FL)), rad_to_step(fabs(vitesse_rad_s_FR)), rad_to_step(fabs(vitesse_rad_s_BL)), rad_to_step(fabs(vitesse_rad_s_FR))};
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_max_speed(pps[i]);
    
	}
  shield_1->perform_prepared_actions();
  shield_2->perform_prepared_actions();

}

/**
  * @brief  Set la vitesse minimale des moteurs
  *
  * @param  vitesse_rad_s_FL vitesse min (en valeur absolue) du moteur FL
  * @param  vitesse_rad_s_FR vitesse min (en valeur absolue) du moteur FR
  * @param  vitesse_rad_s_BL vitesse min (en valeur absolue) du moteur BL
  * @param  vitesse_rad_s_BR vitesse min (en valeur absolue) du moteur BR
  */
void BlocMoteurs::set_min_speed_moteurs(float vitesse_rad_s_FL, float vitesse_rad_s_FR, float vitesse_rad_s_BL, float vitesse_rad_s_BR)
{
	unsigned int pps[4] = {rad_to_step(vitesse_rad_s_FL), rad_to_step(vitesse_rad_s_FR), rad_to_step(vitesse_rad_s_BL), rad_to_step(vitesse_rad_s_FR)};
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_min_speed(pps[i]);
	}
  shield_1->perform_prepared_actions();
  shield_2->perform_prepared_actions();

}

/**
  * @brief  Set l'accéleration maximale des moteurs
  *
  * @param  acc_rad_s2_FL accéleration angulaire en rad/s^2 du moteur FL
  * @param  acc_rad_s2_FR accéleration angulaire en rad/s^2 du moteur FR
  * @param  acc_rad_s2_BL accéleration angulaire en rad/s^2 du moteur BL
  * @param  acc_rad_s2_FR accéleration angulaire en rad/s^2 du moteur BR
  */
void BlocMoteurs::set_max_acc_moteurs(float acc_rad_s2_FL, float acc_rad_s2_FR, float acc_rad_s2_BL, float acc_rad_s2_BR)
{
unsigned int pps2[4] = {rad_to_step(acc_rad_s2_FL), rad_to_step(acc_rad_s2_FR), rad_to_step(acc_rad_s2_BL), rad_to_step(acc_rad_s2_BR)};
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_acceleration(pps2[i]);
	}
  shield_1->perform_prepared_actions();
  shield_2->perform_prepared_actions();

}

/**
  * @brief  Set la décéleration maximale des moteurs
  *
  * @param  dec_rad_s2_FL décéleration angulaire en rad/s^2 du moteur FL
  * @param  dec_rad_s2_FR décéleration angulaire en rad/s^2 du moteur FR
  * @param  dec_rad_s2_BL décéleration angulaire en rad/s^2 du moteur BL
  * @param  dec_rad_s2_FR décéleration angulaire en rad/s^2 du moteur BR
  */
void BlocMoteurs::set_max_dec_moteurs(float dec_rad_s2_FL, float dec_rad_s2_FR, float dec_rad_s2_BL, float dec_rad_s2_BR)
{
unsigned int pps2[4] = {rad_to_step(dec_rad_s2_FL), rad_to_step(dec_rad_s2_FR), rad_to_step(dec_rad_s2_BL), rad_to_step(dec_rad_s2_BR)};
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_deceleration(pps2[i]);
	}
  shield_1->perform_prepared_actions();
  shield_2->perform_prepared_actions();

}

/**
  * @brief  Mesure la vitesse des 4 moteurs en radian/s
  *
  * @retval tableau float des 4 valeursen rad/s des vitesse. Les index correspondent à l'enum motor id
  * ex : id_moteurs::front_left = 0 -> to_return[0] = vitesse du moteur Avant gauche (front left)
  */
float* BlocMoteurs::mesure_vitesses_rad()
{
  static float to_return[NMOTEURS];
  int32_t* moteurs_speed_step = mesure_vitesses_step();
  for( int i = 0; i<4; i ++)
  {
    to_return[i] = step_to_rad( moteurs_speed_step[i]);
  }
  return to_return;
}

/**
  * @brief  Mesure la vitesse des 4 moteurs en full step/s (independant du microstepping)
  *
  * @retval tableau float des 4 valeursen rad/s des vitesse. Les index correspondent à l'enum motor id
  * ex : id_moteurs::front_left = 0 -> to_return[0] = vitesse du moteur Avant gauche (front left)
  */
int32_t * BlocMoteurs::mesure_vitesses_step()
{
	static int32_t to_return[NMOTEURS];
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_speed();
	}
	uint32_t* result1 = shield_1->perform_prepared_actions();
	uint32_t* result2 = shield_2->perform_prepared_actions();
  uint32_t results[4] = {result1[0], result1[1], result2[0], result2[1]};

	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_direction();
	}
	uint32_t* result1_dir = shield_1->perform_prepared_actions();
	uint32_t* result2_dir = shield_2->perform_prepared_actions();
  StepperMotor::direction_t results_dir[4] = {(StepperMotor::direction_t)result1_dir[0], 
                                              (StepperMotor::direction_t)result1_dir[1], 
                                              (StepperMotor::direction_t)result2_dir[0], 
                                              (StepperMotor::direction_t)result2_dir[1]};


  for( int i = 0; i< 4; i ++)
  {
    int32_t multiplier = 1*(results_dir[i]==StepperMotor::FWD) -1*(results_dir[i]==StepperMotor::BWD);
    multiplier*= motor_direction_inverter[index_to_enum[i]];

    to_return[ index_to_enum[i] ] =  multiplier*(results[i]); //utilisation du tableau de correspondance pour assigner la bonne vitesse au bon moteur
  }

	return to_return;
}

/**
  * @brief  Mesure des pas ecoulees des 4 moteurs en microstep (selon le mode selectionné) depuis le dernier appel de la fonction
  *
  * @retval tableau float des 4 valeursen des pas ecoulees. Les index correspondent à l'enum motor id
  * ex : id_moteurs::front_left = 0 -> to_return[0] = nombre de pas ecoulees du moteur Avant gauche (front left)
  */
int32_t* BlocMoteurs::mesure_pas_ecoule()
{
  static uint32_t last_values[NMOTEURS] = {0,0,0,0};
  static int32_t to_return[NMOTEURS];
  
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_position();
	}
	uint32_t* result1 = shield_1->perform_prepared_actions();
	uint32_t* result2 = shield_2->perform_prepared_actions();
  uint32_t results[4] = {result1[0], result1[1], result2[0], result2[1]};
	
  for( int i = 0; i < NMOTEURS; i ++)
	{
		int32_t diff = results[i] - last_values[i] ;
		if( diff > std::pow(2,21)-1)
		{
		  diff-=std::pow(2,22);

		}
		else if( diff < -std::pow(2,21) )
		{
		  diff+=std::pow(2,22);
		}
		to_return[index_to_enum[i]] = motor_direction_inverter[index_to_enum[i]]*( diff);
		last_values[i] = results[i];
	}
  return to_return;

}

bool BlocMoteurs::get_busy()
{

	uint32_t motr0_status = moteurs[0]->get_status();
	uint32_t motr1_status = moteurs[1]->get_status();
	uint32_t motr2_status = moteurs[2]->get_status();
	uint32_t motr3_status = moteurs[3]->get_status();



  sL6470_StatusRegister_t* status_registers[4] = {(sL6470_StatusRegister_t*) &motr0_status,(sL6470_StatusRegister_t*)&motr1_status,(sL6470_StatusRegister_t*)&motr2_status,(sL6470_StatusRegister_t*)&motr3_status};

  return !status_registers[0]->BUSY ||  !status_registers[1]->BUSY || !status_registers[2]->BUSY || !status_registers[3]->BUSY;
}
