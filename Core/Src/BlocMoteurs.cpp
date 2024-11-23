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
		GPIO_TypeDef* standby_reset_port_shield_2  ,uint16_t standby_reset_pin_shield_2 ,GPIO_TypeDef* ssel_port_shield_2 , uint16_t ssel_pin_shield_2)
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

	#ifdef CANONICAL_DIR // inverse ou non shield du haut/bas
		moteurs[front_left] = moteurs_shield_1[left];
		moteurs[front_right] = moteurs_shield_1[right];
		moteurs[back_left] = moteurs_shield_2[left];
		moteurs[back_right] = moteurs_shield_2[right];
	#else
		moteurs[front_left] = moteurs_shield_2[left];
		moteurs[front_right] = moteurs_shield_2[right];
		moteurs[back_left] = moteurs_shield_1[1];
		moteurs[back_right] = moteurs_shield_1[0];
	#endif



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

    // Détermination du signe des vitesses (logique inversée entre les deux moteurs car placés symétriquement sur le robot et branchement identique)
    StepperMotor::direction_t sens_FL = vitesse_normalisee_FL >= 0.0f ? StepperMotor::FWD : StepperMotor::BWD;;
    StepperMotor::direction_t sens_FR = vitesse_normalisee_FR >= 0.0f ? StepperMotor::BWD : StepperMotor::FWD;
    StepperMotor::direction_t sens_BL = vitesse_normalisee_BL >= 0.0f ? StepperMotor::FWD : StepperMotor::BWD;
    StepperMotor::direction_t sens_BR = vitesse_normalisee_BR >= 0.0f ? StepperMotor::BWD : StepperMotor::FWD;


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
  * @brief  Applique la meme commande de nombre de pas aux roues dans les directions indiquees
  * !!!!!!!!!!!!!!!!!!!!!!!!! LE NOMBRE DE PAS EFFECTUE DEPEND DU MICROSTEPPING -> 200 EN FULL STEP = 400 EN HALF, 800 EN 1/4.... !!!!!!!!!!!!!!!!!!!!!!
  * @param  number_of_step nombre de pas duquel tourner
  * @param  dir_FL direction du moteur avant gauche, FWD:sens trigo BWD: sens horaire
  * @param  dir_FR direction du moteur avant droite, FWD:sens trigo BWD: sens horaire
  * @param  dir_BL direction du moteur arriere gauche, FWD:sens trigo BWD: sens horaire
  * @param  dir_BR direction du moteur arriere droite, FWD:sens trigo BWD: sens horaire
  */
void BlocMoteurs::commande_step(unsigned int number_of_step, direction_t dir_FL ,direction_t dir_FR, direction_t dir_BL, direction_t dir_BR)
{
	set_step_moteur(number_of_step, dir_FL, id_moteurs::front_left);
	set_step_moteur(number_of_step, dir_FR, id_moteurs::front_right);
	set_step_moteur(number_of_step, dir_BL, id_moteurs::back_left);
	set_step_moteur(number_of_step, dir_BR, id_moteurs::back_right);
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
  * @brief  Avance le robot en faisant rouler les roues du meme nombre de pas dans le sens qui convient
  * !!!!!!!!!!!!!!!!!!!!!!!!! LE NOMBRE DE PAS EFFECTUE DEPEND DU MICROSTEPPING -> 200 EN FULL STEP = 400 EN HALF, 800 EN 1/4.... !!!!!!!!!!!!!!!!!!!!!!
  * @param number_of_step , nombre de step duquel avancer
  */
void BlocMoteurs::avancer_step( unsigned int number_of_step)
{
	commande_step(number_of_step, direction_t::FWD, direction_t::BWD, direction_t::FWD, direction_t::BWD);
}


/**
  * @brief  Recule le robot en faisant rouler les roues du meme nombre de pas dans le sens qui convient
  * !!!!!!!!!!!!!!!!!!!!!!!!! LE NOMBRE DE PAS EFFECTUE DEPEND DU MICROSTEPPING -> 200 EN FULL STEP = 400 EN HALF, 800 EN 1/4.... !!!!!!!!!!!!!!!!!!!!!!
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::reculer_step( unsigned int number_of_step)
{
	commande_step(number_of_step, direction_t::BWD, direction_t::FWD, direction_t::BWD, direction_t::FWD);
}

/**
  * @brief  Translate a gauche le robot en faisant rouler les roues du meme nombre de pas dans le sens qui convient
  * !!!!!!!!!!!!!!!!!!!!!!!!! LE NOMBRE DE PAS EFFECTUE DEPEND DU MICROSTEPPING -> 200 EN FULL STEP = 400 EN HALF, 800 EN 1/4.... !!!!!!!!!!!!!!!!!!!!!!
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::gauche_step( unsigned int number_of_step)
{
	#ifdef XMECANUM_SHAPE
		commande_step(number_of_step, direction_t::BWD, direction_t::BWD, direction_t::FWD, direction_t::FWD);
	#else
		commande_step(number_of_step, direction_t::FWD, direction_t::FWD, direction_t::BWD, direction_t::BWD);
	#endif


}

/**
  * @brief  Translate a droite le robot en faisant rouler les roues du meme nombre de pas dans le sens qui convient
  * !!!!!!!!!!!!!!!!!!!!!!!!! LE NOMBRE DE PAS EFFECTUE DEPEND DU MICROSTEPPING -> 200 EN FULL STEP = 400 EN HALF, 800 EN 1/4.... !!!!!!!!!!!!!!!!!!!!!!
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::droite_step( unsigned int number_of_step)
{
	#ifdef XMECANUM_SHAPE
		commande_step(number_of_step, direction_t::FWD, direction_t::FWD, direction_t::BWD, direction_t::BWD);
	#else
		commande_step(number_of_step, direction_t::BWD, direction_t::BWD, direction_t::FWD, direction_t::FWD);
	#endif
}

/**
  * @brief  Avance le robot en faisant rouler les roues a la meme vitesse en rad/s dans le sens qui convient
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::avancer_vitesse_abs(float vitesses_roues_rad_par_sec)
{
	commande_vitesses_normalisees(rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,
								  rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse);
}

/**
  * @brief  Recule le robot en faisant rouler les roues a la meme vitesse en rad/s dans le sens qui convient
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::reculer_vitesse_abs(float vitesses_roues_rad_par_sec)
{
	commande_vitesses_normalisees(-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,
								  -rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse);
}

/**
  * @brief  Translate a gauche le robot en faisant rouler les roues a la meme vitesse en rad/s dans le sens qui convient
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::gauche_vitesse_abs(float vitesses_roues_rad_par_sec)
{
	#ifdef XMECANUM_SHAPE
		commande_vitesses_normalisees(-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,
									  rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse);
	#else
		commande_vitesses_normalisees(rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,
									  -rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse);
	#endif
}

/**
  * @brief  Translate a droite le robot en faisant rouler les roues a la meme vitesse en rad/s dans le sens qui convient
  * @param number_of_step , nombre de step duquel realiser le deplacement
  */
void BlocMoteurs::droite_vitesse_abs(float vitesses_roues_rad_par_sec)
{
	#ifdef XMECANUM_SHAPE
		commande_vitesses_normalisees(rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,
									  -rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse);
	#else
		commande_vitesses_normalisees(-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,
									  rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse,-rad_to_step(vitesses_roues_rad_par_sec)/max_vitesse);

	#endif
}

/**
  * @brief  Avance le robot en faisant rouler les roues a la meme vitesse normalisee dans le sens qui convient
  * @param vitesse_normalisee , vitesse normalisee a laquelle realiser le deplacement
  */
void BlocMoteurs::avancer_vitesse_normalisee(float vitesse_normalisee)
{
		commande_vitesses_normalisees(vitesse_normalisee,vitesse_normalisee,
									  vitesse_normalisee,vitesse_normalisee);
}

/**
  * @brief  Recule le robot en faisant rouler les roues a la meme vitesse normalisee dans le sens qui convient
  * @param vitesse_normalisee , vitesse normalisee a laquelle realiser le deplacement
  */
void BlocMoteurs::reculer_vitesse_normalisee(float vitesse_normalisee)
{
		commande_vitesses_normalisees(-vitesse_normalisee,-vitesse_normalisee,
									  -vitesse_normalisee,-vitesse_normalisee);
}

/**
  * @brief  Translate le robot a gauche en faisant rouler les roues a la meme vitesse normalisee dans le sens qui convient
  * @param vitesse_normalisee , vitesse normalisee a laquelle realiser le deplacement
  */
void BlocMoteurs::gauche_vitesse_normalisee(float vitesse_normalisee)
{
	#ifdef XMECANUM_SHAPE
		commande_vitesses_normalisees(-vitesse_normalisee,vitesse_normalisee,
									  vitesse_normalisee,vitesse_normalisee);
	#else
		commande_vitesses_normalisees(vitesse_normalisee,-vitesse_normalisee,
									  -vitesse_normalisee,vitesse_normalisee);
	#endif
}

/**
  * @brief  Translate le robot a droite en faisant rouler les roues a la meme vitesse normalisee dans le sens qui convient
  * @param vitesse_normalisee , vitesse normalisee a laquelle realiser le deplacement
  */
void BlocMoteurs::droite_vitesse_normalisee(float vitesse_normalisee)
{
	#ifdef XMECANUM_SHAPE
		commande_vitesses_normalisees(vitesse_normalisee,-vitesse_normalisee,
									  -vitesse_normalisee,vitesse_normalisee);
	#else
		commande_vitesses_normalisees(-vitesse_normalisee,vitesse_normalisee,
									  vitesse_normalisee,-vitesse_normalisee);
	#endif
}

/**
  * @brief  Definit le mode de microstepping. Plus le diviseur (FULL < HALF < 4  < 8 ... < 128)
  * est haut, plus petit est le déplacement angulaire.
  * Ainsi, plus de finesse sur le deplacement (finesse != precision) est grande.
  * Un mode haut de microstepping degrade cependant la consommation et le couple effectif du moteur
  * @param  step_mode le mode de microstepping souhaite
  */
bool BlocMoteurs::set_microstepping_mode(step_mode_t step_mode)
{

    initShield1[0].step_sel = step_mode;
    initShield1[1].step_sel = step_mode;
    initShield2[0].step_sel = step_mode;
    initShield2[0].step_sel = step_mode;
    bool return_value = true;
    for( int i = 0; i < NMOTEURS; i ++)
    {
    	if(! moteurs[i]->set_step_mode(step_mode))
    	{
    		return_value = false;
    	}
    }
    return return_value;

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
  * @brief  stop les roues et les laisse libres par la suite
  * Le passage a la vitesse a zero ne respecte pas les parametre de deceleration maximale du robot
  * et se realise au mieux des capacitees du driver
  * (contrairement a la methode motors_stop_soft_hiz)
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
 * @brief Permet de faire un pas complet sur chaque stepper
 * sans bouger le robot, afin d'arrondir ma position du stepper à une position correspondant à un pas complet
**/
void BlocMoteurs::step_ceil()
{
	step_mode_t step_mode = (step_mode_t) initShield1[0].step_sel;
	if(step_mode != step_mode_t::STEP_MODE_FULL)
	{
		set_microstepping_mode(step_mode_t::STEP_MODE_FULL);
	}
    moteurs[front_right]->prepare_move(direction_t::FWD, 1);
    moteurs[front_left]->prepare_move(direction_t::FWD, 1);
    moteurs[back_right]->prepare_move(direction_t::BWD, 1);
    moteurs[back_left]->prepare_move(direction_t::BWD, 1);
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();
	if(step_mode != step_mode_t::STEP_MODE_FULL)
	{
		set_microstepping_mode(step_mode);
	}

}

/**
  * @brief  Set la vitesse maximale des moteurs
  *
  * @param  radian_par_seconde la valeur maximale de vitesse angulaire en rad/s
  */
void BlocMoteurs::set_max_speed_moteurs(float radian_par_seconde)
{
	unsigned int pps = rad_to_step(radian_par_seconde);
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_max_speed(pps);
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();
	initShield1[0].maxspeed = pps;
	initShield1[1].maxspeed = pps;
	initShield2[0].maxspeed = pps;
	initShield2[1].maxspeed = pps;
	max_vitesse = pps;
}

/**
  * @brief  Set la vitesse minimale des moteurs
  *
  * @param  radian_par_seconde la valeur minimale de vitesse angulaire en rad/s
  */
void BlocMoteurs::set_min_speed_moteurs(float radian_par_seconde)
{
	unsigned int pps = rad_to_step(radian_par_seconde);
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_min_speed(pps);
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();
	initShield1[0].minspeed = pps;
	initShield1[1].minspeed = pps;
	initShield2[0].minspeed = pps;
	initShield2[1].minspeed = pps;
}

/**
  * @brief  Set l'accéleration maximale des moteurs
  *
  * @param  radian_par_seconde_carre la valeur maximale de l'accéleration angulaire en rad/s^2
  */
void BlocMoteurs::set_max_acc_moteurs(float radian_par_seconde2)
{
	unsigned int pps2 = rad_to_step(radian_par_seconde2);
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_acceleration(pps2);
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();
	initShield1[0].acc = pps2;
	initShield1[1].acc = pps2;
	initShield2[0].acc = pps2;
	initShield2[1].acc = pps2;
}

/**
  * @brief  Set la deceleration maximale des moteurs
  *
  * @param  radian_par_seconde_carre la valeur maximale de la décélération angulaire en rad/s^2
  */
void BlocMoteurs::set_max_dec_moteurs(float radian_par_seconde2)
{
	unsigned int pps2 = rad_to_step(radian_par_seconde2);
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_set_deceleration(pps2);
	}
    shield_1->perform_prepared_actions();
    shield_2->perform_prepared_actions();
	initShield1[0].dec = pps2;
	initShield1[1].dec = pps2;
	initShield2[0].dec = pps2;
	initShield2[1].dec = pps2;
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
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_speed();
	}
	uint32_t* result1 = shield_1->perform_prepared_actions();
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_speed();
	}
	uint32_t* result2 = shield_2->perform_prepared_actions();
	#ifdef CANONICAL_DIR // inverse ou non shield du haut/bas
		to_return[front_left] = step_to_rad(result1[left]);
		to_return[front_right] = step_to_rad(result1[right]);
		to_return[back_left] = step_to_rad(result2[left]);
		to_return[back_right] = step_to_rad(result2[right]);
	#else
		to_return[front_left] = step_to_rad(result2[left]);
		to_return[front_right] = step_to_rad(result2[right]);
		to_return[back_left] = step_to_rad(result1[left]);
		to_return[back_right] = step_to_rad(result1[right]);
	#endif

	return to_return;
}

/**
  * @brief  Mesure la vitesse des 4 moteurs en full step/s (independant du microstepping)
  *
  * @retval tableau float des 4 valeursen rad/s des vitesse. Les index correspondent à l'enum motor id
  * ex : id_moteurs::front_left = 0 -> to_return[0] = vitesse du moteur Avant gauche (front left)
  */
uint32_t * BlocMoteurs::mesure_vitesses_step()
{
	static uint32_t to_return[NMOTEURS];
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_speed();
	}
	uint32_t* result1 = shield_1->perform_prepared_actions();
	for( int i = 0; i < NMOTEURS; i ++)
	{
		moteurs[i]->prepare_get_speed();
	}
	uint32_t* result2 = shield_2->perform_prepared_actions();
	#ifdef CANONICAL_DIR // inverse ou non shield du haut/bas
		to_return[front_left] = result1[left];
		to_return[front_right] = result1[right];
		to_return[back_left] = result2[left];
		to_return[back_right] = result2[right];
	#else
		to_return[front_left] = result2[left];
		to_return[front_right] = result2[right];
		to_return[back_left] = result1[left];
		to_return[back_right] = result1[right];
	#endif

	return to_return;
}
