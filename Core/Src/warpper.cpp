/*
 * warpper.cpp
 *
 *  Created on: 2023/03/12
 *      Author: IndigoCarmine
 */

#include "main.h"

#include "motor.hpp"
using namespace three_phase_motor;

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void callback(CallbackState state)
{
    // Do something.
}

void main_cpp(void)
{
    MotorConfig config;
    config.number_of_pole = 4;

    Motor motor = Motor(htim1, htim3, hadc2, config, callback);
}