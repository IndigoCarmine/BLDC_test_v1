/*
 * motor.cpp
 *
 *  Created on: 2023/03/12
 *      Author: IndigoCarmine
 */

#include "main.h"
#include "3phase_pwm.hpp"
#include "current_feedback.hpp"
#include "park_transform.hpp"

#include <math.h>




namespace three_phase_motor
{
    enum class MotorState
    {
        Free,
        Hold,
        Homing,
        PositionControl,
        PositionEquivalentControl,
        VelocityControl,
        CurrentControl
    };
    struct MotorSettings
    {
        float Velocity;
        float Current;
    };
    struct MotorConfig{
        uint8_t number_of_pole;
        float velocity_to_friqency;

    };

    class MotorBase
    {
        TIM_HandleTypeDef *pwm_timer;
        TIM_HandleTypeDef *hall_sensor;
        ADC_HandleTypeDef *hadc;

        const MotorConfig config;


        MotorState state = MotorState::Free;
        //it will use as max values.
        MotorSettings limitation;
        MotorSettings target;
        int hall_overflow_count = 0;

        void position_control_update(void)
        {}
        void position_equivalent_control_update(void)
        {}
        void velocity_control_update(void)
        {}
        void current_control_update(void)
        {
            Vector2D current = get_current_vector(*hadc, *pwm_timer);
            park_transform(&current,get_angle());


            

        }
        public:
        //setter
        void set_limitation(MotorSettings limitation)
        {
            this->limitation = limitation;
        }
        void set_target(MotorSettings target)
        {
            this->target = target;
        }

        void reset_position(void)
        {
            hall_sensor->Instance->CNT = 0;
            hall_overflow_count = 0;
        }

        void change_state(MotorState state)
        {
            this->state = state;
            switch (state)
            {
            case MotorState::Free:
                stop_pwm(*pwm_timer);
                break;
            case MotorState::Hold:
                //TODO: how to hold motor? 
                //Can I directly access gpio pin? HAL_GPIO_WritePin(...);
                stop_pwm(*pwm_timer);
                break;
            case MotorState::Homing:
                {
                    //TODO: check limit switch gpio pin
                    HAL_NVIC_EnableIRQ(EXTI_GPIOA);
                    velocity_control_update();
                }
                break;
            case MotorState::PositionControl:
                {
                    position_control_update();
                }
            case MotorState::PositionEquivalentControl:
                {
                    position_equivalent_control_update();
                }
            case MotorState::VelocityControl:
                {
                    velocity_control_update();  
                }
            case MotorState::CurrentControl:
                {
                    current_control_update();
                }
                break;
            default:
                break;
            }
        }

        //TODO: It is generated by copilot. Check it.
        //interrupt handler
        void hall_overflow_handler(TIM_HandleTypeDef *htim)
        {
            //DIR is 0, it is count up, maybe?
            if(htim->Instance->DIR == 0)
            {
                hall_overflow_count++;
            }
            else
            {
                hall_overflow_count--;
            }
        }

        void limit_switch_handler(void)
        {
            //moter stop
            stop_pwm(*pwm_timer);

            reset_position();

            //disable interrupt
            //TODO: check it
            HAL_NVIC_DisableIRQ(EXTI_GPIOA);

            //change state
            change_state(MotorState::Hold);

        }

        //it will be called by main loop.
        void update(void)
        {
            switch (state)
            {
            case MotorState::Homing:
                {
                }
                break;
            case MotorState::PositionControl:
                {
                }
            case MotorState::PositionEquivalentControl:
                {
                }
            case MotorState::VelocityControl:
                {
                }
            case MotorState::CurrentControl:
                {
                }
                break;
            default:
                //it will be free or hold.

                break;
            }
        }

        //TODO: It will be overflow, so it is not good. how to fix it?
        uint32_t get_position(void)
        {

            return hall_sensor->Instance->CNT + hall_overflow_count*0xFF;
        }

        //TODO: is it correct? It is from copilot.
        float get_angle(void)
        {
            return get_position() * 2 * M_PI / config.number_of_pole;
        }

        //Hall sensor tim has 16bit counter.??
        MotorBase(TIM_HandleTypeDef &pwm_timer,TIM_HandleTypeDef &hall_sensor,ADC_HandleTypeDef &hadc, MotorConfig config):
            pwm_timer(&pwm_timer),
            hall_sensor(&hall_sensor),
            hadc(&hadc),
            config(config)
        {

        }
        


    };

    class Motor : public MotorBase
    {

        public:
        Motor(TIM_HandleTypeDef &pwm_timer,TIM_HandleTypeDef &hall_sensor,ADC_HandleTypeDef &hadc, MotorConfig config):
            MotorBase(pwm_timer,hall_sensor,hadc,config)
        {

        }

    };


    class SensorlessMotor : public MotorBase
    {

    };


} // namespace motor
