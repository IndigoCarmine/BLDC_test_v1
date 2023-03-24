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
#include "can_interface.hpp"
using namespace can;

#include <math.h>
#include <functional>

namespace three_phase_motor
{

    // It is mutable info of motor.
    struct MotorSettings
    {
        float position;
        float velocity;
        float current;
    };

    // It is no-mutable info of motor.
    struct MotorConfig
    {
        uint8_t number_of_pole;
        float velocity_to_friqency;
        // if true, motor will stop when touch limit switch.
        // And motor will not stop when homing whether it is true or false.
        bool stop_when_touch_limit_switch = true;
    };

    // This is a base class of motor. It is not used directly.
    class MotorBase
    {
        /*
        naming rule
        - **Limitation: It is function for calculation of limitation. It will be called by update().
        - update**: It is function for update motor control values. It will be called by update().
        - **Handler: It is function for interrupt handler. It will be called by interrupt.
        - **Callback: It is function for callback. It will be called by update().

        I'm sorry for my bad naming rule. If you have any idea, please tell me.
        */

    public:
        // Hall sensor tim has 16bit counter.??
        MotorBase(TIM_HandleTypeDef &pwm_timer, TIM_HandleTypeDef &hall_sensor, ADC_HandleTypeDef &hadc,
                  MotorConfig config, std::function<void(CallbackState)> callback)
            : pwm_timer(&pwm_timer), hall_sensor(&hall_sensor), hadc(&hadc), callback(callback), config(config)
        {
        }

        // setter
        void setLimitation(MotorSettings &limitation)
        {
            this->limitation = limitation;
        }
        void setTarget(MotorSettings &target)
        {
            this->target = target;
        }

        virtual void resetPosition(void) {}

        void changeState(MotorState state)
        {
            if (this->state == state)
                return;

            this->state = state;
            switch (state)
            {
            case MotorState::Free:
                motorStop();
                break;
            case MotorState::Hold:
                // TODO: how to hold motor?
                // Can I directly access gpio pin? HAL_GPIO_WritePin(...);
                motorStop();
                break;
            default:
                // Other state don't need to do anything.
                break;
            }
        }
        MotorState getState()
        {
            return state;
        }

        // it will be called by GPIO interrupt.
        void limitSwitchHandler(void)
        {
            if (state == MotorState::Homing)
            {
                motorStop();
                resetPosition();
                changeState(MotorState::Hold);
                callback(CallbackState::FinishedHoming);
            }
            else
            {
                // not homing
                // it may be dangerous.
                if (config.stop_when_touch_limit_switch)
                {
                    motorStop();
                    changeState(MotorState::Hold);
                }
                callback(CallbackState::TouchLimitSwitch);
            }
        }

        void motorStart(void)
        {
            start_pwm(*pwm_timer);
            is_running = true;
        }

        void motorStop(void)
        {
            stop_pwm(*pwm_timer);
            is_running = false;
        }

        // TODO: Write!!!!!!!
        //  it will be called by main loop.
        void update(void)
        {
            if (!is_running)
                return;

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
                updateCurrent();
            }
            break;
            default:
            {
                // if pass through, it is error.
                motorStop();
                callback(CallbackState::UnknownError);
                state = MotorState::Free;
            }
            break;
            }
        }

    protected:
        virtual uint32_t getPosition(void) {}

        virtual void updateVelocity(void) {}

        // TODO: is it correct? It is from copilot.
        float getAngle(void)
        {
            return getPosition() * 2 * M_PI / config.number_of_pole;
        }

        // handle
        TIM_HandleTypeDef *pwm_timer;
        TIM_HandleTypeDef *hall_sensor;
        ADC_HandleTypeDef *hadc;

        const MotorConfig config;

        // for post state or error to outside.
        // TODO: Do std::function work on stm32?
        // it will be called by interrupt. So It should be fast.
        std::function<void(CallbackState)> callback;

        MotorState state = MotorState::Free;
        // it will use as max values.
        MotorSettings limitation;
        MotorSettings target;

        bool is_running = false;

    private:
        void updateCurrent()
        {
            static float last_scaler = 1;
            if (target.current == INFINITY)
            {
                last_scaler = 1;
                return;
            }
            float current = HAL_ADC_GetValue(hadc);
            float scaler = target.current / current;
            uint8_t direction = 0;
            if (scaler < 0)
                direction = 1;
            scaler = fabs(scaler);
            if (scaler > 1)
                scaler = 1;
            update_table(scaler, direction);
            last_scaler = scaler;
        }

        // HACK: These name is not good. Think about a correspodece between currentLimitation() and positionLimitation().
        // culculate pwm duty from current.
        void currentLimitation(void)
        {
            static float last_scaler = 1;
            if (limitation.current == INFINITY)
            {
                last_scaler = 1;
                return;
            }
            float current = HAL_ADC_GetValue(hadc);
            float scaler = limitation.current / current;
            uint8_t direction = 0;
            if (scaler < 0)
                direction = 1;
            scaler = fabs(scaler);
            if (scaler > 1)
                scaler = 1;
            update_table(scaler, direction);
            last_scaler = scaler;
        }

        void positionLimitation(void)
        {
            if (limitation.position == INFINITY)
                return;
            float position = getAngle();
            if (position > limitation.position)
            {
                motorStop();
                changeState(MotorState::Hold);
                callback(CallbackState::ExcessPositionLimitation);
            }
        }
    };

    class Motor : public MotorBase
    {

    public:
        Motor(TIM_HandleTypeDef &pwm_timer, TIM_HandleTypeDef &hall_sensor, ADC_HandleTypeDef &hadc,
              MotorConfig config, std::function<void(CallbackState)> callback)
            : MotorBase(pwm_timer, hall_sensor, hadc, config, callback)
        {
        }

        // TODO: It is generated by copilot. Check it.
        // interrupt handler
        void hallOverflowHandler(TIM_HandleTypeDef *htim)
        {
            // DIR is 0, it is count up, maybe?
            if (htim->Instance->DIER == 0)
            {
                hall_overflow_count++;
            }
            else
            {
                hall_overflow_count--;
            }
        }

    private:
        int hall_overflow_count = 0;
        void resetPosition(void) override
        {
            hall_sensor->Instance->CNT = 0;
            hall_overflow_count = 0;
        }

        // TODO: It will be overflow, so it is not good. how to fix it?
        uint32_t getPosition(void) override
        {
            return hall_sensor->Instance->CNT + hall_overflow_count * 0xFF;
        }

        void updateVelocity(void) override
        {
        }
    };

    class SensorlessMotor : public MotorBase
    {
    public:
        SensorlessMotor(TIM_HandleTypeDef &pwm_timer, TIM_HandleTypeDef &hall_sensor, ADC_HandleTypeDef &hadc,
                        MotorConfig config, std::function<void(CallbackState)> callback)
            : MotorBase(pwm_timer, hall_sensor, hadc, config, callback)
        {
        }

        void resetPosition(void) override
        {
            position = 0;
        }
        uint32_t getPosition(void) override
        {
            return position;
        }

    private:
        float position = 0;
    };

} // namespace motor
