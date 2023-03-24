
namespace can
{
    enum class MotorState : uint8_t
    {
        Free,
        Hold,
        Homing,
        PositionControl,
        PositionEquivalentControl,
        VelocityControl,
        CurrentControl
    };

    enum class CallbackState : uint8_t
    {
        TouchLimitSwitch,
        FinishedHoming,
        ExcessPositionLimitation,
        MotorIsNotConnected,
        InvalidParameter,
        UnknownError,
        NotImplimented,
        Debug
    };

    class ICanCommunicator
    {
    public:
        virtual void setValue(uint32_t value,uint8_t id_offset){};
    };
} // namespace can
