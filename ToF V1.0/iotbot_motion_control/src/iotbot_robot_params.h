#ifndef IOTBOT_MOTION_CONTROL__ROBOT_PARAMS_H_
#define IOTBOT_MOTION_CONTROL__ROBOT_PARAMS_H_


namespace iotbot {


/**
 * @struct Structure for encapsulating motor parameters
 * @author Stefan May, David Grenner, Manuel Kreutz, Christoph Kögel, Antonello Pastore
 * @date 01.09.2021
 */
struct MotorParams
{
    float gearRatio;
    float encoderRatio;
    float rpmMax;

    /**
     * Standard constructor assigns default parameters
     */
    MotorParams()
    {
        gearRatio       = 0.f;
        encoderRatio    = 0.f;
        rpmMax          = 0.00f;
    }

    /**
     * Copy constructor
     * @param[in] p parameter instance to be copied
     */
    MotorParams(const MotorParams & p)
    {
        gearRatio       = p.gearRatio;
        encoderRatio    = p.encoderRatio;
        rpmMax          = p.rpmMax;
    }
};


/**
 * @struct Structure for encapsulating parameters of robot chassis
 * @author Stefan May, David Grenner, Manuel Kreutz, Christoph Kögel, Antonello Pastore
 * @date 01.09.2021
 */
struct ChassisParams
{
    float   track;
    float   wheelBase;
    float   wheelDiameter;
    int     direction;
    uint8_t chFrontLeft;
    uint8_t chFrontRight;
    uint8_t chRearLeft;
    uint8_t chRearRight;

    ChassisParams()
    {
        track           = 0.f;
        wheelBase       = 0.f;
        wheelDiameter   = 0.f;
        direction       = 0;
        chFrontLeft     = 0;
        chFrontRight    = 0;
        chRearLeft      = 0;
        chRearRight     = 0;
    }
};


/**
 * @struct Structure for encapsulating parameters of robot properties
 * @author David Grenner
 * @date 04.04.2021
 */
struct RobotConversionParams
{
    float vMax;      // maximum velocity [m/s]
    float omegaMax;  // maximum rotating rate [rad]
    float ms2rpm;    // conversion from [m/s] to revolutions per minute [RPM]
    float rpm2ms;    // conversion from revolutions per minute [RPM] to [m/s]
    float rad2rpm;   // conversion from [rad/s] to revolutions per minute [RPM]
    float rpm2rad;   // conversion from revolutions per minute [RPM] to [rad/s]

    RobotConversionParams()
    {
        vMax        = 0.0f;
        omegaMax    = 0.0f;
        ms2rpm      = 0.0f;
        rpm2ms      = 0.0f;
        rad2rpm     = 0.0f;
        rpm2rad     = 0.0f;
    }
};


} // namespace iotbot


#endif // IOTBOT_MOTION_CONTROL__ROBOT_PARAMS_H_