#include "AdsLib.h"
#include "AdsNotificationOOI.h"
#include "AdsVariable.h"
#include <motor_control/VelocitySetpoints.h>
#include <motor_control/AngularPositions.h>

class AdsMotorController
{
    private:
        int max_velocity = 1100; //Determined by plc

        static AdsDevice *route;

        static AdsVariable<std::array<double, 4>> *_read_position_data_ADS;
         std::array<double, 4> read_position_data;

        static AdsVariable<std::array<double, 4>> *_write_position_data_ADS;
        std::array<double, 4> write_position_data;

    public:
        AdsMotorController(/* args */);
        ~AdsMotorController();
        motor_control::AngularPositions getData();
        void setData(motor_control::VelocitySetpoints p_velocity_setpoints);

    protected:
        void connectToMotor();
        //void readByName(std::ostream& out,const AdsDevice& route,float* rData);
        //void readWriteArray(std::ostream& out, const AdsDevice& route);
};
