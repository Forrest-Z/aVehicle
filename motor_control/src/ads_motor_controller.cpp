#include <motor_control/ads_motor_controller.h>

// for static member
AdsDevice *AdsMotorController::route;
AdsVariable<std::array<double, 4>> *AdsMotorController::_read_position_data_ADS;
AdsVariable<std::array<double, 4>> *AdsMotorController::_write_position_data_ADS;

AdsMotorController::AdsMotorController() 
{
    try {
        connectToMotor();
    } catch (const AdsException& ex) {
        std::cout << "Error: " << ex.errorCode << "\n";
        std::cout << "AdsException message: " << ex.what() << "\n";
    } catch (const std::runtime_error& ex) {
        std::cout << ex.what() << '\n';
    }
}

AdsMotorController::~AdsMotorController() {}

motor_control::AngularPositions AdsMotorController::getData()
{
    read_position_data = *_read_position_data_ADS; // read data from plc

     motor_control::AngularPositions _angularPosition;
    _angularPosition.left_motor  = read_position_data[0]/100.0; // The received data is in mms
    _angularPosition.right_motor = read_position_data[1]/100.0; // The received data should be in ms

     return _angularPosition;
}

void AdsMotorController::setData(motor_control::VelocitySetpoints p_velocity_setpoints)
{
        if (p_velocity_setpoints.left_motor >= max_velocity)
            p_velocity_setpoints.left_motor = max_velocity;
        if (p_velocity_setpoints.left_motor <= -1*max_velocity)
            p_velocity_setpoints.left_motor = -1*max_velocity;
        if (p_velocity_setpoints.right_motor >= max_velocity)
            p_velocity_setpoints.right_motor = max_velocity;
        if (p_velocity_setpoints.right_motor <= -1*max_velocity)
            p_velocity_setpoints.right_motor = -1*max_velocity;


    write_position_data[2] = p_velocity_setpoints.left_motor;
    write_position_data[3] = p_velocity_setpoints.right_motor;
    *_write_position_data_ADS = write_position_data;
}



void AdsMotorController::connectToMotor()
{
    
    // uncomment and adjust if automatic AmsNetId deduction is not working as expected
    //bhf::ads::SetLocalAddress({192, 168, 214, 103, 1, 1});
    //setting connection
    static const AmsNetId remoteNetId { 5, 97, 71, 177, 1, 1};
    static const char remoteIpV4[] = "192.168.10.20";

    static AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};
    this->route = &route; // local route is assigned to global route

    static AdsVariable<std::array<double, 4> > _read_position_data_ADS {route, "GVL_Axis.stDataWrite"};
    this->_read_position_data_ADS = &_read_position_data_ADS; // local _read_position_data_ADS is assigned to global _read_position_data_ADS

    static AdsVariable<std::array<double, 4> > _write_position_data_ADS {route, "GVL_Axis.stDataRead"};
    this->_write_position_data_ADS = &_write_position_data_ADS; //local _write_position_data_ADS is assigned to global _write_position_data_ADS
    
    std::array<bool, 2> motorstart = {true,true};
    AdsVariable<std::array<bool, 2> > _motor_init {route, "GVL_Axis.stDataReadBool"};
    //motor_init = &_motor_init;
    _motor_init = motorstart; // enable motor
}

/* //example for read data from plc 
void AdsMotorController::readByName(std::ostream& out,const AdsDevice& route,float* rData)
{

    AdsVariable<uint8_t> readVar {route, "MAIN.nCounter"};
    out << __FUNCTION__ << "():\n";
    out << "ADS read " << std::dec << (uint32_t)readVar << '\n';
    readVar = 0x7D;
    out << "ADS read " << std::dec << (uint32_t)readVar << '\n';

    float tmp = (float)readVar;
    rData = &tmp;
    //out << "rData" << std::dec << *rData << '\n';
}


void AdsMotorController::readWriteArray(std::ostream& out, const AdsDevice& route)
{

    AdsVariable<std::array<double, 4> > arrayVar {route, "GVL_Axis.stDataWrite"};
    //arrayVar = arrayToWrite;
    std::array<double, 4> readArray = arrayVar;
  
    out << "Read back array with first value " << (double)readArray[0] << " 2. " <<
        (double)readArray[1] <<"3."  <<(double)readArray[2] << " and last value " <<
        (double)readArray[3] << "\n";
}
*/
