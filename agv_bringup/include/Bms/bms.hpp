#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/scoped_ptr.hpp>
#include <modbus/modbus.h>
#include <agv_bringup/battery.h>
#include <iostream>

namespace bms
{

   enum ModbusRegisters {
        TotalVoltage = 103, 
        Current = 104,
        BatteryPercentage = 107,
        BatteryState = 114,
        /*Cell1_V = 131,   these two cells are not
        Cell2_V,          included in our battey */
        Cell3_V =133 ,
        Cell4_V ,
        Cell5_V ,
        Cell6_V ,
        Cell7_V ,
        Cell8_V ,
        Cell9_V ,
        Cell10_V ,
        Cell11_V ,
        Cell12_V ,
        Cell13_V ,
        Cell14_V ,
        Cell15_V ,
        Cell16_V  };

    enum BatteryStates{
        Discharging,
        Charging,
        Idle,
        Fault 
    };

  struct BmsReader
  {
      BmsReader (std::string name,std::uint8_t id,std::uint8_t length, std::uint16_t *addr);
      ~BmsReader ();
      void readData(modbus_t *ctx);
      
      std::string name;
      std::uint8_t id;
      std::uint8_t length;
      void *data;
  };

  /*!
  * Class containing the Battery Management 
  */
  class Bms {
      public:
        Bms(ros::NodeHandle& nh, const char *port);
        virtual ~Bms();
        modbus_t *ctx;
    
        void update();
        void connect(const char *port);

      private:
        ros::NodeHandle nodeHandle_;
        ros::Publisher battery_pub;
      
        agv_bringup::battery battery_msg;
        std::list<BmsReader> bmsList;
  };

} 




 