#include <Bms/bms.hpp>

  bms::BmsReader::BmsReader(std::string name,std::uint8_t id,std::uint8_t length,std::uint16_t *addr)
  :name(name),id(id),length(length),data(addr){}
  bms::BmsReader::~BmsReader(){}

  void bms::BmsReader::readData(modbus_t *ctx)
  {
    if (modbus_read_registers(ctx, id, length, (std::uint16_t *)data) == -1)
    {
      fprintf(stderr, "Connection failed in %s : %s\n", name.c_str(), modbus_strerror(errno));
    }
    else
    {
      switch (id)
      {
      case TotalVoltage:
        if(*(uint16_t *)data < 448)
          ROS_WARN_STREAM( "Low voltage " << *(uint16_t *)data );
        break;
      case BatteryPercentage: 
        if(*(uint16_t *)data < 30)
          ROS_WARN_STREAM( "Low battery " << *(uint16_t *)data );
        break;
      case Current:
        if(*(int16_t *)data < -200)
          ROS_WARN_STREAM( "!!! High Current !!! " <<std::dec << (float)(*(int16_t *)data/10.0) << "A" );
        break;
      default:
        break;
      }
    }
  }

  bms::Bms::Bms(ros::NodeHandle& nh, const char *port)
  {  
    nodeHandle_ = nh; 

    //Initialize Modbus connection
    connect(port);
    //Battery type msg created.
    battery_msg = agv_bringup::battery(); 
    //Publisher created.
    battery_pub = nodeHandle_.advertise<agv_bringup::battery>("battery", 100 , this);
    
    bmsList.push_back(BmsReader("Voltage",bms::TotalVoltage,1, &battery_msg.voltage)); // If It reads 589, means 58.9 V
    bmsList.push_back(BmsReader("Current",bms::Current,1, (std::uint16_t *)&battery_msg.current)); // If It reads 300, means 30 A (- decharging, + charging)
    bmsList.push_back(BmsReader("BatteryPercentage",bms::BatteryPercentage,1,&battery_msg.percentage));
    bmsList.push_back(BmsReader("BatteryState",bms::BatteryState,1,&battery_msg.state));
    bmsList.push_back(BmsReader("BatteryCellState",bms::Cell3_V, 14, (std::uint16_t *)&(battery_msg.cells_V)));   // Reads 14 cells' Voltage, ReadData must be 16 element array , In terms of mV(milliVolts)
  }

  // Needs Usb-port name as parameter
  // This Connection settings are specific to Solion Minis 416
  void bms::Bms::connect(const char *port){
    
    ctx = modbus_new_rtu(port, 19200, 'N', 8, 1);
    if (ctx == NULL) {
    fprintf(stderr, "Unable to create the libmodbus context\n");
    }
    modbus_set_slave(ctx, 0x01);
     
    if (modbus_connect(ctx) == -1) {
      fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
      modbus_free(ctx);
    }
    else{
      std::cout << "Connection established." << std::endl;
    } 
  }

  // Updates battery message 
  void bms::Bms::update(){

    for(std::list<BmsReader>::iterator it = bmsList.begin(); it != bmsList.end(); ++it)
    {
      it->readData(ctx);
     // std::cout << it->name << " : " << *(uint16_t *)it->data << std::endl;
    }
    battery_pub.publish(battery_msg);
  }
  
  bms::Bms::~Bms()
  {
      modbus_close(ctx);
      modbus_free(ctx);
  }

 