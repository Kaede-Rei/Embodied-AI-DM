#include "protocol/damiao.h"
#include <csignal>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;
  using duration = std::chrono::duration<double>;

  std::signal(SIGINT, signalHandler);

  try 
  {
      uint16_t canid1 = 0x01;
      uint16_t mstid1 = 0x11;
      uint16_t canid2 = 0x02;
      uint16_t mstid2 = 0x12;
      uint16_t canid3 = 0x03;
      uint16_t mstid3 = 0x13;
      uint16_t canid4 = 0x04;
      uint16_t mstid4 = 0x14;
      uint16_t canid5 = 0x05;
      uint16_t mstid5 = 0x15;
      uint16_t canid6 = 0x06;
      uint16_t mstid6 = 0x16;
      uint16_t canid7 = 0x07;
      uint16_t mstid7 = 0x17;
      uint16_t canid8 = 0x08;
      uint16_t mstid8 = 0x18;
      uint16_t canid9 = 0x09;
      uint16_t mstid9 = 0x19;
      uint16_t canid10 = 0x0A;
      uint16_t mstid10 = 0x21;
      uint16_t canid11 = 0x0B;
      uint16_t mstid11 = 0x22;
      uint16_t canid12 = 0x0C;
      uint16_t mstid12 = 0x23;
      uint16_t canid13 = 0x0D;
      uint16_t mstid13 = 0x24;
      uint16_t canid14 = 0x0E;
      uint16_t mstid14 = 0x25;
      
      uint32_t nom_baud =1000000;
      uint32_t dat_baud =5000000;

      std::vector<damiao::DmActData> init_data;
      std::vector<damiao::DmActData> init_data2;
    
      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid1,
        .mst_id=mstid1 });

      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
      // .mode = damiao::POS_VEL_MODE,
      // .can_id=canid2,
      // .mst_id=mstid2 });

      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
      // .mode = damiao::POS_VEL_MODE,
      // .can_id=canid3,
      // .mst_id=mstid3 });

      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
      // .mode = damiao::POS_VEL_MODE,
      // .can_id=canid4,
      // .mst_id=mstid4 });

      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
      // .mode = damiao::POS_VEL_MODE,
      // .can_id=canid5,
      // .mst_id=mstid5 });

      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
      // .mode = damiao::POS_VEL_MODE,
      // .can_id=canid6,
      // .mst_id=mstid6 });
      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM8009,
      //   .mode = damiao::POS_VEL_MODE,
      //   .can_id=canid7,
      //   .mst_id=mstid7 });

      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM8009,
      //   .mode = damiao::POS_VEL_MODE,
      //   .can_id=canid8,
      //   .mst_id=mstid8 });
      
      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM10010L,
      //   .mode = damiao::POS_VEL_MODE,
      //   .can_id=canid9,
      //   .mst_id=mstid9 });
      
      // init_data.push_back(damiao::DmActData{.motorType = damiao::DM10010,
      //   .mode = damiao::POS_VEL_MODE,
      //   .can_id=canid10,
      //   .mst_id=mstid10 });
      
      // init_data.push_back(damiao::DmActData{.motorType = damiao::DMG6220,
      //   .mode = damiao::POS_VEL_MODE,
      //   .can_id=canid13,
      //   .mst_id=mstid13 });
      /*init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid7,
        .mst_id=mstid7 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid8,
        .mst_id=mstid8 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid9,
        .mst_id=mstid9 });*/

      //init_data2.push_back(damiao::DmActData{.motorType = damiao::DM3507,
      //  .mode = damiao::MIT_MODE,
      //  .can_id=canid9,
     //   .mst_id=mstid9 });
    //  init_data2.push_back(damiao::DmActData{.motorType = damiao::DM4310,
    //    .mode = damiao::MIT_MODE,
    //    .can_id=canid7,
    //    .mst_id=mstid7 });
     std::shared_ptr<damiao::Motor_Control> control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
      "14AA044B241402B10DDBDAFE448040BB",&init_data);

     //std::shared_ptr<damiao::Motor_Control> control2 = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
     //   "14AA044B241402B10DDBDAFE448040BB",&init_data2);
     auto start = std::chrono::steady_clock::now();
      while (running) 
      { 
        const duration desired_duration(0.001); // 计算期望周期
        auto current_time = clock::now();
        //control->set_zero_position(*control->getMotor(canid4));
       // control->set_zero_position(*control->getMotor(canid5));
      //  control->control_mit(*control->getMotor(canid1), 0.0, 0.0, 0.0, 0.0, 0.0);    
      //  control->control_mit(*control->getMotor(canid2), 0.0, 0.0, 0.0, 0.0, 0.0);
      //  control->control_mit(*control->getMotor(canid3), 0.0, 0.0, 0.0, 0.0, 0.0);
      //  control->control_mit(*control->getMotor(canid4), 0.0, 0.0, 0.0, 0.0, 0.0);
      //  control->control_mit(*control->getMotor(canid5), 0.0, 0.0, 0.0, 0.0, 0.0);
      //  control->control_mit(*control->getMotor(canid6), 0.0, 0.0, 0.0, 0.0, 0.0);
      // control2->control_mit(*control2->getMotor(canid9), 0.0, 0.5, 0.0, 0.5, 0.0);
       //control2->control_mit(*control2->getMotor(canid7), 0.0, 0.5, 0.0, 5.0, 0.0);
       control->control_vel(*control->getMotor(canid1),3.0);
       for(uint16_t id = 1;id<=1;id++)
       {
         float pos=control->getMotor(id)->Get_Position();
         float vel=control->getMotor(id)->Get_Velocity();
         float tau=control->getMotor(id)->Get_tau();
         double interval=control->getMotor(id)->getTimeInterval() ;
         std::cerr<<"canid is: "<<id<<" pos: "<<pos<<" vel: "<<vel
                 <<" effort: "<<tau<<" time(s): "<<interval<<std::endl;
       }


        const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
        std::this_thread::sleep_until(sleep_till);    
      }

      std::cout <<  std::endl<<"The program exited safely." << std::endl<< std::endl;
  }
  catch (const std::exception& e) {
      std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
      return 1;
  }

  return 0;
}
