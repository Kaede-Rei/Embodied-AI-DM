#include "protocol/damiao.h"
#include <csignal>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

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
std::shared_ptr<damiao::Motor_Control> control;
void threadFunction(int id) {

  using clock = std::chrono::steady_clock;
  using duration = std::chrono::duration<double>;
  while (running) {
    const duration desired_duration(0.001); // 计算期望周期
    auto current_time = clock::now();

    control->control_mit(*control->getMotor(canid4), 0.0, 0.0, 0.0, 0.0, 0.0);
    control->control_mit(*control->getMotor(canid5), 0.0, 0.0, 0.0, 0.0, 0.0);
    control->control_mit(*control->getMotor(canid6), 0.0, 0.0, 0.0, 0.0, 0.0);

    const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
    std::this_thread::sleep_until(sleep_till);
  }

}

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;
  using duration = std::chrono::duration<double>;

  uint32_t nom_baud =1000000;
  uint32_t dat_baud =5000000;

  std::vector<damiao::DmActData> init_data;

    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
                                            .mode = damiao::MIT_MODE,
                                            .can_id=canid1,
                                            .mst_id=mstid1 });

    // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
    //   .mode = damiao::MIT_MODE,
    //   .can_id=canid2,
    //   .mst_id=mstid2 });
    //
    // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
    //   .mode = damiao::MIT_MODE,
    //   .can_id=canid3,
    //   .mst_id=mstid3 });
    //
    // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
    //   .mode = damiao::MIT_MODE,
    //   .can_id=canid4,
    //   .mst_id=mstid4 });
    //
    // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
    //  .mode = damiao::MIT_MODE,
    //   .can_id=canid5,
    //   .mst_id=mstid5 });
    //
    // init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
    //   .mode = damiao::MIT_MODE,
    //   .can_id=canid6,
    //  .mst_id=mstid6 });
  control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
          "14AA044B241402B10DDBDAFE448040BB",&init_data);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  //std::thread t1(threadFunction, 1);
  std::signal(SIGINT, signalHandler);

  try
  {
      while (running)
      {
        const duration desired_duration(0.001); // 计算期望周期
        auto current_time = clock::now();

        control->control_mit(*control->getMotor(canid1), 0.0, 0.0, 0.0, 0.0, 0.9);
        // control->control_mit(*control->getMotor(canid2), 0.0, 0.0, 0.0, 0.0, 0.0);
        // control->control_mit(*control->getMotor(canid3), 0.0, 0.0, 0.0, 0.0, 0.0);

        for(uint16_t id = 1;id<=1;id++)
       {
        float pos=control->getMotor(id)->Get_Position();
        float vel=control->getMotor(id)->Get_Velocity();
        float tau=control->getMotor(id)->Get_tau();
        float interval=control->getMotor(id)->getTimeInterval();

         std::cerr<<"id is: "<<id<<" pos: "<<pos<<
           " vel: "<<vel<<" effort: "<<tau<<" time_interval(s): "<<interval<<std::endl;
          //std::cerr<<"id is: "<<id<<" time_interval(s): "<<interval<<std::endl;
       }

        //std::this_thread::sleep_for(std::chrono::milliseconds(1));

        const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
        std::this_thread::sleep_until(sleep_till);
      }

      std::cout << "The program exited safely." << std::endl;
  }
  catch (const std::exception& e) {
      std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
      return 1;
  }

  return 0;
}
