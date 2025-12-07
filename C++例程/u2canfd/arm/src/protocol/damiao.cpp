#include "protocol/damiao.h"

namespace damiao
{    
     
Limit_param limit_param[Num_Of_Motor]=
{       
        {12.566, 50, 5},   // DM3507         check 
        {12.5, 30, 10},   // DM4310          check
        {12.5, 50, 10},   // DM4310_48V
        {12.5, 10, 28},   // DM4340          check
        {12.5, 20, 28},   // DM4340_48V      check
        {12.5, 45, 12},   // DM6006          check
        {12.566, 20, 120},   // DM6248       check
        {12.5, 45, 20},   // DM8006          check
        {12.5, 45, 54},   // DM8009          check
        {12.5, 25, 200},  // DM10010L        check
        {12.5, 20, 200},  // DM10010         check
        {12.5, 280, 1},   // DMH3510         check
        {12.5, 45, 10},   // DMH6215
        {12.5, 2000, 2},    // DMS3519         check
        {12.5, 45, 10}    // DMG6220         check
};
            
Motor::Motor(DM_Motor_Type motor_type, Control_Mode ctrl_mode,uint16_t can_id, uint16_t master_id)
        :  Motor_Type(motor_type),mode(ctrl_mode),Master_id(master_id), Can_id(can_id){
    this->limit_param = damiao::limit_param[motor_type];
    this->last_time_= std::chrono::steady_clock::now();
}

void Motor::updateTimeInterval() 
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - last_time_;
    last_time_ = now;

    delta_time_ = dt.count(); // 秒为单位
}

double Motor::getTimeInterval() 
{
    return delta_time_;
}


void Motor::receive_data(float q, float dq, float tau)
{
    this->state_q = q;
    this->state_dq = dq;
    this->state_tau = tau;
}

void Motor::set_param(int key, float value)
{
    ValueType v{};
    v.value.floatValue = value;
    v.isFloat = true;
    param_map[key] = v;
}

void Motor::set_param(int key, uint32_t value)
{
    ValueType v{};
    v.value.uint32Value = value;
    v.isFloat = false;
    param_map[key] = v;
}

float Motor::get_param_as_float(int key) const
{
    auto it = param_map.find(key);
    if (it != param_map.end())
    {
        if (it->second.isFloat)
        {
            return it->second.value.floatValue;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

uint32_t Motor::get_param_as_uint32(int key) const 
{
    auto it = param_map.find(key);
    if (it != param_map.end()) {
        if (!it->second.isFloat) {
            return it->second.value.uint32Value;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

bool Motor::is_have_param(int key) const
{
    return param_map.find(key) != param_map.end();
}

/******一个can，一个Motor_Control**********************/
Motor_Control::Motor_Control(uint32_t nom_baud,uint32_t dat_baud,std::string sn,
    std::vector<DmActData> *data_ptr)
    :  data_ptr_(data_ptr)
{
    for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it) 
    {//遍历该bus下的所有电机
     std::shared_ptr<Motor> motor = std::make_shared<Motor>(it->motorType,it->mode,it->can_id, it->mst_id);
     addMotor(motor);
    }

    usb_hw = std::make_shared<usb_class>(nom_baud,dat_baud,sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    usb_hw->setFrameCallback(
        [this](can_value_type& val) {
            this->canframeCallback(val);
        });
   
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
   
    enable_all();//使能该接口下的所有电机

    std::cout<<"**********Motor_Control initialization successful**********"<<std::endl<<std::endl;
}

Motor_Control::~Motor_Control()
{   
    std::cout<<"Enter ~Motor_Control"<<std::endl;
   
    disable_all();//使能该接口下的所有电机
}

/**
 * @brief add motor to class 添加电机
 * @param DM_Motor : motor object 电机对象
 */
void Motor_Control::addMotor(std::shared_ptr<Motor> DM_Motor)
{
    motors.insert({DM_Motor->GetCanId(), DM_Motor});
    motors.insert({DM_Motor->GetMasterId(), DM_Motor});
}

void Motor_Control::enable_all()
{   
    // uint8_t data[4]={9,0,0,0};
    // for(auto& it : motors)
    // {   
    //     for(int j=0;j<5;j++)
    //     {
    //         write_motor_param(*(it.second),0x23,data);
    //         usleep(2000);
    //     }
    // }
    // for(auto& it : motors)
    // {   
    //     for(int j=0;j<5;j++)
    //     {
    //         save_motor_param(*(it.second));
    //         usleep(2000);
    //     }
    // }
    // for(auto& it : motors)
    // {   
    //     for(int j=0;j<5;j++)
    //     {
    //         set_zero_position(*(it.second));
    //         usleep(2000);
    //     }
    // }
    for(auto& it : motors)
    {   
      switchControlMode(*(it.second),VEL);
      usleep(2000);
    }
     for(auto& it : motors)
     {   
         for(int j=0;j<5;j++)
        {
         read_motor_param(*(it.second),10);
         usleep(2000);
        }
     }
     for(auto& it : motors)
     {  
        uint32_t parm=it.second->get_param_as_uint32(10);
        std::cerr<<"id: "<<it.first<<" mode is: "<<parm<<std::endl;
     }
    for(auto& it : motors)
    {   
       for(int j=0;j<5;j++)
       {
        control_cmd(it.second->GetCanId()+it.second->GetMotorMode(), 0xFC);
        usleep(2000);
       }
    }
    
}

void Motor_Control::disable_all()
{
    for(auto& it : motors)//总线有1个电机会注册两对
    {   
        for(int j=0;j<5;j++)
        {
         control_cmd(it.second->GetCanId()+it.second->GetMotorMode(), 0xFD);
         usleep(2000);
        }
    }  
}

/*
    * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
    * @param DM_Motor: motor object 电机对象
    * @param RID: register id 寄存器ID  example: damiao::UV_Value
    * @return: motor param 电机参数 如果没查询到返回的参数为0
    */
float Motor_Control::read_motor_param(Motor &DM_Motor,uint8_t RID)
{
    read_write_save=true;//发送读参数命令，返回的数据和常规返回的不一样，需要单独处理
    uint16_t id = DM_Motor.GetCanId();
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;

    std::vector<uint8_t> mydata = {id_low, id_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
    usb_hw->fdcanFrameSend(mydata, 0x7FF);
    usleep(2000);
    return 0;
}

/*
    * @description: save all param to motor flash 保存电机的所有参数到flash里面
    * @param DM_Motor: motor object 电机对象
    * 电机默认参数不会写到flash里面，需要进行写操作
    */
void Motor_Control::save_motor_param(Motor &DM_Motor)
{
    uint16_t id = DM_Motor.GetCanId();
    uint16_t mode=DM_Motor.GetMotorMode();
    control_cmd(id+mode, 0xFD);//失能
    usleep(10000);
    read_write_save=true;//发送保存参数命令，返回的数据和常规返回的不一样，需要单独处理

    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;

    std::vector<uint8_t> mydata = {id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
    usb_hw->fdcanFrameSend(mydata, 0x7FF);
    usleep(100000);
}

//读电机反馈命令
void Motor_Control::refresh_motor_status(Motor& motor)
{
    uint8_t id_low = motor.GetCanId() & 0xff; // id low 8 bit
    uint8_t id_high = (motor.GetCanId() >> 8) & 0xff; //id high 8 bit

    std::vector<uint8_t> mydata = {id_low, id_high, 0xCC, 0x00};
    usb_hw->fdcanFrameSend(mydata, 0x7FF);
}

void Motor_Control::control_cmd(uint16_t id , uint8_t cmd)
{
    std::vector<uint8_t> mydata = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    usb_hw->fdcanFrameSend(mydata, id);
}

void Motor_Control::write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
{   
    read_write_save=true;//发送写参数命令，返回的数据和常规返回的不一样，需要单独处理

    uint16_t id = DM_Motor.GetCanId();
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;

    std::vector<uint8_t> mydata = {id_low, id_high, 0x55, RID, data[0], data[1], data[2], data[3]};
    usb_hw->fdcanFrameSend(mydata, 0x7FF);
}

void Motor_Control::set_zero_position(Motor &DM_Motor)
{
    control_cmd(DM_Motor.GetCanId()+DM_Motor.GetMotorMode(), 0xFE);
}

void Motor_Control::control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
{
    // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    };
    uint16_t id = DM_Motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        std::cerr << "[Error] In control_mit,no motor with id " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }
    auto& m = motors[id];
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    Limit_param limit_param_cmd = m->get_limit_param();
    uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

    uint16_t can_id = id+MIT_MODE;
    uint8_t data[8];
    data[0] = (q_uint >> 8) & 0xff;
    data[1] = q_uint & 0xff;
    data[2] = dq_uint >> 4;
    data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    data[4] = kp_uint & 0xff;
    data[5]= kd_uint >> 4;
    data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    data[7] = tau_uint & 0xff;
    
    std::vector<uint8_t> mydata = {data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]};
    usb_hw->fdcanFrameSend(mydata, can_id);
}

void Motor_Control::control_pos_vel(Motor &DM_Motor,float pos,float vel)
{
    uint16_t id = DM_Motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        std::cerr << "[Error] In control_pos_vel,no motor with id " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }

    uint16_t can_id = id+POS_VEL_MODE;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&pos;
    vbuf=(uint8_t*)&vel;

    uint8_t data[8];
    data[0] = *pbuf;
    data[1] = *(pbuf+1);
    data[2] = *(pbuf+2);
    data[3] = *(pbuf+3);
    data[4] = *vbuf;
    data[5]=  *(vbuf+1);
    data[6] = *(vbuf+2);
    data[7] = *(vbuf+3);
  
    std::vector<uint8_t> mydata = {data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]};
    usb_hw->fdcanFrameSend(mydata, can_id);
}

void Motor_Control::control_vel(Motor &DM_Motor,float vel)
{
    uint16_t id =DM_Motor.GetCanId();
    if(motors.find(id) == motors.end())
    {
        std::cerr << "[Error] In control_vel,no motor with id " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }

    uint16_t can_id = id+VEL_MODE;

    uint8_t *vbuf;
    vbuf=(uint8_t*)&vel;
    uint8_t data[8];
    data[0] = *vbuf;
    data[1]=  *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);

    std::vector<uint8_t> mydata = {data[0], data[1], data[2], data[3]};
    usb_hw->fdcanFrameSend(mydata, can_id);
}
   

void Motor_Control::receive_param(uint8_t* data)
{
    uint16_t canID = (uint16_t(data[1]) << 8) | data[0];
    uint8_t RID = data[3];
    if (motors.find(canID) == motors.end())
    {
        std::cerr << "[Error] In receive_param,no motor with id " << canID << " is registered." << std::endl;
        //std::exit(-1);  // 终止程序，返回非 0 表示错误
        return;
    }
    if(is_in_ranges(RID))
    {
        uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
        motors[canID]->set_param(RID, data_uint32);
        if(RID==10)
        {   
            if(data_uint32==1)
            {
                motors[canID]->set_mode(MIT_MODE);
            }
            else if(data_uint32==2)
            {   
                motors[canID]->set_mode(POS_VEL_MODE);
            }
            else if(data_uint32==3)
            { 
                motors[canID]->set_mode(VEL_MODE);
            }
            else if(data_uint32==4)
            {
                motors[canID]->set_mode(POS_FORCE_MODE);
            }
        }
    }
    else
    {
        float data_float = uint8_to_float(data + 4);
        motors[canID]->set_param(RID, data_float);
    }   
}


/*
    * @description: switch control mode 切换电机控制模式
    * @param DM_Motor: motor object 电机对象
    * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
    */
bool Motor_Control::switchControlMode(Motor &DM_Motor,Control_Mode_Code mode)
{
    uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
    uint8_t RID = 10;
    write_motor_param(DM_Motor,RID,write_data);
    if (motors.find(DM_Motor.GetCanId()) == motors.end())
    {
        std::cerr << "[Error] In switchControlMode,no motor with id " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // 终止程序
        return false;
    }
    
    return true;
}

/*
    * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
    * @param DM_Motor: motor object 电机对象
    * @param RID: register id 寄存器ID
    * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
    * @return: bool true or false  是否修改成功
    */
bool Motor_Control::change_motor_param(Motor &DM_Motor,uint8_t RID,float data)
{
    if(is_in_ranges(RID)) {
        //居然传进来的是整型的范围 救一下
        uint32_t data_uint32 = float_to_uint32(data);
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data_uint32;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    else
    {
        //is float
        uint8_t *data_uint8;
        data_uint8=(uint8_t*)&data;
        write_motor_param(DM_Motor,RID,data_uint8);
    }
    if (motors.find(DM_Motor.GetCanId()) == motors.end())
    {   
        std::cerr << "[Error] In change_motor_param,no motor with id " << DM_Motor.GetCanId() << " is registered." << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
        return false;
    }
    return true;
}

/*
    * @description: change motor limit param 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
    * @param DM_Motor: motor object 电机对象
    * @param P_MAX: position max 位置最大值
    * @param Q_MAX: velocity max 速度最大值
    * @param T_MAX: torque max 扭矩最大值
    */
void Motor_Control::changeMotorLimit(Motor &DM_Motor,float P_MAX,float Q_MAX,float T_MAX)
{
    limit_param[DM_Motor.GetMotorType()]={P_MAX,Q_MAX,T_MAX};
}

void Motor_Control::canframeCallback(can_value_type& value)
{   
    static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = float(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        
        return data;
    };

    uint32_t canID =value.head.id;

    if(read_write_save==true&& motors.find(canID) != motors.end())
    {//这是发送保存参数或者写参数或者读参数返回的数据
        if(value.data[2]==0x33 || value.data[2]==0x55 || value.data[2]==0xAA)
        {//发的是读参数或写参数命令，返回对应寄存器参数
            if(value.data[2]==0x33 || value.data[2]==0x55)
            {//写参数或者读参数返回
                receive_param(&value.data[0]);  
                read_write_save=false;
            }
            read_write_save=false;
        }
    }
    else
    {//这是正常返回的位置速度力矩数据
        uint16_t q_uint = (uint16_t(value.data[1]) << 8) | value.data[2];
        uint16_t dq_uint = (uint16_t(value.data[3]) << 4) | (value.data[4] >> 4);
        uint16_t tau_uint = (uint16_t(value.data[4] & 0xf) << 8) | value.data[5];

        if(motors.find(canID) == motors.end())
        {
            return;
        }
        auto m = motors[canID];
        Limit_param limit_param_receive = m->get_limit_param();
        float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);

        float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
        float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
        m->receive_data(receive_q, receive_dq, receive_tau); 
        
        m->updateTimeInterval();
       
       double interval=m->getTimeInterval() ;
       //std::cerr<<"motor id is: "<<canID<<": "<<interval<<std::endl;
    }             
}

}
        
       

