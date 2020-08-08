//
// Created by raysuner on 19-7-10.
//



#include "RobotBase.h"

///////////////////////////////////////////////////////////////////////////
// initialization function
Driver* RobotBase_Init( ConfigFile* cf, int section) {
    return((Driver*)(new RobotBase(cf, section)));
}

///////////////////////////////////////////////////////////////////////////
// a driver registration function
void RobotBase_Register(DriverTable* table) {
    table->AddDriver("RobotBase",  RobotBase_Init);
}


RobotBase::RobotBase(ConfigFile *cf, int section) : ThreadedDriver(cf, section){
 
    memset (&this->position_addr_, 0, sizeof (player_devaddr_t));
    memset(&this->position_data_,0, sizeof(position_data_));
    // Check the config file to see if we are providing a position2d interface.
    if (cf->ReadDeviceAddr(&(this->position_addr_), section, "provides",
                           PLAYER_POSITION2D_CODE, -1, nullptr) == 0) {
        PLAYER_MSG0(MESSAGE_INFO, "Adding position2d interface.");
        if (this->AddInterface(this->position_addr_) != 0) {
            PLAYER_ERROR("Error adding position2d interface.");
            SetError(-1);
            return;
        }
    }

    if(!(this->port_ = (char*)cf->ReadString(section, "port", nullptr))){
        PLAYER_ERROR("RobotBaseEQ: you must specify the serial port device.");
        this->SetError(-1);
        return;
    }

    this->addimu_ = cf->ReadBool(section, "addimu", true);
    this->rate_ = cf->ReadFloat(section, "rate", (100.0/2700.0)*3.141592654*0.125);
    this->recrate_ = cf->ReadFloat(section, "recrate", (100.f/2700.f)*3.141592654*0.125*1.04);


    portinfo_.baudrate = 15200;
    portinfo_.databit = '8';
    portinfo_.debug = '0';
    portinfo_.echo = '0';
    portinfo_.fctl = '2';
    portinfo_.parity = '0';
    portinfo_.prompt = '0';
    portinfo_.port = port_;
    portinfo_.reserved = 0;

    position_data_.pos.px = 0;
    position_data_.pos.pa = 0;
    last_imu_v_=last_vth_=0;

    current_time_ = std::chrono::steady_clock::now();
    last_time_ = current_time_;
    clock_sum=0;
}

int RobotBase::MainSetup() {
    init();
}

void RobotBase::MainQuit(){
    WriteMotorVelocity(0.0, 0.0);
    close(fd_);
}

void RobotBase::WriteMotorVelocity(double vx, double vtheta) {
    unsigned char rrot;
    unsigned char lrot;
    unsigned char rv;
    unsigned char lv;
    double vright = (2.f * vx + vtheta * 0.33) / (2.f*rate_);
    double vleft = (2 * vx - vtheta * 0.33) / (2.f*rate_);
    rrot = vright > 0 ? 0 : 1;
    lrot = vleft > 0 ? 0 : 1;
    rv = round(fabs(vright));
    lv = round(fabs(vleft));



     assert(fabs(rv-lv)<20 && rv<40 &&lv<40);
    unsigned  char data[8] = {0xff, 0xfe, lv, rv, lrot, rrot, 0x00, 0x00};
    //for debug
   // std::cout<<std::endl;
    //std::cout<<"sent vel is ("<<int(rv)<<","<<int(lv)<<")"<<std::endl;
    //std::cout<<std::endl;
    int SendLen = sendPort(data, 10);
    if(SendLen < 0){
        printf("Error: send failed.\n");
    }
}


void RobotBase::ProcessOdometry(double dt){

    int recv_len = recvPort(recvbuf_, 8);
   // current_time_ = std::chrono::steady_clock::now();
    //if(recv_len > 0)
    {
        double rd = recvbuf_[3] == 0x00 ? -1 : 1;  //左轮方向
        double ld = recvbuf_[5] == 0x00 ? -1 : 1;  //右轮方向
        double vx = ((rd * double(recvbuf_[2]) + ld * double(recvbuf_[4])) / 2.f) * recrate_;
        double vth = ((-rd * double(recvbuf_[2]) + ld * double(recvbuf_[4])) / 0.3225) * recrate_;

        double imu_vth = double(recvbuf_[6] * 256 + recvbuf_[7]-32768)/16.4f/ 180.f * 3.1415;  // 来源于imu
       // std::cout<<"vx  "<<vx<<"vth  "<<vth<<std::endl;
        //double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - last_time_).count();
        if(fabs(vth)>0.9) vth=0;
        if(fabs(vth-last_vth_)>1.7) vth=(vth+last_vth_)/2.f;
        if(fabs(imu_vth)>0.9) imu_vth=0;
        if(fabs(imu_vth-last_imu_v_)>1.7) imu_vth=(imu_vth+last_imu_v_)/2.0f;
        //for debug
        //pure_imu+=imu_vth*180.0f/3.1415926*dt;
        //pure_odo+=vth*180.0f/3.1415926*dt;
        //std::cout << "whole_imu is: " <<pure_imu<<"whole odom is"<<pure_odo<< std::endl;



            position_data_.vel.px = vx;
            position_data_.vel.py = 0;
            position_data_.vel.pa = vth;
            fusion.update(position_data_, imu_vth, dt);
            //std::cout<<"position_data_.pos.px "<<position_data_.pos.px<<std::endl;
        last_imu_v_=imu_vth;
        last_vth_=vth;
        //clock_sum+=dt;
       // std::cout<<std::endl;
       // std::cout << "pose is:(" << position_data_.pos.px << " ," << position_data_.pos.py << " , " << position_data_.pos.pa/3.1415926*180.0 <<" )"<< std::endl;
       // std::cout<<std::endl;
        this->Publish(position_addr_, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
                      (void*)&position_data_,sizeof(position_data_),NULL);
    }
}


int RobotBase::ProcessMessage(QueuePointer &resp_queue, player_msghdr_t *hdr, void *data) {
    PLAYER_MSG3(MESSAGE_DEBUG,"Received Message for addr.index %d type %d subtype %d",hdr->addr.index, hdr->type, hdr->subtype);
     HANDLE_CAPABILITY_REQUEST (position_addr_, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
    HANDLE_CAPABILITY_REQUEST(position_addr_, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL);
  HANDLE_CAPABILITY_REQUEST (position_addr_, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_REQ_SET_ODOM);
  HANDLE_CAPABILITY_REQUEST (position_addr_, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_REQ_RESET_ODOM);
  HANDLE_CAPABILITY_REQUEST (position_addr_, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_REQ_GET_GEOM);
   HANDLE_CAPABILITY_REQUEST (position_addr_, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_REQ_MOTOR_POWER);
    //////////////////////////////////////////////////////////////////////////////
    // Process position2d messages
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL, position_addr_)) {

        assert(hdr->size == sizeof(player_position2d_cmd_vel_t));

        player_position2d_cmd_vel_t & command
                = *reinterpret_cast<player_position2d_cmd_vel_t *> (data);

        WriteMotorVelocity(command.vel.px, command.vel.pa);
        return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION2D_REQ_MOTOR_POWER,
        this->position_addr_))
  {
    this->Publish(this->position_addr_, resp_queue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
    return 0;
  }
  else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ,
                                PLAYER_POSITION2D_REQ_GET_GEOM,
                                this->position_addr_))
  {
    /* Return the robot geometry. */
    memset(&pos_geom, 0, sizeof pos_geom);
    // Assume that it turns about its geometric center, so zeros are fine

    pos_geom.size.sl = 0.28;
    pos_geom.size.sw = 0.28;

    this->Publish(this->position_addr_, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_POSITION2D_REQ_GET_GEOM,
                  reinterpret_cast<void *>(&pos_geom), sizeof pos_geom, NULL);
    return 0;
  }

    else if (Message::MatchMessage(hdr , PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_SET_ODOM, position_addr_)) {
        assert(hdr->size == sizeof(player_position2d_set_odom_req_t));

        player_position2d_set_odom_req_t & odom_req = * reinterpret_cast<player_position2d_set_odom_req_t *> (data);

        current_position_.px = odom_req.pose.px;
        current_position_.py = odom_req.pose.py;
        current_position_.pa = odom_req.pose.pa;

        this->Publish(position_addr_, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);
        return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_RESET_ODOM, position_addr_)) {
        current_position_.px = 0.0;
        current_position_.py = 0.0;
        current_position_.pa = 0.0;

        this->Publish(position_addr_, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_RESET_ODOM);
        return 0;
    }
    return 0;

}

void RobotBase::Main(){
    while (1){
        //pthread_testcancel();
        //WriteMotorVelocity(0.3, 0.0);  //测试下
//        if((std::chrono::steady_clock::now() - start).count() >= 10){
//            WriteMotorVelocity(0.0, 0.0);
//        }
        ProcessMessages();
        current_time_ = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - last_time_).count();
        ProcessOdometry(dt);
       // std::cout<<"dt is     "<<dt<<std::endl;
        last_time_=std::chrono::steady_clock::now();
        //
       // std::cout<<"clock sum is     "<<clock_sum<<std::endl;
       // this->Publish(position_addr_, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,
        //              (unsigned char *) &position_data_, sizeof(position_data_), nullptr);
        usleep(80000);
    }

}



