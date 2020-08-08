/**
@par Provides

- @ref interface_position3d

@par Requires

- @ref interface_position2d

**/

#include <libplayercore/playercore.h>
#include <iostream>
#include <chrono>
class To3d:public Driver{
private:
    player_position2d_data_t g_p2d;
    std::chrono::steady_clock::time_point last_time;
private:
    player_devaddr_t p2d_addr;
    player_devaddr_t p3d_addr;
    Device *p2d_device;
public:
    To3d(ConfigFile *cf,int section):Driver(cf,section) {
		std::cout<<"TO3D starting"<<std::endl;
        memset(&(this->p2d_addr), 0, sizeof(player_devaddr_t));
        
        memset(&(this->p3d_addr), 0, sizeof(player_devaddr_t));
        memset(&(this->g_p2d), 0, sizeof(player_position2d_data_t));
        if (cf->ReadDeviceAddr(&this->p2d_addr, section, "requires",
                               PLAYER_POSITION2D_CODE, -1, NULL) != 0)
        {
            this->SetError(-1);
            return;
        }
        

        if(cf->ReadDeviceAddr(&(this->p3d_addr), section, "provides",
                              PLAYER_POSITION3D_CODE, -1, NULL) == 0)
        {
            if(this->AddInterface(this->p3d_addr))
            {
                this->SetError(-1);
                return;
            }
        }
       
        last_time= std::chrono::steady_clock::now();
     




    }
    ~To3d(){
        free(this->p2d_device);
    }
    virtual int Setup(){
	
        if(!(this->p2d_device = deviceTable->GetDevice(this->p2d_addr)))
        {
            PLAYER_ERROR("unable to locate suitable position device");
            return(-1);
        }
        if(this->p2d_device->Subscribe(this->InQueue) != 0)
        {
            PLAYER_ERROR("unable to subscribe to position device");
            return(-1);
        }
	
    }
    virtual int Shutdown(){
        this->p2d_device->Unsubscribe(this->InQueue);
        return(0);

    }
    int ProcessMessage(QueuePointer &resp_queue, player_msghdr_t* hdr, void* data){
        if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                                 PLAYER_POSITION2D_DATA_STATE,
                                 this->p2d_addr)){
	    //std::cout<<"TO3D started"<<std::endl;
            player_position3d_data_t p3d;
            memset(&p3d,0, sizeof(player_position3d_data_t));

            player_position2d_data_t *p2= reinterpret_cast<player_position2d_data_t * > (data);
            std::chrono::steady_clock::time_point current_time= std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_time).count();
           // std::cout<<"time is "<<dt<<std::endl;
            //Robot Pose(x,y,theta)
            p3d.pos.px=p2->pos.px;
            p3d.pos.py=p2->pos.py;
            p3d.pos.pz=p2->pos.pa;

            //gyro value velocity (roll,pitch,yaw)
            p3d.pos.proll=p2->vel.pa;
            p3d.pos.ppitch=0;
            p3d.pos.pyaw=0;

            //Acc value(x,y,z)
            p3d.vel.px=(p2->vel.px-g_p2d.vel.px)/dt;
            p3d.vel.py=(p2->vel.py-g_p2d.vel.py)/dt;
            p3d.vel.pz=9.7;

            //odo value(left,right,v) 这一部分好像没有用到 需要验证
            p3d.vel.proll=0;
            p3d.vel.ppitch=0;
            p3d.vel.pyaw=0;

//std::cout<<"imu lv and acc is "<<p3d.pos.proll<<","<<p3d.pos.ppitch<<","<<p3d.pos.pyaw<<"--"<<p3d.vel.px<<","<<p3d.vel.py<<","<<p3d.vel.pz<<std::endl;
            this->Publish(this->device_addr,
                          PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
                          (void*)&p3d, sizeof(p3d), &hdr->timestamp);
            g_p2d=*p2;
            last_time=current_time;
        }
    }

};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver * To3d_Init(ConfigFile * cf, int section)
{
  // Create and return a new instance of this driver
  return reinterpret_cast<Driver *>(new To3d(cf, section));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void To3d_Register(DriverTable * table)
{
  table->AddDriver("To3d", To3d_Init);
}

