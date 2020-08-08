
/**
@par Provides

- @ref interface_position3d

@par Requires

- @ref interface_position2d

**/

#include <libplayercore/playercore.h>
#include <iostream>
class Toscanangle:public Driver{
private:
   // player_position3d_data_t p3d;
private:
    player_devaddr_t laser_scan_addr;
    player_devaddr_t laser_scanangle_addr;
    Device *laser_scan_device;
public:
    Toscanangle(ConfigFile *cf,int section):Driver(cf,section) {
        std::cout<<"Toscanangle starting"<<std::endl;
        memset(&(this->laser_scan_addr), 0, sizeof(player_devaddr_t));
        memset(&(this->laser_scanangle_addr), 0, sizeof(player_devaddr_t));
        //memset(&(this->p3d), 0, sizeof(player_position3d_data_t));

        if (cf->ReadDeviceAddr(&this->laser_scan_addr, section, "requires",
                               PLAYER_LASER_CODE, -1, NULL) != 0)
        {
            this->SetError(-1);
            return;
        }

        if(cf->ReadDeviceAddr(&(this->laser_scanangle_addr), section, "provides",
                              PLAYER_LASER_CODE, -1, NULL) == 0)
        {
            if(this->AddInterface(this->laser_scanangle_addr))
            {
                this->SetError(-1);
                return;
            }
        }





    }
    ~Toscanangle(){
        free(this->laser_scan_device);
    }
    virtual int Setup(){
	
        if(!(this->laser_scan_device = deviceTable->GetDevice(this->laser_scan_addr)))
        {
            PLAYER_ERROR("unable to locate suitable laser device");
            return(-1);
        }
        if(this->laser_scan_device->Subscribe(this->InQueue) != 0)
        {
            PLAYER_ERROR("unable to subscribe to laser device");
            return(-1);
        }
	
    }
    virtual int Shutdown(){
        this->laser_scan_device->Unsubscribe(this->InQueue);
        return(0);

    }
    int ProcessMessage(QueuePointer &resp_queue, player_msghdr_t* hdr, void* data){
        if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                                 PLAYER_LASER_DATA_SCAN,
                                 this->laser_scan_addr)){

	    player_laser_data_scanangle_t laserDataScanangle;
            memset(&laserDataScanangle,0, sizeof(player_laser_data_scanangle_t));
            player_laser_data_t *scan= reinterpret_cast<player_laser_data_t * > (data);
           // std::cout<<"ranges count "<<scan->ranges_count<<std::endl;
            //std::cout<<"angles_count"<<(scan->max_angle-scan->min_angle)/scan->resolution<<std::endl;

            laserDataScanangle.max_range=scan->max_range;
            laserDataScanangle.intensity=scan->intensity;
            laserDataScanangle.intensity_count=scan->intensity_count;
            laserDataScanangle.ranges_count=scan->ranges_count;
            laserDataScanangle.angles_count=(scan->max_angle-scan->min_angle)/scan->resolution;
            laserDataScanangle.id=scan->id;
            laserDataScanangle.ranges=scan->ranges;
            laserDataScanangle.angles=(float*)malloc(laserDataScanangle.angles_count*sizeof(float));
            for (int i = 0; i < laserDataScanangle.angles_count; ++i) {
                laserDataScanangle.angles[i]=scan->min_angle+i*scan->resolution;
                //std::cout<<"for loop angles count is"<<laserDataScanangle.angles_count<<std::endl;

            }



            this->Publish(this->device_addr,
                          PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCANANGLE,
                          (void*)&laserDataScanangle, sizeof(laserDataScanangle), &hdr->timestamp);
//std::cout<<"published"<<std::endl;
//
        }
    }

};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver * Toscanangle_Init(ConfigFile * cf, int section)
{
  // Create and return a new instance of this driver
  return reinterpret_cast<Driver *>(new Toscanangle(cf, section));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void Toscanangle_Register(DriverTable * table)
{
  table->AddDriver("Toscanangle", Toscanangle_Init);
}
