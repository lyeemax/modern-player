#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include "rplidar.h"
#include <signal.h>
#include <time.h>
#include <libplayercore/playercore.h>
#include <hal/util.h>
#include <limits>
#include <iostream>

#define DEG2RAD(x) ((x)*M_PI/180.)
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;
class RplidarDriver:ThreadedDriver{
public:
    RplidarDriver(ConfigFile* cf,int section);
    ~RplidarDriver();
    int MainSetup();
    void MainQuit();
    virtual int ProcessMessage(QueuePointer &resp_queue,
                               player_msghdr * hdr,
                               void * data);
private:
    virtual void Main();
    const char * opt_com_path;
    _u32         baudrateArray[2] = {115200, 256000};
    u_result     op_result;
    RPlidarDriver * drv;
    bool connectSuccess= false;
    rplidar_response_device_info_t devinfo;

    float max_distance=40.0;
    bool inverted;
    bool angle_compensate;
    player_laser_data_t scan;
    player_laser_geom_t Geom;
    player_laser_config_t Conf;
    bool is_laser_boundary;

    bool checkRPLIDARHealth(RPlidarDriver * drv)
    {
        u_result     op_result;
        rplidar_response_device_health_t healthinfo;


        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("RPLidar health status : %d\n", healthinfo.status);
            if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
                fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
                // enable the following code if you want rplidar to be reboot by software
                // drv->reset();
                return false;
            } else {
                return true;
            }

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            return false;
        }
    }
    static float getAngle(const rplidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

};

RplidarDriver::RplidarDriver(ConfigFile *cf, int section):ThreadedDriver(cf,section, false,PLAYER_MSGQUEUE_DEFAULT_MAXLEN,PLAYER_LASER_CODE) {
    //init vars
    memset(&scan, 0, sizeof(scan));
    memset(&Geom, 0, sizeof(Geom));
    memset(&Conf, 0, sizeof(Conf));
    Geom.size.sw = (0.050);
    Geom.size.sl = (0.050);
    Geom.pose.px = (cf->ReadTupleLength (section, "pose", 0, 0));
    Geom.pose.py = (cf->ReadTupleLength (section, "pose", 1, 0));
    Geom.pose.pyaw = (cf->ReadTupleAngle  (section, "pose", 2, 0));
    Conf.min_angle = cf->ReadAngle(section,"min_angle",DTOR(0));
    Conf.max_angle = cf->ReadAngle(section,"max_angle",DTOR(359.99));
    is_laser_boundary=cf->ReadBool(section,"is_laser_boundary",false);
    Conf.resolution = DTOR(1.f);
    Conf.max_range = 40;
    Conf.range_res = 1;
    Conf.intensity = 254;
    angle_compensate=true;
    opt_com_path = cf->ReadString(section, "port", "/dev/ttyUSB0");
    scan.intensity=new uint8_t[360];
    scan.ranges=new float[360];
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    return;
}

RplidarDriver::~RplidarDriver() {
    delete drv;
    delete[] scan.ranges;
    delete[] scan.intensity;
}
int RplidarDriver::MainSetup() {
    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for(size_t i = 0; i < baudRateArraySize; ++i)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
                break;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
    }
    return 0;
}

void RplidarDriver::MainQuit() {
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
}

int RplidarDriver::ProcessMessage(QueuePointer &resp_queue, player_msghdr *hdr, void *data) {
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_GEOM);
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_GET_CONFIG);
    HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_LASER_REQ_SET_CONFIG);

    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                             PLAYER_LASER_REQ_GET_GEOM,
                             this->device_addr))
    {
        Publish(device_addr,resp_queue, PLAYER_MSGTYPE_RESP_ACK,PLAYER_LASER_REQ_GET_GEOM,&Geom,sizeof(Geom),NULL);
    }
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                  PLAYER_LASER_REQ_GET_CONFIG,
                                  this->device_addr))
    {
        Publish(device_addr,resp_queue, PLAYER_MSGTYPE_RESP_ACK,PLAYER_LASER_REQ_GET_CONFIG,&Conf,sizeof(Conf),NULL);
    }
    else
    {
        return -1;
    }
    return 0;
}

void RplidarDriver::Main() {
    if (!checkRPLIDARHealth(drv)) {
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
        exit(-1);
    }
    drv->startMotor();
    drv->startScan(0, 1);

    for (int veces = 0;; veces++) {
       // pthread_testcancel();
        ProcessMessages();
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);
        op_result = drv->grabScanDataHq(nodes, count);
        if (op_result == RESULT_OK) {
            drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));
                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {
                                angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                            }
                        }
                    }
                    size_t node_count=angle_compensate_nodes_count;

                    //
                    static int scan_count = 0;
                    scan_count++;
                    //memset(&scan, 0, sizeof(scan));
                    scan.id=scan_count;
                    bool reversed = (angle_max > angle_min);
                    if ( reversed ) {
                        scan.min_angle =  M_PI - angle_max;
                        scan.max_angle =  M_PI - angle_min;
                    } else {
                        scan.min_angle =  M_PI - angle_min;
                        scan.max_angle =  M_PI - angle_max;
                    }
                    scan.max_range=max_distance;
                    scan.intensity_count=node_count;
			scan.resolution=DTOR(1.0f);
                    scan.ranges_count=node_count;
                    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
                    if (!reverse_data) {
                        for (size_t i = 0; i < node_count; i++) {
                            float read_value = (float) angle_compensate_nodes[i].dist_mm_q2/4.0f/1000;
                            
                                //scan.ranges[i] = std::numeric_limits<float>::infinity();
                            if(is_laser_boundary) read_value=read_value<=0.1?40.0:read_value;
                                scan.ranges[i] = read_value;
                            scan.intensity[i] = (size_t) (angle_compensate_nodes[i].quality >> 2);
                        }
                    } else {
                        for (size_t i = 0; i < node_count; i++) {
                            float read_value = (float)angle_compensate_nodes[i].dist_mm_q2/4.0f/1000;
                            
                                //scan.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
                                if(is_laser_boundary) read_value=read_value<=0.1?40.0:read_value;
                            
                                scan.ranges[node_count-1-i] = read_value;
                            scan.intensity[node_count-1-i] = (size_t) (angle_compensate_nodes[i].quality >> 2);
                        }
                    }
                    Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_LASER_DATA_SCAN,
                            &scan, sizeof(player_laser_data_t), NULL);
                }
            }

        }

    }
}


Driver* RplidarDriver_Init(ConfigFile* cf, int section) {
    // Create and return a new instance of this driver
    return((Driver*)(new RplidarDriver(cf, section)));
}

void Rplidar_Register(DriverTable* table) {
    table->AddDriver("Rplidar", RplidarDriver_Init);
}
