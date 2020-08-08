#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <linux/joystick.h>
#include <libplayercore/playercore.h>

#define JS_AXIS_X 0x01
#define JS_AXIS_YAW 0x02

#define MESSAGE_DEBUG  2

class LinuxJoystick : public ThreadedDriver{
public:
    LinuxJoystick(ConfigFile *cf, int section);
    int MainSetup();
    void MainQuit();

private:
    virtual void Main();
    virtual int ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data);
    void readJoystick();
    void sendVelocity();

    private: player_devaddr_t joystick_addr;
    // Position device
    private: player_devaddr_t position_addr;
    private: player_position2d_data_t pos_data;

    private: player_devaddr_t cmd_position_addr;
    private: Device * position;

    // Joystick
    private: player_joystick_data_t joy_data;


    const char *dev;
    int fd;
    int axes[2];

    double x;
    double th;

};


int LinuxJoystick::ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data){
                               
                               return 1;
                               }
Driver* LinuxJoystick_Init(ConfigFile* cf, int section)
{
    // Create and return a new instance of this driver
    return ((Driver*) (new LinuxJoystick(cf, section)));
}

void linuxjoystick_Register(DriverTable* table)
{
    table->AddDriver("linuxjoystick", LinuxJoystick_Init);
}

LinuxJoystick::LinuxJoystick(ConfigFile *cf, int section) : ThreadedDriver(cf, section) {
    memset(&(this->cmd_position_addr), 0, sizeof(player_devaddr_t));
    memset(&(this->position_addr), 0, sizeof(player_devaddr_t));
    this->position = NULL;
    // Do we create a joystick interface?
    if (cf->ReadDeviceAddr(&(this->position_addr), section, "provides",
                           PLAYER_POSITION2D_CODE, -1, NULL) == 0) {
        if (this->AddInterface(this->position_addr)) {
            this->SetError(-1);
            return;
        }
    }
    // Do we create a joystick interface?
    if (cf->ReadDeviceAddr(&(this->joystick_addr), section, "provides",
                           PLAYER_JOYSTICK_CODE, -1, NULL) == 0) {
        if (this->AddInterface(this->joystick_addr)) {
            this->SetError(-1);
            return;
        }
    }

    this->dev = cf->ReadString(section, "port", "/dev/input/js0");

    this->axes[0] = cf->ReadTupleInt(section, "axes", 0, JS_AXIS_X);
    this->axes[1] = cf->ReadTupleInt(section, "axes", 1, JS_AXIS_YAW);

    if (cf->GetTupleCount(section, "requires")) {
        if (cf->ReadDeviceAddr(&(this->cmd_position_addr), section, "requires",
                               PLAYER_POSITION2D_CODE, -1, NULL) == 0) {
        } else memset(&(this->cmd_position_addr), 0, sizeof(player_devaddr_t));

    }
}
int LinuxJoystick::MainSetup() {
	this->fd = open(this->dev, O_RDONLY);
    if (this->fd < 1)
    {
        PLAYER_ERROR2("unable to open joystick [%s]; %s",
                      this->dev, strerror(errno));
        return -1;
    }
    this->position = NULL;
    // If we're asked, open the position2d device
    if (this->cmd_position_addr.interf)
    {
        if (!(this->position = deviceTable->GetDevice(this->cmd_position_addr)))
        {
            PLAYER_ERROR("unable to locate suitable position2d device");
            return -1;
        }
        if (this->position->Subscribe(this->InQueue) != 0)
        {
            PLAYER_ERROR("unable to subscribe to position2d device");
            return -1;
        }
    }
}

// Shutdown the device
void LinuxJoystick::MainQuit()
{
    if ((this->cmd_position_addr.interf) && (this->position))
        this->position->Unsubscribe(this->InQueue);
    close(this->fd);
}

void LinuxJoystick::readJoystick() {
    struct js_event je;
    int len;
    len = read(fd, &je, sizeof(je));
    if (len < 0) {
        printf("read failed\n");
        return;
    }
    if (je.type == JS_EVENT_AXIS) {
        switch (je.number) {
            case JS_AXIS_X:
                x = je.value;
                break;
            case JS_AXIS_YAW:
                th = je.value;
                break;
        }
    }
}

void LinuxJoystick::sendVelocity()
{
    readJoystick();
    player_position2d_cmd_vel_t cmd;
    memset(&cmd,0,sizeof(cmd));
    cmd.vel.px=-1 * (x / 32767) / 4;
    cmd.vel.pa=-1 * (th / 32767) / 4;
    //printf("%lf, %lf", cmd.vel.px, cmd.vel.pa);
    this->position->PutMsg(this->InQueue,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           (void*)&cmd, sizeof(player_position2d_cmd_vel_t),
                           NULL);

}


void LinuxJoystick::Main(){
    while (true) {
        pthread_testcancel();
        sendVelocity();

    }
}
