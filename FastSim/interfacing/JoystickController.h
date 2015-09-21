#ifndef __JOYSTICKCONTROLLER_H__
#define __JOYSTICKCONTROLLER_H__

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <QtCore>

#include <res/Comm.h>

#define JOYSTICK_DEVNAME "/dev/input/js1"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */


struct js_event
{
	unsigned int time;	/* event timestamp in milliseconds */
	short value;   /* value */
	unsigned char type;     /* event type */
	unsigned char number;   /* axis/button number */
};

class JoystickController
{
public:
	JoystickController();
	~JoystickController();

	int open_joystick();
	int read_joystick_event();
	void close_joystick();

	struct js_event getJoystickEvent();
	int getJoystickFd();

	void updateControls();

    void getCurrentControl(double& steeringAngle_rad, double& acceleration_mpss);

private:
	int joystick_fd;
	struct js_event joystick_event;

    int steeringValue_;
    int throttleValue_;

};

#endif
