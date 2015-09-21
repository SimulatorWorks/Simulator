#include "JoystickController.h"

JoystickController::JoystickController()
{
    joystick_fd=-1;

	joystick_fd=this->open_joystick();
}

JoystickController::~JoystickController(){
	this->close_joystick();
}

struct js_event JoystickController::getJoystickEvent(){
	return this->joystick_event;
}

int JoystickController::getJoystickFd(){
    return this->joystick_fd;
}

void JoystickController::getCurrentControl(double& steeringAngle_rad, double& acceleration_mpss)
{
    steeringAngle_rad = - 30 * (double)steeringValue_ / 32767 * DEG2RAD;

    if(throttleValue_ <= 0) { acceleration_mpss = - 0.5*G * (double)throttleValue_ / 32767; }
    if(throttleValue_ > 0) { acceleration_mpss = - 1.0*G * (double)throttleValue_ / 32767; }
}

int JoystickController::open_joystick()
{
    joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
//	if (joystick_fd < 0)
//        joystick_fd = open(JOYSTICK_DEVNAME2, O_RDONLY | O_NONBLOCK); /* read write for force feedback? */

	return joystick_fd;
}

void JoystickController::close_joystick()
{
	close(joystick_fd);
}


int JoystickController::read_joystick_event()
{
	int bytes;

	bytes = read(joystick_fd, &joystick_event, sizeof(joystick_event));

	if (bytes == -1)
		return 0;

	if (bytes == sizeof(joystick_event))
		return 1;

//	printf("Unexpected bytes from joystick:%d\n", bytes);

	return -1;
}

void JoystickController::updateControls()
{
	int rc;
	//Read Joystick Event in this cycle
	rc = this->read_joystick_event();
    if (rc == 1)
    {
		//Successfully Read Joystick Event in this cycle
        struct js_event jse = this->getJoystickEvent();

//        qDebug("%d %d %d %d", jse.time, jse.type, jse.value, jse.number);

        if((int)jse.type == 2 && (int)jse.number == 0) { steeringValue_ = jse.value; }
        if((int)jse.type == 2 && (int)jse.number == 1) { throttleValue_ = jse.value; }

	}
    else
    {
		//No Joystick Event in this cycle, check hold booleans

	}

//    qDebug("steeringValue_ = %d \t throttleValue_ = %d", steeringValue_, throttleValue_);
}
