#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <linux/joystick.h>
#include <fcntl.h>


using std::endl;

//controller constants
#define AXIS_LX 0
#define AXIS_LY 1
#define AXIS_RX 2
#define AXIS_RY 3

#define TRIG_L1 14
#define TRIG_L2 12
#define TRIG_R1 15
#define TRIG_R2 13

#define BTN_TRIANGLE 12
#define BTN_CIRCLE 13
#define BTN_X 14
#define BTN_SQUARE 15

typedef struct
{
	double axisLX;
	double axisLY;
	double axisRX;
	double axisRY;

	double trigL1;
	double trigL2;
	double trigR1;
	double trigR2;

	bool btnUp;
	bool btnDown;
	bool btnLeft;
	bool btnRight;

	bool btnTriangle;
	bool btnSquare;
	bool btnX;
	bool btnCircle;

	bool btnSelect;
	bool btnStart;

} CONTROLLER_STATE;

void print_state(CONTROLLER_STATE state)
{
	std::stringstream ss;

	ss << "axes:" << endl;
	ss << "LX: " << state.axisLX << " LY: " << state.axisLY;
	ss << " RX: " << state.axisRX << " RY: " << state.axisRY << endl;

	ss << "triggers:" << endl;
	ss << "L1: " << state.trigL1 << " L2: " << state.trigL2;
	ss << " R1: " << state.trigR1 << " R2: " << state.trigR2 << endl;

	ss << "buttons:" << endl;
	ss << "Triangle: " << state.btnTriangle << " Square: " << state.btnSquare;
	ss << " X: " << state.btnX << " Circle: " << state.btnCircle << endl;

	std::cout << ss.rdbuf() << std::endl;

//		 ROS_DEBUG("axisLX: %d", state.axisLX);
//		 ROS_DEBUG("axisLY: %d", state.axisLY);
//		 ROS_DEBUG("axisRX: %d", state.axisRX);
//		 ROS_DEBUG("axisRY: %d", state.axisRY);
}

CONTROLLER_STATE cleanup_state(CONTROLLER_STATE &instate)
{
	CONTROLLER_STATE outstate = instate;

	outstate.axisLX = (double)instate.axisLX/32767;
	outstate.axisLY = (double)instate.axisLY/32767;
	outstate.axisRX = (double)instate.axisRX/32767;
	outstate.axisRY = (double)instate.axisRY/32767;

	outstate.trigL1 = (double)(instate.trigL1+32767)/65534;
	outstate.trigL2 = (double)(instate.trigL2+32767)/65534;
	outstate.trigR1 = (double)(instate.trigR1+32767)/65534;
	outstate.trigR2 = (double)(instate.trigR2+32767)/65534;

	return outstate;
}

void update_state(const int fd, CONTROLLER_STATE &state)
{
	js_event e;

	while(read(fd, &e, sizeof(e)) > 0)
	{
		switch(e.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_BUTTON:
				switch(e.number)
				{
					case BTN_X:
						state.btnX = e.value;
						break;
					case BTN_CIRCLE:
						state.btnCircle = e.value;
						break;
					case BTN_SQUARE:
						state.btnSquare = e.value;
						break;
					case BTN_TRIANGLE:
						state.btnTriangle = e.value;
						break;

					default:
						break;
				}
				break;


			case JS_EVENT_AXIS:
				switch(e.number)
				{
					case AXIS_LX:
						state.axisLX = e.value;
						break;
					case AXIS_LY:
						state.axisLY = e.value;
						break;
					case AXIS_RX:
						state.axisRX = e.value;
						break;
					case AXIS_RY:
						state.axisRY = e.value;
						break;

					case TRIG_L1:
						state.trigL1 = e.value;
						break;
					case TRIG_L2:
						state.trigL2 = e.value;
						break;
					case TRIG_R1:
						state.trigR1 = e.value;
						break;
					case TRIG_R2:
						state.trigR2 = e.value;
						break;

				}
				break;
		}
	}
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.	The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "ps3_controller");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.	advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().	Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.	If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher pub = n.advertise<std_msgs::String>("/joystick", 1);

	ros::Rate loop_rate(10);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	CONTROLLER_STATE state = {0};
	int fd;
	fd = open("/dev/input/js1", O_RDONLY | O_NONBLOCK);
	while (ros::ok())
	{
	/**
	 * This is a message object. You stuff it with data, and then publish it.
	 */
	 /*
	std_msgs::String msg;

	std::stringstream ss;
	ss << "hello world " << count;
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());
	*/

	/**
	 * The publish() function is how you send messages. The parameter
	 * is the message object. The type of this object must agree with the type
	 * given as a template parameter to the advertise<>() call, as was done
	 * in the constructor above.
	 */
	//pub.publish(msg);
	update_state(fd, state);
	print_state(cleanup_state(state));
	ros::spinOnce();
	loop_rate.sleep();
	}


	return 0;
}
