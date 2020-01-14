#include "ros/ros.h"
#include "Base_control/ControlCmd.h"
#include <iostream>
// #include <cstdlib>
// #include <turtlesim/Spawn.h>
#include <signal.h>
#include <termios.h>
using namespace std;

#define LINEAR_MAX 4
#define ANGULAR_MAX 4

#define speed 2.0
#define KEYCODE_SPACE 0x20
struct termios cooked, raw;
int kfd = 0;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Client1");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<Base_control::ControlCmd>("Controler");
  Base_control::ControlCmd srv;

  // construct speed
  int linear = 0;
  int angular = 0;

  char c; 
  signal(SIGINT, quit);
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Press w a s d  keys to move the Robot.");
  puts("Press the space keys to stop the Robot.");

  while (1)
  {
    bool valid = true;
    if(read(kfd, &c, 1) < 0)
    {
      perror( "read():" );
      exit( -1 );
    }

    int linear_add = 0;
    int angular_add = 0;
    switch ( c )
    {
      case 'w': { linear_add = 1; break; }
      case 's': { linear_add = -1; break; }
      case 'a': { angular_add = 1; break; }
      case 'd': { angular_add = -1;  break; }
      case KEYCODE_SPACE: { linear = 0; angular = 0; break; }
      default: { valid = false; break; };
    }
    if (valid) {
      if ((linear + linear_add) <= LINEAR_MAX && (linear + linear_add) >= -LINEAR_MAX) {
        linear += linear_add;
      }
      if ((angular + angular_add) <= ANGULAR_MAX && (angular + angular_add) >= -ANGULAR_MAX) {
        angular += angular_add;
      }
      srv.request.xx = linear * 100;
      srv.request.yy = angular * 100;
      if (client.call(srv))
      {
        ROS_INFO("Set vel=%d,omiga=%d,res=%f ",srv.request.xx,srv.request.yy,srv.response.zz);
      }
      else
      {
        ROS_ERROR("Failed to call service add_two_ints");
      }
    }

  }
  return 0;
}
