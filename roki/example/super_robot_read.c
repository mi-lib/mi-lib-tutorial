#include <roki/roki.h>

int main(int argc, char *argv[])
{
  rkChain robot;

  rkChainReadZTK( &robot, "super_robot.ztk" );
  rkChainDestroy( &robot );
  return 0;
}
