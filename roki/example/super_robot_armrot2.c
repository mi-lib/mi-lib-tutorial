#include <roki/roki.h>

int main(int argc, char *argv[])
{
  rkChain robot;
  int id;
  zVec3D hand_local, hand_world;
  double dis;

  rkChainReadZTK( &robot, "super_robot.ztk" );
  zVec3DCreate( &hand_local, 0, -0.175, 0 );
  id = rkChainFindLinkID( &robot, "right_arm" );

  printf( "center of rotation\n" );
  zVec3DPrint( rkChainLinkWldPos( &robot, id ) );

  rkChainLinkPointWldPos( &robot, id, &hand_local, &hand_world );
  printf( "before rotation\n" );
  zVec3DPrint( &hand_world );

  dis = zDeg2Rad(90.0);
  rkChainLinkJointSetDis( &robot, id, &dis );
  rkChainUpdateFK( &robot );

  rkChainLinkPointWldPos( &robot, id, &hand_local, &hand_world );
  printf( "after rotation\n" );
  zVec3DPrint( &hand_world );

  rkChainDestroy( &robot );
  return 0;
}
