#include <roki/roki.h>

int main(int argc, char *argv[])
{
  rkChain robot;
  rkLink *l;
  zVec3D hand_local, hand_world;
  double dis;

  rkChainReadZTK( &robot, "super_robot.ztk" );
  zVec3DCreate( &hand_local, 0, -0.175, 0 ); /* right hand position in the iink frame */
  l = rkChainFindLink( &robot, "right_arm" );

  printf( "center of rotation\n" );
  zVec3DPrint( rkLinkWldPos(l) );

  rkLinkPointWldPos( l, &hand_local, &hand_world );
  printf( "before rotation\n" );
  zVec3DPrint( &hand_world );

  dis = zDeg2Rad(90.0);
  rkLinkJointSetDis( l, &dis );
  rkChainUpdateFK( &robot );

  rkLinkPointWldPos( l, &hand_local, &hand_world );
  printf( "after rotation\n" );
  zVec3DPrint( &hand_world );

  rkChainDestroy( &robot );
  return 0;
}
