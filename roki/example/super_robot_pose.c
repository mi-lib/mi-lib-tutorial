#include <roki/roki.h>

#define LINK_ID_HEAD 1
#define LINK_ID_RARM 3
#define LINK_ID_LLEG 4

void output(rkChain *robot)
{
  zVec3D hand_local, hand_world, hand_d, foot_local, foot_world, foot_d;

  zVec3DCreate( &hand_local, 0, -0.175, 0 );
  zVec3DCreate( &foot_local, 0, -0.175, 0 );

  printf( "head attitude (before rotation)\n" );
  zMat3DPrint( rkChainLinkWldAtt( robot, LINK_ID_HEAD ) );

  printf( "right hand position from joint center\n" );
  rkChainLinkPointWldPos( robot, LINK_ID_RARM, &hand_local, &hand_world );
  zVec3DPrint( zVec3DSub( &hand_world, rkChainLinkWldPos( robot, LINK_ID_RARM ), &hand_d ) );

  printf( "leg foot position from joint center\n" );
  rkChainLinkPointWldPos( robot, LINK_ID_LLEG, &foot_local, &foot_world );
  zVec3DPrint( zVec3DSub( &foot_world, rkChainLinkWldPos( robot, LINK_ID_LLEG ), &foot_d ) );
}

int main(int argc, char *argv[])
{
  rkChain robot;
  int j_head, j_righthand, j_leftfoot;
  zVec dis;

  rkChainReadZTK( &robot, "super_robot.ztk" );
  dis = zVecAlloc( rkChainJointSize( &robot ) );

  j_head      = rkChainFindLinkJointIDOffset( &robot, "head" );
  j_righthand = rkChainFindLinkJointIDOffset( &robot, "right_arm" );
  j_leftfoot  = rkChainFindLinkJointIDOffset( &robot, "left_leg" );

  printf( "(before rotation)\n" );
  output( &robot );
  zVecSetElem( dis, j_head,      zDeg2Rad(45) );
  zVecSetElem( dis, j_righthand, zDeg2Rad(90) );
  zVecSetElem( dis, j_leftfoot,  zDeg2Rad(-90) );
  rkChainFK( &robot, dis );
  printf( "after rotation\n" );
  output( &robot );

  zVecFree( dis );
  rkChainDestroy( &robot );
  return 0;
}
