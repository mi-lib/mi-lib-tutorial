#include <roki/roki.h>

#define N 100

int main(int argc, char *argv[])
{
  rkChain robot;
  int j_head, j_righthand, j_leftfoot;
  zVec dis;
  double angle;
  int i;

  rkChainReadZTK( &robot, "super_robot.ztk" );
  dis = zVecAlloc( rkChainJointSize( &robot ) );

  j_head      = rkChainFindLinkJointIDOffset( &robot, "head" );
  j_righthand = rkChainFindLinkJointIDOffset( &robot, "right_arm" );
  j_leftfoot  = rkChainFindLinkJointIDOffset( &robot, "left_leg" );

  for( i=0; i<=N; i++ ){
    angle = zDeg2Rad(90) * i / N;
    zVecSetElem( dis, j_head,  0.5*angle );
    zVecSetElem( dis, j_righthand, angle );
    zVecSetElem( dis, j_leftfoot, -angle );
    printf( "%g ", 1.0 / N );
    zVecPrint( dis );
  }
  zVecFree( dis );
  rkChainDestroy( &robot );
  return 0;
}
