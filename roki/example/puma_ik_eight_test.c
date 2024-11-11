#include <roki/rk_chain.h>

#define R  0.1

#define T  3.0
#define DT 0.01

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  zVec3D goal_origin, goal_rel;
  zFrame3D goal;
  rkIKCell *cell[2];
  rkIKAttr attr;
  double theta;
  int i, step;

  if( !rkChainReadZTK( &chain, "puma.ztk" ) ||
      !( dis = zVecAlloc( rkChainJointSize( &chain ) ) ) ) return EXIT_FAILURE;

  attr.id = rkChainFindLinkID( &chain, "link6" );
  cell[0] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  cell[1] = rkChainRegisterIKCellWldAtt( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );

  rkChainRegisterIKJointAll( &chain, 0.001 );

  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &goal_origin );
  goal_origin.c.x += 0.05;
  goal_origin.c.z += 0.1;
  zMat3DRotPitch( rkChainLinkWldAtt( &chain, attr.id ), -zDeg2Rad(90), zFrame3DAtt(&goal) );
  rkIKCellSetRefAtt( cell[1], zFrame3DAtt(&goal) );

  step = T / DT;
  for( i=0; i<=2*step; i++ ){
    theta = zPIx2 * i / step;
    zVec3DCreate( &goal_rel, 0, R*sin(theta), R*sin(2*theta) );
    zVec3DAdd( &goal_origin, &goal_rel, zFrame3DPos(&goal) );
    rkIKCellSetRefVec( cell[0], zFrame3DPos(&goal) );
    rkChainIK( &chain, dis, zTOL, 0 );
    printf( "%g ", DT );
    zVecPrint( dis );
  }
  zVecFree( dis );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
