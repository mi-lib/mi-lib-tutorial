#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  zFrame3D goal;
  rkIKCell *cell[2];
  rkIKAttr attr;

  if( !rkChainReadZTK( &chain, "puma.ztk" ) ||
      !( dis = zVecAlloc( rkChainJointSize( &chain ) ) ) ) return EXIT_FAILURE;

  rkIKAttrSetLinkID( &attr, &chain, "link6" );
  cell[0] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  cell[1] = rkChainRegisterIKCellWldAtt( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );

  rkChainRegisterIKJointAll( &chain, 0.001 );

  zFrame3DCopy( rkChainLinkWldFrame( &chain, attr.id ), &goal );
  zFrame3DPos(&goal)->c.x += 0.05;
  zFrame3DPos(&goal)->c.z += 0.1;
  zMat3DRotPitchDRC( zFrame3DAtt(&goal), -zDeg2Rad(90) );

  rkIKCellSetRefVec( cell[0], zFrame3DPos(&goal) );
  rkIKCellSetRefAtt( cell[1], zFrame3DAtt(&goal) );
  rkChainIK( &chain, dis, zTOL, 0 );

#if 1
  int i;

  printf( "[%s]\n", ZTK_TAG_ROKI_CHAIN_INIT );
  for( i=0; i<rkChainLinkNum(&chain); i++ ){
    if( rkChainLinkJointDOF(&chain,i) == 0 ) continue;
    printf( "joint: %s ", rkChainLinkName(&chain,i) );
    rkJointDisFPrintZTK( stdout, rkChainLinkJoint(&chain,i) );
  }
#else
  zVec6D error;

  printf( "goal frame\n" );
  zFrame3DPrint( &goal );
  printf( "final frame\n" );
  zFrame3DPrint( rkChainLinkWldFrame(&chain,attr.id) );
  printf( "error\n" );
  zVec6DPrint( zFrame3DError( &goal, rkChainLinkWldFrame(&chain,attr.id), &error ) );
#endif
  zVecFree( dis );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
