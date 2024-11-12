#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  rkIKCell *cell[3];
  zVec3D ref[3];

  rkChainReadZTK( &chain, "tri_arm.ztk" );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainFK( &chain, dis );
  rkChainRegisterIKJointAll( &chain, 0.01 );

  rkIKAttrSetLinkID( &attr, &chain, "hand1" );
  rkIKAttrSetWeight( &attr, 1.0, 1.0, 1.0 );
  cell[0] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID | RK_IK_ATTR_MASK_WEIGHT );
  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &ref[0] );
  rkIKAttrSetLinkID( &attr, &chain, "hand2" );
  cell[1] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID | RK_IK_ATTR_MASK_WEIGHT );
  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &ref[1] );
  rkIKAttrSetLinkID( &attr, &chain, "hand3" );
  rkIKAttrSetWeight( &attr, 0.1, 0.1, 0.1 );
  cell[2] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID | RK_IK_ATTR_MASK_WEIGHT );
  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &ref[2] );

  ref[0].c.y -= 1.0;
  ref[2].c.y += 1.0;
  rkIKCellSetRefVec( cell[0], &ref[0] );
  rkIKCellSetRefVec( cell[1], &ref[1] );
  rkIKCellSetRefVec( cell[2], &ref[2] );
  rkChainIK( &chain, dis, zTOL, 0 );

  rkChainInitFPrintZTK( stdout, &chain );
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
