#include <roki/rk_chain.h>

void output_result(rkChain *chain)
{
  FILE *fp;

  fp = fopen( "result.ztk", "w" );
  rkChainInitFPrintZTK( fp, chain );
  fclose( fp );
}

void output_target(zVec3D ref[])
{
  FILE *fp;

  fp = fopen( "target.ztk", "w" );
  fprintf( fp, "[zeo::optic]\n" );
  fprintf( fp, "name: blue\n" );
  fprintf( fp, "ambient:  0.0, 0.0, 1.0\n" );
  fprintf( fp, "diffuse:  0.0, 0.0, 1.0\n" );
  fprintf( fp, "specular: 0.0, 0.0, 0.0\n" );
  fprintf( fp, "esr: 1.0\n" );
  fprintf( fp, "alpha: 0.5\n" );
  fprintf( fp, "[zeo::optic]\n" );
  fprintf( fp, "name: cyan\n" );
  fprintf( fp, "ambient:  0.0, 1.0, 1.0\n" );
  fprintf( fp, "diffuse:  0.0, 1.0, 1.0\n" );
  fprintf( fp, "specular: 0.0, 0.0, 0.0\n" );
  fprintf( fp, "esr: 1.0\n" );
  fprintf( fp, "alpha: 0.5\n" );
  fprintf( fp, "[zeo::optic]\n" );
  fprintf( fp, "name: yellow\n" );
  fprintf( fp, "ambient:  1.0, 1.0, 0.0\n" );
  fprintf( fp, "diffuse:  1.0, 1.0, 0.0\n" );
  fprintf( fp, "specular: 0.0, 0.0, 0.0\n" );
  fprintf( fp, "esr: 1.0\n" );
  fprintf( fp, "alpha: 0.5\n" );
  fprintf( fp, "[zeo::shape]\n" );
  fprintf( fp, "name: target1\n" );
  fprintf( fp, "type: sphere\n" );
  fprintf( fp, "optic: blue\n" );
  fprintf( fp, "center: " ); zVec3DFPrint( fp, &ref[0] );
  fprintf( fp, "radius: 0.03\n" );
  fprintf( fp, "[zeo::shape]\n" );
  fprintf( fp, "name: target2\n" );
  fprintf( fp, "type: sphere\n" );
  fprintf( fp, "optic: cyan\n" );
  fprintf( fp, "center: " ); zVec3DFPrint( fp, &ref[1] );
  fprintf( fp, "radius: 0.03\n" );
  fprintf( fp, "[zeo::shape]\n" );
  fprintf( fp, "name: target3\n" );
  fprintf( fp, "type: sphere\n" );
  fprintf( fp, "optic: yellow\n" );
  fprintf( fp, "center: " ); zVec3DFPrint( fp, &ref[2] );
  fprintf( fp, "radius: 0.03\n" );
  fclose( fp );
}

void output_error(rkChain *chain, rkIKCell *cell[], zVec3D ref[])
{
  zVec3D error;
  int i;

  for( i=0; i<3; i++ ){
    printf( "error at hand%d = %g\n", i, zVec3DNorm( zVec3DSub( &ref[i], rkChainLinkWldPos(chain,cell[i]->data.attr.id), &error ) ) );
  }
}

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
  cell[0] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &ref[0] );
  rkIKAttrSetLinkID( &attr, &chain, "hand2" );
  cell[1] = rkChainRegisterIKCellWldPos( &chain, NULL, 1, &attr, RK_IK_ATTR_MASK_ID );
  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &ref[1] );
  rkIKAttrSetLinkID( &attr, &chain, "hand3" );
  cell[2] = rkChainRegisterIKCellWldPos( &chain, NULL, 2, &attr, RK_IK_ATTR_MASK_ID );
  zVec3DCopy( rkChainLinkWldPos( &chain, attr.id ), &ref[2] );

  ref[0].c.y -= 0.3;
  ref[0].c.z -= 0.2;
  ref[2].c.y += 0.4;
  rkIKCellSetRefVec( cell[0], &ref[0] );
  rkIKCellSetRefVec( cell[1], &ref[1] );
  rkIKCellSetRefVec( cell[2], &ref[2] );
  rkChainIK( &chain, dis, zTOL, 0 );

  output_error( &chain, cell, ref );
  output_result( &chain );
  output_target( ref );
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
