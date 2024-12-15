#include <roki/roki.h>

int ext_wrench(rkChain *chain)
{
  zVec3D vel, extforce;
  const double kz = 100.0;
  const double cz = 1.0;
  const double mu = 0.3;
  rkLink *l;

  l = rkChainLink( chain, rkChainLinkNum(chain)-1 );
  zMulMat3DVec3D( rkLinkWldAtt(l), rkLinkLinVel(l), &vel );
  extforce.c.z = -zMin( kz * rkLinkWldPos(l)->c.z, 0 ) -zMax( cz * vel.c.z, 0 );
  extforce.c.x = -mu * extforce.c.z * zSgn(vel.c.x);
  extforce.c.y = -mu * extforce.c.z * zSgn(vel.c.y);
  zMulMat3DTVec3DDRC( rkLinkWldAtt(l), &extforce );
  rkLinkSetExtForce( l, &extforce, ZVEC3DZERO );
  return 1;
}

zVec chain_fd(double t, zVec dis, zVec vel, void *chain, zVec acc)
{
  zVec trq;
  int i;

  trq = zVecAlloc( zVecSizeNC( dis ) );
  for( i=0; i<zVecSizeNC(trq); i++ )
    zVecSetElem( trq, i, -0.1*zVecElem(vel,i) );
  ext_wrench( (rkChain *)chain );
  rkChainFD( (rkChain *)chain, dis, vel, trq, acc );
  zVecFree( trq );
  return acc;
}

#define T  3.0
#define DT 0.001

#define DEFAULT_N 5

int main(int argc, char *argv[])
{
  zODE2 ode;
  rkChain chain;
  zVec dis, vel, acc;
  int size, i, step;
  FILE *fp;

  rkChainReadZTK( &chain, "pendula.ztk" );
  size = rkChainJointSize( &chain );
  dis = zVecAlloc( size );
  vel = zVecAlloc( size );
  acc = zVecAlloc( size );
  zVecSetElem( dis, 0, zDeg2Rad(45) );
  rkChainUpdateFK( &chain );
  rkChainUpdateID( &chain );

  zODE2Assign( &ode, Sympl, NULL, NULL, NULL, NULL );
  zODE2Init( &ode, size, 0, chain_fd );
  fp = fopen( "test.zvs", "w" );
  fprintf( fp, "%g ", DT );
  zVecFPrint( fp, dis );
  step = T / DT;
  for( i=0; i<=step; i++ ){
    zODE2Update( &ode, i*DT, dis, vel, DT, &chain );
    fprintf( fp, "%g ", DT );
    zVecFPrint( fp, dis );
  }
  fclose( fp );
  zODE2Destroy( &ode );
  zVecFreeAtOnce( 3, dis, vel, acc );
  rkChainDestroy( &chain );
  return 0;
}
