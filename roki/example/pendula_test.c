#include <roki/roki.h>

rkChain *create_pendula(rkChain *chain, double length, int num)
{
  int i;
  char name[BUFSIZ];
  double d;
  rkMP mp;

  rkChainInit( chain );
  zNameSet( chain, "pendula" );
  rkLinkArrayAlloc( rkChainLinkArray(chain), num + 1 );
  d = length / num;
  rkMPSetMass( &mp, 1.0/num );
  zVec3DCreate( rkMPCOM(&mp), 0.5*d, 0, 0 );
  zMat3DCreate( rkMPInertia(&mp),
    rkMPMass(&mp)*(0.03+d*d)/12, 0, 0,
    0, rkMPMass(&mp)*(0.03+d*d)/12, 0,
    0, 0, rkMPMass(&mp)*0.03/6 );
  /* base */
  rkLinkInit( rkChainLink(chain,0) );
  zVec3DCreate( rkChainLinkOrgPos(chain,0), 0, 0, length*(num-1)/num );
  zMat3DFromZYX( rkChainLinkOrgAtt(chain,0), 0, zDeg2Rad(90), 0 );
  zNameSet( rkChainLink(chain,0), "base" );
  rkJointAssignByStr( rkChainLinkJoint(chain,0), "revolute" );
  /* pendulum */
  for( i=1; i<num; i++ ){
    sprintf( name, "link%d", i );
    rkLinkInit( rkChainLink(chain,i) );
    rkLinkSetMP( rkChainLink(chain,i), &mp );
    zVec3DCreate( rkChainLinkOrgPos(chain,i), d, 0, 0 );
    zNameSet( rkChainLink(chain,i), name );
    rkJointAssignByStr( rkChainLinkJoint(chain,i), "revolute" );
    rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  /* tip */
  rkLinkInit( rkChainLink(chain,num) );
  zVec3DCreate( rkChainLinkOrgPos(chain,num), d, 0, 0 );
  zNameSet( rkChainLink(chain,num), "tip" );
  rkJointAssignByStr( rkChainLinkJoint(chain,num), "fixed" );
  rkLinkAddChild( rkChainLink(chain,num-1), rkChainLink(chain,num) );

  rkChainSetJointIDOffset( chain );
  rkChainUpdateCRBMass( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return chain;
}

zVec chain_fd(double t, zVec dis, zVec vel, void *chain, zVec acc)
{
  zVec trq;
  int i;

  trq = zVecAlloc( zVecSizeNC( dis ) );
  for( i=0; i<zVecSizeNC(trq); i++ )
    zVecSetElem( trq, i, -0.1*zVecElem(vel,i) );
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

  create_pendula( &chain, 1.0, argc > 1 ? atoi( argv[1] ) : DEFAULT_N );
  rkChainWriteZTK( &chain, "pendula.ztk" );
  size = rkChainJointSize( &chain );
  dis = zVecAlloc( size );
  vel = zVecAlloc( size );
  acc = zVecAlloc( size );
  zVecSetElem( dis, 0, zDeg2Rad(45) );
  rkChainFK( &chain, dis );
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
