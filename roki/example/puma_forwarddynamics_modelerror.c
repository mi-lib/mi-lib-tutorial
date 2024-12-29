#include <roki/roki.h>

void add_mp_error(rkChain *chain_model, rkChain *chain_real, double mass_error, double com_error)
{
  rkMP mp;
  zVec3D com_diff;
  zMat3D inertia_scale;
  int i;

  for( i=0; i<rkChainLinkNum(chain_real); i++ ){
    if( zIsTiny( rkChainLinkMass(chain_model,i) ) ) continue;
    rkMPSetMass( &mp, rkChainLinkMass(chain_model,i)*( 1 + zRandF(0,mass_error) ) );
    zVec3DCreate( &com_diff, zRandF(-com_error,com_error), zRandF(-com_error,com_error), zRandF(-com_error,com_error) );
    zVec3DAdd( rkChainLinkCOM(chain_model,i), &com_diff, rkMPCOM(&mp) );
    zMat3DMul( rkChainLinkInertia(chain_model,i), rkMPMass(&mp)/rkChainLinkMass(chain_model,i), &inertia_scale );
    zMat3DCatVec3DDoubleOuterProd( &inertia_scale, -rkMPMass(&mp), &com_diff, rkMPInertia(&mp) );
    rkLinkSetMP( rkChainLink(chain_real,i), &mp );
  }
}

typedef struct{
  rkChain *chain_model;
  rkChain *chain_real;
  zVec ref;
  zVec trq;
  double omega;
  double zeta;
} fd_data_t;

zVec chain_fd(double t, zVec dis, zVec vel, void *data, zVec acc)
{
  fd_data_t *fd_data;

  fd_data = data;
  zVecSub( dis, fd_data->ref, acc );
  zVecMulDRC( acc, -zSqr( fd_data->omega ) );
  zVecCatDRC( acc, -2*fd_data->zeta*fd_data->omega, vel );
  rkChainID( fd_data->chain_model, dis, vel, acc, fd_data->trq );
  rkChainFD( fd_data->chain_real, dis, vel, fd_data->trq, acc );
  return acc;
}

#define T  1.0
#define DT 0.001

#define DEFAULT_N 5

int main(int argc, char *argv[])
{
  zODE2 ode;
  fd_data_t fd_data;
  rkChain puma_model, puma_real;
  zVec dis, vel, acc;
  int size, i, step;
  FILE *fp;

  zRandInit();
  rkChainReadZTK( &puma_model, "puma.ztk" );
  rkChainReadZTK( &puma_real, "puma.ztk" );

  add_mp_error( &puma_model, &puma_real, 0.01, 0.01 );

  size = rkChainJointSize( &puma_model );
  dis = zVecAlloc( size );
  vel = zVecAlloc( size );
  acc = zVecAlloc( size );

  fd_data.chain_model = &puma_model;
  fd_data.chain_real = &puma_real;
  fd_data.ref = zVecAlloc( size );
  fd_data.trq = zVecAlloc( size );
  fd_data.omega = zPIx2 * 3;
  fd_data.zeta  = 1.0;
  zVecSetElem( fd_data.ref, 1, -zDeg2Rad(45) );
  zVecSetElem( fd_data.ref, 2, -zDeg2Rad(45) );

  zODE2Assign( &ode, Regular, NULL, NULL, NULL, NULL );
  zODE2AssignRegular( &ode, RKF45 );
  zODE2Init( &ode, size, 0, chain_fd );
  fp = fopen( "test.zvs", "w" );
  fprintf( fp, "%g ", DT );
  zVecFPrint( fp, dis );
  step = T / DT;
  for( i=0; i<=step; i++ ){
    zODE2Update( &ode, i*DT, dis, vel, DT, &fd_data );
    fprintf( fp, "%g ", DT );
    zVecFPrint( fp, dis );
  }
  fclose( fp );
  zODE2Destroy( &ode );
  zVecFreeAtOnce( 5, dis, vel, acc, fd_data.ref, fd_data.trq );
  rkChainDestroy( &puma_model );
  rkChainDestroy( &puma_real );
  return 0;
}
