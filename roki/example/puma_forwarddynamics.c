#include <roki/roki.h>

typedef struct{
  rkChain *chain;
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
  rkChainID( fd_data->chain, dis, vel, acc, fd_data->trq );
  rkChainFD( fd_data->chain, dis, vel, fd_data->trq, acc );
  return acc;
}

#define T  1.0
#define DT 0.001

#define DEFAULT_N 5

int main(int argc, char *argv[])
{
  zODE2 ode;
  fd_data_t fd_data;
  rkChain chain;
  zVec dis, vel, acc;
  int size, i, step;
  FILE *fp;

  rkChainReadZTK( &chain, "puma.ztk" );
  size = rkChainJointSize( &chain );
  dis = zVecAlloc( size );
  vel = zVecAlloc( size );
  acc = zVecAlloc( size );

  fd_data.chain = &chain;
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
  rkChainDestroy( &chain );
  return 0;
}
