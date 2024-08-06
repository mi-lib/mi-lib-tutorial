#include <roki/rk_chain.h>

#define T 3.0

#define STEP 100

void set_joint_angle(zVec q, double t)
{
  double phase;

  phase = zPIx2 * t / T;
  zVecSetElem( q, 0, zDeg2Rad(45) * sin(  phase) );
  zVecSetElem( q, 1, zDeg2Rad(30) * sin(2*phase) );
  zVecSetElem( q, 2, zDeg2Rad(60) * sin(2*phase) - zPI_2 );
}

int main(int argc, char *argv[])
{
  rkChain robot;
  zVec q, trq;
  double t, dt;
  int i;

  if( !rkChainReadZTK( &robot, "puma" ) ||
      !( q = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( trq = zVecAlloc( rkChainJointSize(&robot) ) ) )
    return EXIT_FAILURE;

  dt = T / STEP;
  for( i=0; i<STEP; i++ ){
    set_joint_angle( q, ( t = T*(double)i/STEP ) );
    rkChainFKCNT( &robot, q, dt );
    rkChainGetJointTrqAll( &robot, trq );
    printf( "%g %g %g %g %g %g %g\n", t, zVecElem(q,0), zVecElem(q,1), zVecElem(q,2), zVecElem(trq,0), zVecElem(trq,1), zVecElem(trq,2) );
  }
  zVecFreeAtOnce( 2, q, trq );
  rkChainDestroy( &robot );
  return EXIT_SUCCESS;
}
