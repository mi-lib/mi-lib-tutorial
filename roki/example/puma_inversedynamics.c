#include <roki/rk_chain.h>

#define T 3.0

#define STEP 100

void set_joint_angle(zVec q, zVec dq, zVec ddq, double t)
{
  double phase, omega;

  omega = zPIx2 / T;
  phase = omega * t;
  zVecSetElem( q, 0, zDeg2Rad(45) * sin(  phase) );
  zVecSetElem( q, 1, zDeg2Rad(30) * sin(2*phase) );
  zVecSetElem( q, 2, zDeg2Rad(60) * sin(2*phase) - zPI_2 );
  zVecSetElem( dq, 0,   omega * zDeg2Rad(45) * cos(  phase) );
  zVecSetElem( dq, 1, 2*omega * zDeg2Rad(30) * cos(2*phase) );
  zVecSetElem( dq, 2, 2*omega * zDeg2Rad(60) * cos(2*phase) );
  zVecSetElem( ddq, 0,-  zSqr(omega) * zDeg2Rad(45) * sin(  phase) );
  zVecSetElem( ddq, 1,-4*zSqr(omega) * zDeg2Rad(30) * sin(2*phase) );
  zVecSetElem( ddq, 2,-4*zSqr(omega) * zDeg2Rad(60) * sin(2*phase) );
}

int main(int argc, char *argv[])
{
  rkChain robot;
  zVec q, dq, ddq, trq;
  int i;
  double t;

  if( !rkChainReadZTK( &robot, "puma" ) ||
      !( q = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( dq = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( ddq = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( trq = zVecAlloc( rkChainJointSize(&robot) ) ) )
    return EXIT_FAILURE;

  for( i=0; i<STEP; i++ ){
    set_joint_angle( q, dq, ddq, ( t = T*(double)i/STEP ) );
    rkChainID( &robot, q, dq, ddq, trq );
    printf( "%g %g %g %g %g %g %g %g %g %g\n", t, zVecElem(q,0), zVecElem(q,1), zVecElem(q,2), zVecElem(dq,0), zVecElem(dq,1), zVecElem(dq,2), zVecElem(trq,0), zVecElem(trq,1), zVecElem(trq,2) );
  }
  zVecFreeAtOnce( 4, q, dq, ddq, trq );
  rkChainDestroy( &robot );
  return EXIT_SUCCESS;
}
