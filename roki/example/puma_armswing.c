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
  zVec q, dq, ddq;
  zVec3D vel, acc;
  zVec3D pos_prev, vel_prev, vel_approx, acc_approx;
  FILE *fp;
  int i;

  if( !rkChainReadZTK( &robot, "puma" ) ||
      !( q = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( dq = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( ddq = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( fp = fopen( "test.zvs", "w" ) ) )
    return EXIT_FAILURE;

  zVec3DCopy( rkChainLinkWldPos(&robot,6), &pos_prev );
  zMulMat3DVec3D( rkChainLinkWldAtt(&robot,6), rkChainLinkLinVel(&robot,6), &vel_prev );
  for( i=0; i<STEP; i++ ){
    set_joint_angle( q, dq, ddq, T*(double)i/STEP );
    rkChainSetJointDisAll( &robot, q );
    rkChainSetJointVelAll( &robot, dq );
    rkChainSetJointAccAll( &robot, ddq );
    rkChainUpdateFK( &robot );
    rkChainUpdateRate0G( &robot );
    zMulMat3DVec3D( rkChainLinkWldAtt(&robot,6), rkChainLinkLinVel(&robot,6), &vel );
    zMulMat3DVec3D( rkChainLinkWldAtt(&robot,6), rkChainLinkLinAcc(&robot,6), &acc );
    zVec3DSub( rkChainLinkWldPos(&robot,6), &pos_prev, &vel_approx );
    zVec3DDivDRC( &vel_approx, T / STEP );
    zVec3DSub( &vel, &vel_prev, &acc_approx );
    zVec3DDivDRC( &acc_approx, T / STEP );
    zVec3DCopy( rkChainLinkWldPos(&robot,6), &pos_prev );
    zVec3DCopy( &vel, &vel_prev );
    /* ZVS */
    fprintf( fp, "%g ", T / STEP );
    zVecFPrint( fp, q );
    /* hand position & velocity & acceleration */
    zVec3DDataPrint( rkChainLinkWldPos(&robot,6) );
    zVec3DDataPrint( &vel );
    zVec3DDataPrint( &vel_approx );
    zVec3DDataPrint( &acc );
    zVec3DDataPrint( &acc_approx );
    zEndl();
  }
  fclose( fp );
  zVecFreeAO( 3, q, dq, ddq );
  rkChainDestroy( &robot );
  return EXIT_SUCCESS;
}
