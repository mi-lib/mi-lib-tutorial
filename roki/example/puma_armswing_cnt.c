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
  zVec q;
  zVec3D vel, acc;
  FILE *fp;
  double dt;
  int i;

  if( !rkChainReadZTK( &robot, "puma" ) ||
      !( q = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( fp = fopen( "test.zvs", "w" ) ) )
    return EXIT_FAILURE;

  dt = T / STEP;
  for( i=0; i<STEP; i++ ){
    set_joint_angle( q, T*(double)i/STEP );
    rkChainFKCNT( &robot, q, dt );
    zMulMat3DVec3D( rkChainLinkWldAtt(&robot,6), rkChainLinkLinVel(&robot,6), &vel );
    zMulMat3DVec3D( rkChainLinkWldAtt(&robot,6), rkChainLinkLinAcc(&robot,6), &acc );
    zVec3DSubDRC( &acc, RK_GRAVITY3D );
    /* hand position & velocity & acceleration */
    zVec3DValuePrint( rkChainLinkWldPos(&robot,6) );
    zVec3DValuePrint( &vel );
    zVec3DValuePrint( &acc );
    zEndl();
  }
  fclose( fp );
  zVecFree( q );
  rkChainDestroy( &robot );
  return EXIT_SUCCESS;
}
