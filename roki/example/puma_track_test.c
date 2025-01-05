#include <roki/rk_chain.h>

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

/* place a series of star-shaped viapoints */
bool create_viapoints(zVec3DArray *viapoints)
{
  const zVec3D center = { { 0.3, 0.05, 0.3 } };
  const double radius = 0.1;
  double angle;
  int i;
  FILE *fp;

  zVec3DArrayAlloc( viapoints, 6 );
  if( zArraySize(viapoints) != 6 ) return false;
  fp = fopen( "star_ref", "w" );
  for( i=0; i<5; i++ ){
    angle = zPIx2 * 2 * i / 5;
    zVec3DCat( &center, radius*sin(angle), ZVEC3DY, zArrayElemNC(viapoints,i) );
    zVec3DCatDRC( zArrayElemNC(viapoints,i), radius*cos(angle), ZVEC3DZ );
    zVec3DValueNLFPrint( fp, zArrayElemNC(viapoints,i) );
  }
  zVec3DCopy( zArrayElemNC(viapoints,0), zArrayElemNC(viapoints,5) );
  zVec3DValueFPrint( fp, zArrayElemNC(viapoints,5) );
  fclose( fp );
  return true;
}

/* get referential position of the endpoint */
zVec3D *get_ref_point(zVec3DArray *viapoints, double t, double term, zVec3D *refpoint)
{
  zPexIP pc;
  int n;

  if( !zPexIPCreateBoundary( &pc, term, 0, 0, 0, 1, 0, 0, NULL ) ) return NULL;
  for( n=0; t>term; t-=term, n++ );
  zVec3DInterDiv( zArrayElemNC(viapoints,n), zArrayElemNC(viapoints,n+1), zPexIPVal(&pc,t), refpoint );
  zPexIPFree( &pc );
  return refpoint;
}

typedef struct{
  /* robot models */
  rkChain chain_model;   /* robot model for inverse kinematics and computed torque method */
  /* joint control vectors */
  zVec trq;              /* joint driving torque */
  zVec dis_ref;          /* referential joint displacement */
  zVec dis_ref_1;        /* one-step-before referential joint displacement */
  zVec dis_ref_2;        /* two-step-before referential joint displacement */
  zVec dis_obs;          /* observed joint displacement */
  zVec vel_obs;          /* observed joint velocity */
  zVec tmp;
  /* referential trajectory */
  zVec3DArray viapoints; /* viapoints for referential trajectory */
  double term;           /* duration for moving from a viapoint to another */
  /* control parameters */
  double omega;          /* angular-eigenfrequency for proportional compensation */
  double zeta;           /* damping coefficient */
  /* inverse kinematics constraints */
  rkIKCell *ikcell[2];
} track_ctrlr_t;

bool track_ctrlr_create(track_ctrlr_t *ctrlr, const char *filename, double omega, double zeta, double term)
{
  rkIKAttr attr;
  zMat3D ref_att;

  /* create robot models */
  if( !rkChainReadZTK( &ctrlr->chain_model, filename ) ) return false;
  /* allocate joint displacement/velocity/acceleration/torque vectors */
  ctrlr->trq = zVecAlloc( rkChainJointSize(&ctrlr->chain_model) );
  ctrlr->dis_ref = zVecAlloc( zVecSizeNC(ctrlr->trq) );
  ctrlr->dis_ref_1 = zVecAlloc( zVecSizeNC(ctrlr->trq) );
  ctrlr->dis_ref_2 = zVecAlloc( zVecSizeNC(ctrlr->trq) );
  ctrlr->dis_obs = zVecAlloc( zVecSizeNC(ctrlr->trq) );
  ctrlr->vel_obs = zVecAlloc( zVecSizeNC(ctrlr->trq) );
  ctrlr->tmp     = zVecAlloc( zVecSizeNC(ctrlr->trq) );
  if( !ctrlr->trq || !ctrlr->dis_ref || !ctrlr->dis_ref_1 || !ctrlr->dis_ref_2 ||
      !ctrlr->dis_obs || !ctrlr->vel_obs || !ctrlr->tmp ) return false;
  /* initialize inverse kinematics solver */
  attr.id = rkChainFindLinkID( &ctrlr->chain_model, "link6" );
  ctrlr->ikcell[0] = rkChainRegisterIKCellWldPos( &ctrlr->chain_model, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  ctrlr->ikcell[1] = rkChainRegisterIKCellWldAtt( &ctrlr->chain_model, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  if( !ctrlr->ikcell[0] || !ctrlr->ikcell[1] ) return false;
  if( !rkChainRegisterIKJointAll( &ctrlr->chain_model, 0.001 ) ) return false;
  /* create viapoints on the referential trajectory */
  if( !create_viapoints( &ctrlr->viapoints ) ) return false;
  ctrlr->term = term;
  /* initialize tracking control parameters */
  ctrlr->omega = omega;
  ctrlr->zeta = zeta;
  /* initialize robot posture */
  rkIKCellSetRefVec( ctrlr->ikcell[0], zArrayElemNC(&ctrlr->viapoints,0) );
  zMat3DRotPitch( rkChainLinkWldAtt( &ctrlr->chain_model, attr.id ), -zDeg2Rad(90), &ref_att );
  rkIKCellSetRefAtt( ctrlr->ikcell[1], &ref_att );
  rkChainIK( &ctrlr->chain_model, ctrlr->dis_ref, zTOL, 0 );
  zVecCopy( ctrlr->dis_ref, ctrlr->dis_ref_1 );
  zVecCopy( ctrlr->dis_ref, ctrlr->dis_ref_2 );
  return true;
}

void track_ctrlr_destroy(track_ctrlr_t *ctrlr)
{
  zVecFree( ctrlr->trq );
  zVecFree( ctrlr->dis_ref );
  zVecFree( ctrlr->dis_ref_1 );
  zVecFree( ctrlr->dis_ref_2 );
  zVecFree( ctrlr->dis_obs );
  zVecFree( ctrlr->vel_obs );
  zVecFree( ctrlr->tmp );
  rkChainDestroy( &ctrlr->chain_model );
}

void track_ctrlr_copy_state(track_ctrlr_t *ctrlr, zVec dis, zVec vel)
{
  zVecCopyNC( dis, ctrlr->dis_obs );
  zVecCopyNC( vel, ctrlr->vel_obs );
  rkChainFK( &ctrlr->chain_model, ctrlr->dis_obs );
}

zVec track_ctrlr_trq(track_ctrlr_t *ctrlr, double t, double dt, zVec acc)
{
  zVec3D ref_point;

  /* referential joint displacement via inverse kinematics */
  get_ref_point( &ctrlr->viapoints, t, ctrlr->term, &ref_point );
  rkIKCellSetRefVec( ctrlr->ikcell[0], &ref_point );
  rkIKCellEnable( ctrlr->ikcell[1] );
  rkChainIK( &ctrlr->chain_model, ctrlr->dis_ref, zTOL, 0 );
  /* PD compensation control */
  zVecCat( ctrlr->dis_ref, -2, ctrlr->dis_ref_1, ctrlr->tmp );
  zVecAddDRC( ctrlr->tmp, ctrlr->dis_ref_2 );
  zVecDiv( ctrlr->tmp, zSqr(dt), acc ); /* referential acceleration */
  zVecSub( ctrlr->dis_ref, ctrlr->dis_ref_1, ctrlr->tmp );
  zVecDivDRC( ctrlr->tmp, dt );
  zVecSubDRC( ctrlr->tmp, ctrlr->vel_obs );
  zVecCatDRC( acc, 2*ctrlr->zeta*ctrlr->omega, ctrlr->tmp ); /* velocity error feedback */
  zVecSub( ctrlr->dis_ref, ctrlr->dis_obs, ctrlr->tmp );
  zVecCatDRC( acc, zSqr(ctrlr->omega), ctrlr->tmp ); /* displacement error feedback */
  rkChainID( &ctrlr->chain_model, ctrlr->dis_obs, ctrlr->vel_obs, acc, ctrlr->trq );
  /* update history of referential joint displacement */
  zVecCopy( ctrlr->dis_ref_1, ctrlr->dis_ref_2 );
  zVecCopy( ctrlr->dis_ref, ctrlr->dis_ref_1 );
  return ctrlr->trq;
}

typedef struct{
  rkChain *chain_real;
  track_ctrlr_t *ctrlr;
  double dt;
} fd_data_t;

void fd_data_set(fd_data_t *fd_data, rkChain *chain_real, track_ctrlr_t *ctrlr, double dt, zVec dis, zVec vel, zVec acc)
{
  const double mass_error_max = 0.1;
  const double com_error_max = 0.03;

  zVecCopy( ctrlr->dis_ref, dis );
  rkChainID( chain_real, dis, vel, acc, ctrlr->trq );
  fd_data->chain_real = chain_real;
  fd_data->ctrlr = ctrlr;
  fd_data->dt = dt;
  add_mp_error( &ctrlr->chain_model, chain_real, mass_error_max, com_error_max );
}

zVec chain_fd(double t, zVec dis, zVec vel, void *data, zVec acc)
{
  fd_data_t *fd_data;

  fd_data = data;
  track_ctrlr_trq( fd_data->ctrlr, t, fd_data->dt, acc );
  rkChainFD( fd_data->chain_real, dis, vel, fd_data->ctrlr->trq, acc );
  return acc;
}

#define T  1.0
#define DT 0.001

int main(int argc, char *argv[])
{
  rkChain chain_real;
  track_ctrlr_t ctrlr;
  fd_data_t fd_data;
  zVec dis;
  zVec vel;
  zVec acc;
  zODE2 ode;
  int i, step;
  FILE *fp;

  if( !rkChainReadZTK( &chain_real, "puma.ztk" ) ) return EXIT_FAILURE;
  dis = zVecAlloc( rkChainJointSize( &chain_real ) );
  vel = zVecAlloc( zVecSizeNC(dis) );
  acc = zVecAlloc( zVecSizeNC(dis) );
  if( !track_ctrlr_create( &ctrlr, "puma.ztk", zPIx2 * 3, 1.0, T ) ) return EXIT_FAILURE;
  fd_data_set( &fd_data, &chain_real, &ctrlr, DT, dis, vel, acc );

  zODE2Assign( &ode, Regular, NULL, NULL, NULL, NULL );
  zODE2AssignRegular( &ode, RK4 );
  zODE2Init( &ode, rkChainJointSize(&chain_real), 0, chain_fd );
  step = T / DT;
  fp = fopen( "e", "w" );
  for( i=0; i<=5*step; i++ ){
    track_ctrlr_copy_state( &ctrlr, dis, vel );
    zODE2Update( &ode, i*DT, dis, vel, DT, &fd_data );
    printf( "%g ", DT );
    zVecPrint( dis );
    zVec3DValueNLFPrint( fp, rkChainLinkWldPos(&chain_real,6) );
  }
  fclose( fp );

  zODE2Destroy( &ode );
  zVecFree( dis );
  zVecFree( vel );
  zVecFree( acc );
  track_ctrlr_destroy( &ctrlr );
  rkChainDestroy( &chain_real );
  return EXIT_SUCCESS;
}
