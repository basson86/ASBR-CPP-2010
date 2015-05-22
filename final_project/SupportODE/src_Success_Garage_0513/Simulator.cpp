#include "Simulator.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <GL/glu.h>

const int TYPE_OBSTACLE = 1;
const int TYPE_TERRAIN  = 2;
const int TYPE_ROBOT    = 3;

Simulator::Simulator(const double posx, const double posy)
{
    m_collision = false;    

//Open Dynamics Engine stuff
    m_world     = dWorldCreate();
    m_space     = dHashSpaceCreate(0);
    m_contacts  = dJointGroupCreate(0);	    
    m_ground    = dCreatePlane(m_space, 0, 0, 1, 0);
    
    dGeomSetData(m_ground, (void *) &TYPE_TERRAIN);

    dWorldSetGravity(m_world, 0, 0, -0.81);

//create robot
    const double LENGTH = 2.50;	// chassis length 
    const double WIDTH  = 1.00;	// chassis width
    const double HEIGHT = 0.40;	// chassis height
    const double RADIUS = 0.45 * WIDTH;	// wheel radius
    const double STARTZ = 0.05;
    
    dMass m;
    dQuaternion q;
   
	double car_center_x= posx + 1;
	double car_center_y= posy + 6;

    // chassis body
    m_robotBodyChassis = dBodyCreate(m_world);
    dBodySetPosition(m_robotBodyChassis, car_center_x, car_center_y, 0.85 * RADIUS + 0.5 * HEIGHT + STARTZ);
    dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
    dBodySetMass(m_robotBodyChassis,&m);

    // chassis geometry
    m_robotGeomChassis = dCreateBox(m_space, LENGTH, WIDTH, HEIGHT);
    dGeomSetBody(m_robotGeomChassis, m_robotBodyChassis);

    m_robotBodies.push_back(m_robotBodyChassis);
    dGeomSetData(m_robotGeomChassis, (void *) &TYPE_ROBOT);

    // wheel bodies
    for(int i= 0; i < 3; i++) 
    {
	m_robotBodyWheels[i] = dBodyCreate(m_world);
	dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
	dBodySetQuaternion(m_robotBodyWheels[i], q);
	dMassSetSphere(&m, 1, RADIUS);
		
	dBodySetMass(m_robotBodyWheels[i], &m);
	m_robotGeomWheels[i] = dCreateSphere(m_space, RADIUS);

	dGeomSetBody(m_robotGeomWheels[i], m_robotBodyWheels[i]);

	m_robotBodies.push_back(m_robotBodyWheels[i]);	
	dGeomSetData(m_robotGeomWheels[i], (void *) &TYPE_ROBOT);
    }

    dBodySetPosition(m_robotBodyWheels[0],  car_center_x + 0.5 * LENGTH, car_center_y,               RADIUS + STARTZ);
    dBodySetPosition(m_robotBodyWheels[1],  car_center_x - 0.5 * LENGTH, car_center_y + 0.5 * WIDTH, RADIUS + STARTZ);
    dBodySetPosition(m_robotBodyWheels[2],  car_center_x - 0.5 * LENGTH, car_center_y - 0.5 * WIDTH, RADIUS + STARTZ);
    

    // front and back wheel hinges
    for (int i = 0; i < 3; i++) 
    {
	m_robotJoints[i] = dJointCreateHinge2(m_world, 0); // creat hign-2 joint as wheels
	dJointAttach(m_robotJoints[i], m_robotBodyChassis, m_robotBodyWheels[i]);
	const dReal *a = dBodyGetPosition(m_robotBodyWheels[i]);
	dJointSetHinge2Anchor(m_robotJoints[i], a[0], a[1], a[2]);
	dJointSetHinge2Axis1(m_robotJoints[i], 0, 0, 1);
	dJointSetHinge2Axis2(m_robotJoints[i], 0, 1, 0);
    
	// set joint suspension
	dJointSetHinge2Param(m_robotJoints[i], dParamSuspensionERP, 0.04);
	dJointSetHinge2Param(m_robotJoints[i], dParamSuspensionCFM, 0.08);
    }
        
    // lock back wheels along the steering axis
    for (int i = 1; i < 3; i++) 
    {
	// set stops to make sure wheels always stay in alignment
	dJointSetHinge2Param (m_robotJoints[i], dParamLoStop, 0);
	dJointSetHinge2Param (m_robotJoints[i], dParamHiStop, 0);
    }
}

Simulator::~Simulator(void)
{
}

void Simulator::AddGeometryAsObstacle(dGeomID geom)
{
    m_geoms.push_back(geom);
    dGeomSetData(geom, (void *) &TYPE_OBSTACLE);    
}

void Simulator::AddGeometryAsTerrain(dGeomID geom)
{
    m_geoms.push_back(geom);
    dGeomSetData(geom, (void *) &TYPE_TERRAIN);    
}

int Simulator::GetNrStateDimensions(void) const
{
    return 1 + 13 * m_robotBodies.size();
}

void Simulator::SetCurrentState(const double s[])
{
/*
 *        State for each body consists of position, orientation (stored as a quaternion), 
 *        linear velocity, and angular velocity. In the current robot model,
 *        the bodies are vehicle chassis and the vehicle wheels.
 */
    const int n = (int) m_robotBodies.size();
    for (int i = 0; i < n; ++i)
    {
	const int offset = i * 13;
	const dReal q[]  = {s[offset + 3], s[offset + 4], s[offset + 5], s[offset + 6]};
	
	dBodySetPosition  (m_robotBodies[i], s[offset], s[offset + 1], s[offset + 2]);
	dBodySetQuaternion(m_robotBodies[i], q);
	dBodySetLinearVel (m_robotBodies[i], s[offset +  7],  s[offset +  8], s[offset +  9]);
	dBodySetAngularVel(m_robotBodies[i], s[offset + 10],  s[offset + 11], s[offset + 12]);
    }
    dRandSetSeed((long unsigned int) (s[n * 13]));        
}

void Simulator::GetCurrentState(double s[])
{
/*
 *        State for each body consists of position, orientation (stored as a quaternion), 
 *        linear velocity, and angular velocity. In the current robot model,
 *        the bodies are vehicle chassis and the vehicle wheels.
 */
    const int n = (int) m_robotBodies.size();    
    for (int i = 0; i < n; ++i)
    {
	const dReal *pos    = dBodyGetPosition  (m_robotBodies[i]);
	const dReal *rot    = dBodyGetQuaternion(m_robotBodies[i]);
	const dReal *vel    = dBodyGetLinearVel (m_robotBodies[i]);
	const dReal *ang    = dBodyGetAngularVel(m_robotBodies[i]);
	const int    offset = 13 * i;
	
	s[offset     ] = pos[0];
	s[offset +  1] = pos[1];
	s[offset +  2] = pos[2];
	s[offset +  3] = rot[0];
	s[offset +  4] = rot[1];
	s[offset +  5] = rot[2];
	s[offset +  6] = rot[3];
	s[offset +  7] = vel[0];
	s[offset +  8] = vel[1];
	s[offset +  9] = vel[2];
	s[offset + 10] = ang[0];
	s[offset + 11] = ang[1];
	s[offset + 12] = ang[2];
    }
    s[13 * n] = dRandGetSeed();
}

void Simulator::GetCurrentChassisCenter(double c[])
{
    const dReal *pos = dBodyGetPosition(m_robotBodyChassis);

    c[0] = pos[0];
    c[1] = pos[1];
    c[2] = pos[2];    
}


bool Simulator::SimulateOneStep(const double speed, const double steer)
{
//set desired speed    
    dJointSetHinge2Param(m_robotJoints[0], dParamVel2, -speed); // hinge-2 velocity = -speed
    dJointSetHinge2Param(m_robotJoints[0], dParamFMax2, 10);   // maximum torque = 0.1
    
//set desired steering angle
    dReal v = steer - dJointGetHinge2Angle1 (m_robotJoints[0]);
    //if (v > 0.1) v = 0.1;
    //if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param(m_robotJoints[0], dParamVel, v);
    dJointSetHinge2Param(m_robotJoints[0], dParamFMax, 0.2);
    dJointSetHinge2Param(m_robotJoints[0], dParamLoStop, -0.75);
    dJointSetHinge2Param(m_robotJoints[0], dParamHiStop, 0.75);
    dJointSetHinge2Param(m_robotJoints[0], dParamFudgeFactor, 0.1);
    
//simulate the dynamics for one time step (set to 0.05)
    m_collision = false;

    dSpaceCollide(m_space, reinterpret_cast<void *>(this), &CollisionCheckingCallbackFn);
    dWorldStep(m_world, 0.2); // setting time step for integral
    dJointGroupEmpty(m_contacts);

    return !m_collision;    
}

void Simulator::CollisionCheckingCallbackFn(void *data, dGeomID geom1, dGeomID geom2)
{
    reinterpret_cast<Simulator *>(data)->CollisionChecking(geom1, geom2);
}

void Simulator::CollisionChecking(dGeomID geom1, dGeomID geom2)
{	
    const bool isRobot1    = dGeomGetData(geom1) == NULL || dGeomGetData(geom1) == ((void *) &TYPE_ROBOT);
    const bool isObstacle1 = dGeomGetData(geom1) == ((void *) &TYPE_OBSTACLE);
    const bool isTerrain1  = dGeomGetData(geom1) == ((void *) &TYPE_TERRAIN);

    const bool isRobot2    = dGeomGetData(geom2) == NULL || dGeomGetData(geom2) == ((void *) &TYPE_ROBOT);
    const bool isObstacle2 = dGeomGetData(geom2) == ((void *) &TYPE_OBSTACLE);
    const bool isTerrain2  = dGeomGetData(geom2) == ((void *) &TYPE_TERRAIN);
    
    if((isObstacle1 && isObstacle2) || (isTerrain1 && isTerrain2))
	return;
    
    if(dGeomIsSpace(geom1) || dGeomIsSpace(geom2)) 
    { 		
	dSpaceCollide2(geom1, geom2, this, &CollisionCheckingCallbackFn); 
	return;
    } 
    
    const int NR_CONTACTS = 3;
    dContact contact[NR_CONTACTS]; 
    if(int numc = dCollide(geom1, geom2, NR_CONTACTS, &contact[0].geom, sizeof(dContact)))
    {
	if((isRobot1 && isObstacle2) || (isRobot2 && isObstacle1))
	    m_collision = true;

	for(int i = 0; i < numc; ++i) 
	{
	    contact[i].surface.mode     = dContactSoftCFM | dContactApprox1;
	    contact[i].surface.mu       = 0.6;
	    contact[i].surface.soft_cfm = 0.2;
	    
	    dJointAttach(dJointCreateContact(m_world, m_contacts, &contact[i]),
			 dGeomGetBody(contact[i].geom.g1),
			 dGeomGetBody(contact[i].geom.g2));
	}		
    }
}

void Simulator::DrawEnvironment(void)
{    

//draw ground
    glColor3d(0.4, 0.5, 0.5);    
    glNormal3d(0, 0, 1);
    glBegin(GL_QUADS);
    glVertex3f(-30, -30, 0);
    glVertex3f( 30, -30, 0);
    glVertex3f( 30,  30, 0);
    glVertex3f(-30,  30, 0);
    glEnd();

//draw obstacles and terrain
    for(int i = 0; i < (int) m_geoms.size(); ++i)
	DrawGeometry(m_geoms[i]);
    
}

void Simulator::DrawRobot(void)
{
    DrawGeometry(m_robotGeomChassis);
    DrawGeometry(m_robotGeomWheels[0]);
    DrawGeometry(m_robotGeomWheels[1]);
    DrawGeometry(m_robotGeomWheels[2]);
}

void Simulator::DrawGeometry(dGeomID geom)
{
    if(dGeomGetData(geom) == ((void *) &TYPE_OBSTACLE))
	glColor3d(0.8, 0.2, 0.2);
    else if(dGeomGetData(geom) == ((void *) &TYPE_TERRAIN))
	glColor3d(0.4, 0.5, 0.7);
    else //type robot
	glColor3d(0.0, 0, 1.0);
    
    
    const int    type = dGeomGetClass(geom);    
    const dReal *pos  = dGeomGetPosition(geom);
    const dReal *rot  = dGeomGetRotation(geom);
    double       m[16];
    
    glPushMatrix();
 
//transform position and orientation into an OpenGL matrix   
    m[0]  = rot[0];
    m[1]  = rot[4];
    m[2]  = rot[8];
    m[4]  = rot[1];
    m[5]  = rot[5];
    m[6]  = rot[9];
    m[8]  = rot[2];
    m[9]  = rot[6];
    m[10] = rot[10];
    m[3]  = m[7] = m[11] =  0;
    m[15] = 1;
    m[12] = pos[0];
    m[13] = pos[1];
    m[14] = pos[2];
    
    glMultMatrixd(m);
    
    if(type == dBoxClass)
    {
	dVector3 lengths;
	dGeomBoxGetLengths(geom, lengths);

	glPushMatrix();
	glScaled(lengths[0], lengths[1], lengths[2]);
	glutSolidCube(1.0);
	glPopMatrix();

    }
    else if(type == dSphereClass)
	glutSolidSphere(dGeomSphereGetRadius(geom), 20, 20);
    else if(type == dCylinderClass)
    {
	dReal r, length;
	dGeomCylinderGetParams(geom, &r, &length);
	glTranslated(0, 0, -0.5 * length);
	
	static GLUquadric *gluQuadric = NULL;
	if(gluQuadric == NULL)
	    gluQuadric = gluNewQuadric();
	gluCylinder(gluQuadric, r, r, length, 20, 20);
    }
    
    glPopMatrix();
}


