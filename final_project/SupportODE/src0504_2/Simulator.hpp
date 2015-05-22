/**
 *@file Simulator.hpp
 *@author Erion Plaku 
 *@brief Simulator constructs robot and interfaces with Open Dynamics Engine (ODE)
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <ode/ode.h>
#include <vector>

/**
 *@author Erion Plaku 
 *@brief  Simulator constructs robot and interfaces with Open Dynamics Engine (ODE)
 */
class Simulator 
{
public:
   /**
    *@brief  Construct robot at position (posx, posy)
    */
    Simulator(const double posx, const double posy);
        
   /**
    *@brief Free memory
    */
    ~Simulator(void);
      
   /**
    *@brief  Add the object as an obstacle (robot should not collide with obstacles)
    *@param geom pointer to an ODE geometry object
    */
    void AddGeometryAsObstacle(dGeomID geom);
    
   /**
    *@brief  Add the object as terrain (robot can go over terrain, e.g., staris, ramp)
    *@param geom pointer to an ODE geometry object
    */
    void AddGeometryAsTerrain(dGeomID geom);
            
   /**
    *@brief Number of state dimensions (as represented by ODE)
    */
    int GetNrStateDimensions(void) const;

   /**
    *@brief  Set current state to s. 
    *        State for each body consists of position, orientation (stored as a quaternion), 
    *        linear velocity, and angular velocity. In the current robot model,
    *        the bodies are vehicle chassis and the vehicle wheels.
    *@param s current state
    */
    void SetCurrentState(const double s[]);
    
   /**
    *@brief  Get the internal state values from ODE and store them in s
    *        State for each body consists of position, orientation (stored as a quaternion), 
    *        linear velocity, and angular velocity. In the current robot model,
    *        the bodies are vehicle chassis and the vehicle wheels.
    */
    void GetCurrentState(double s[]);

   /**
    *@brief Get the chassis center and store it in c
    */
    void GetCurrentChassisCenter(double c[]);
    
   /**
    *@brief Simulate the dynamics for one time step when applying the controls
    *       speed (vehicle speed) and steer (vehicle steering angle).
    *       If a collision occurs, return false. Otherwise return true.
    *       After calling this function, you can retrieve the new state
    *       of the system by using the function GetCurrentState
    */
    bool SimulateOneStep(const double speed, const double steer);

    void DrawEnvironment(void);
    
    void DrawRobot(void);
        
    void DrawGeometry(dGeomID geom);

   /**
    *@brief Access to ODE variables
    */
    dWorldID GetWorldID(void)
    {
	return m_world;
    }
    
   /**
    *@brief Access to ODE variables
    */
    dSpaceID GetSpaceID(void)
    {
	return m_space;
    }

	dGeomID GetGeomID(int i)
    {
	return m_geoms[i];
    }
    
    
protected:
   /**
    *@brief ODE variables
    */
    dWorldID              m_world;
    dSpaceID              m_space;
    dGeomID               m_ground;
    dJointGroupID         m_contacts;
    std::vector<dGeomID>  m_geoms;
    std::vector<dBodyID>  m_robotBodies;
    dBodyID               m_robotBodyChassis;
    dGeomID               m_robotGeomChassis;
    dBodyID               m_robotBodyWheels[3];
    dGeomID               m_robotGeomWheels[3];
    dJointID              m_robotJoints[3];
    bool                  m_collision;
    
    /**
    *@brief Required to do collision checking via ODE
    */
    static void CollisionCheckingCallbackFn(void *data, dGeomID geom1, dGeomID geom2);
    
    void CollisionChecking(dGeomID geom1, dGeomID geom2);
};


#endif



