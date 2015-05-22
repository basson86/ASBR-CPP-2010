/**
 *@file Simulator.hpp
 *@author Erion Plaku 
 *@brief Simulate robot motion and sensor
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>

/**
 *@author Erion Plaku
 *@brief Simulate robot motion and sensor
 *
 *@remark
 *  ::Simulator provides functionality to simulate the robot motion and the sensor.
 *    Simulator is considered as an input to your implementation of
 *    the bug algorithms. As such, you should not make changes to ::Simulator.
 *    In particular, your bug algorithms should only access the public functions
 *    of ::Simulator. 
 *    If, however, you see that the functionality provided by the ::Simulator class is not
 *    sufficient to implement your bug algorithms, please contact the support staff for this 
 *    project (see main webpage for contact information). In your email, succinctly describe
 *    the functionality you intend to add to this class and the reasons as to why it is 
 *    necessary for your implementation of the bug algorithms.
 */      
class Simulator
{
public:    
    /**
     *@brief Initialize variables
     */
    Simulator(void);
    
    /**
     *@brief Delete allocated memory
     */
    ~Simulator(void);

    /**
     *@brief Compute the minimum distance from the center 
     *       of the robot to the obstacles (if within sensing range)
     *
     *@param xmin 
     *@param ymin
     *
     *@remark
     *  - Returns the minimum distance from the center of the robot to the obstacles
     *  - upon return, xmin and ymin contain the x-coord and y-coord 
     *    of the obstacle point that achieves this miminum
     *  - if the obstacles are outside the sensing range, then ::GetSensorRadius() is returned and
     *    xmin and ymin are set to HUGE_VAL
     *  .
     */
    double TakeSensorReading(double &xmin, double &ymin) const;
 
    /**
     *@brief Maximum step length that the robot can take
     *@remark
     * Use this value when determining the dx and dy robot displacements.
     * You need to ensure that the robot is taking small steps, i.e.,
     * displacement (sqrt(dx * dx + dy * dy)) 
     * should not be larger than the max step length
     */
    double GetRobotMaxStepLength(void) const
    {
	return 0.05;
    }
               
    /**
     *@brief Get robot radius
     */
    double GetRobotRadius(void) const
    {
	return m_robotRadius;	
    }

    /**
     *@brief Get x-coordinate of robot's center
     */
    double GetRobotCenterX(void) const
    {
	return m_robotCenterX;	
    }

    /**
     *@brief Get y-coordinate of robot's center
     */
    double GetRobotCenterY(void) const
    {
	return m_robotCenterY;	
    }
    
    /**
     *@brief Get sensor radius
     */
    double GetSensorRadius(void) const
    {
	return m_sensorRadius;	
    }

    /**
     *@brief Get goal radius
     */
    double GetGoalRadius(void) const
    {
	return m_goalRadius;	
    }

    /**
     *@brief Get x-coordinate of goal center
     */
    double GetGoalCenterX(void) const
    {
	return m_goalCenterX;	
    }

    /**
     *@brief Get y-coordinate of goal center
     */
    double GetGoalCenterY(void) const
    {
	return m_goalCenterY;	
    }

    /**
     *@brief Get distance from the robot center to the goal
     */
    double GetDistanceFromRobotToGoal(void) const
    {
	return
	    sqrt((m_robotCenterX - m_goalCenterX) * (m_robotCenterX - m_goalCenterX) +
		 (m_robotCenterY - m_goalCenterY) * (m_robotCenterY - m_goalCenterY));	
    }

    /**
     *@brief Returns true iff the robot center is inside the goal circle
     */
    bool HasRobotReachedGoal(void) const
    {
	return
	    GetDistanceFromRobotToGoal() <= m_goalRadius;
    }


protected:
    /**
     *@brief Return true iff the robot is in collision with the obstacles
     */
    bool IsRobotInCollision(void) const;

    /**
     *@brief Set robot radius
     *@param r radius
     */ 
    void SetRobotRadius(const double r)
    {
	m_robotRadius = r;
    }

    /**
     *@brief Set robot center
     *
     *@param cx x position of center
     *@param cy y position of center
     */
    void SetRobotCenter(const double cx, const double cy)
    {
	m_robotCenterX = cx;
	m_robotCenterY = cy;
	
	m_path.push_back(cx);
	m_path.push_back(cy);
    }
    

   /**
     *@brief Set sensor radius
     *@param r radius
     */ 
    void SetSensorRadius(const double r)
    {
	m_sensorRadius = r;
    }
    
    /**
     *@brief Set goal radius
     *@param r radius
     */ 
    void SetGoalRadius(const double r)
    {
	m_goalRadius = r;
    }

    /**
     *@brief Set goal center
     *@param x x position of the goal center
     *@param y y position of the goal center
     */ 
    void SetGoalCenter(const double x, const double y)
    {
	m_goalCenterX = x;
	m_goalCenterY = y;	
    }

    /**
     *@brief Read polygonal obstacles from input file
     *
     *@param fname name of obstacle file
     */ 	
    void ReadObstacles(const char fname[]);


    /**
     *@brief Robot (robot is a circle)
     */
    double m_robotRadius;
    double m_robotCenterX;
    double m_robotCenterY;

    /**
     *@brief Sensor radius
     */
    double m_sensorRadius;

    /**
     *@brief Goal (goal region is a circle)
     */
    double m_goalRadius;
    double m_goalCenterX;
    double m_goalCenterY;

    /**
     *@brief Geometry of the obstacles
     *@remark
     *  Each obstacle corresponds to a polygon
     */
    struct Obstacle
    {
	std::vector<double> m_vertices;
	std::vector<int>    m_triangles;
    };
	
    std::vector<Obstacle *> m_obstacles;
    
    /**
     *@brief For storing the history of robot positions
     */
    std::vector<double> m_path;

    friend class Graphics;
	//friend class BugAlgorithms;
};

#endif
