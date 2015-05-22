/**
 *@file BugAlgorithms.hpp
 *@brief Prototype for the Bug algorithms required in this assignment
 */

#ifndef BUG_ALGORITHMS_HPP_
#define BUG_ALGORITHMS_HPP_

#include "Simulator.hpp"

/**
 *@brief Prototype for the different Bug algorithms required in this assignment
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class BugAlgorithms
{
public:
    /**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
    BugAlgorithms(Simulator * const simulator);
        
    
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     *  You should free the memory and delete other objects that you might have
     *  allocated in your implementation of this class
     */
    ~BugAlgorithms(void);
     
    
    /**
     *@brief Reset any internal variables/data structures to allow proper execution
     *       of your algorithm after user makes changes to problem setup
     *
     *@remark
     * This function maybe called when the user changes the current robot position,
     * robot radius, goal center, goal radius, or sensor radius \n\n
     * If your control strategies can handle such changes at any time, then you can
     * leave the implementation of this function empty. Otherwise, you may need to reset
     * the values of some (or all) of your variables and data structures in order to be
     * able to execute your bug strategies properly after the user has made such changes
     */
    void Restart(void);
    
    /**
     *@brief Strategy for the Bug1 algorithm
     *
     *@remark
     * Select the appropriate displacements dx and dy so that the robot behaves
     * as described in the Bug1 algorithm 
     */
    void Bug1Strategy(double &dx, double &dy);

    /**
     *@brief Strategy for the Bug2 algorithm
     *
     *@remark
     * Select the appropriate displacements dx and dy so that the robot behaves
     * as described in the Bug2 algorithm 
     */
    void Bug2Strategy(double &dx, double &dy);

    /**
     *@brief Strategy for your own Bug algorithm
     *
     *@remark
     * Design a new Bug algorithm. 
     * Select the appropriate displacements dx and dy so that the robot behaves
     * as described in your new Bug algorithm 
     */
    void MyBugStrategy(double &dx, double &dy);

    /**
     *@brief Select appropriate displacements along the x and y axis so 
     *       that the robot moves in a straight-line toward the goal
     *
     *@remark
     * This strategy is for partial-credit evaluation when your main 
     * bug algorithms do not work properly
     */
    void StepMoveStraightTowardGoal(double &dx, double &dy);
 
    /**
     *@brief Select appropriate displacements along the x and y axis so 
     *       that the robot follows obstacle boundary
     *
     *@remark
     * This strategy is for partial-credit evaluation when your main 
     * bug algorithms do not work properly
     */
    void StepFollowObstacleBoundary(double &dx, double &dy);
    
protected:
    /**
     *@brief Pointer to simulator, which provides access to robot, sensor, and goal
     */
    Simulator  *m_simulator;

	double iRCx;
	double iRCy; 
	// Initial Robot Position 

	double RCx; 
	double RCy;
    // Robot Position

	double GCx;
    double GCy;
    // Goal Position

		
	double LineSpeedFactor;
    // The parameter of speed to move straight toward the goal  
    
	
    double SeR;
	// Radius of Sensor
    double ObSensitivity;
	// The sensitivity of sensor to detect obstacle, defined as the factor to reduce the sensor radius 

	double DminToObs; // minimum distance to the obstacle
	double xmin;      
	double ymin;
	// the x-coord and y-coord of the obstacle point that achieves this miminum
	double contactx;
	double contacty;
    // 
	
	double DistRGH;
    double DistRGL;
    // distance to the goal at Hitting Point & Leaving Point

	double xH;
	double yH;
	double xL;
	double yL;
	// the x-cord & y-cord of Hitting Point and Leaving Point


	double Circumferrence; 	// For Bug 1 only, calculate the circumferrence of obstacle while searching the closest leaving point 
	double Search_path; //Record the path length at the closest leaving point 
	
	
	bool search; 
	bool hit;
	bool circle;
    // Bug status boolean logic, representing the status of bug

	double DistRGonBoundary; // the recorded distance to goal on the boundary of obstacle 
    
	
	double LineEqn; // for Bug 2 only, the resulting value of plugging the robot position to m-line equation   
    double LineEqnResidue; // the residule error to determine if the leaving point on the m-line
};

#endif
