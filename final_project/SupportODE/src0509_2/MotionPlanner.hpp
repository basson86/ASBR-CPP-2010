/**
 *@file MotionPlanner.hpp
 *@brief Interface for the motion-planning algorithms
 */

#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"

/**
 *@brief Interface for the motion-planning algorithms
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class MotionPlanner
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
    MotionPlanner(Simulator * const simulator);
                
    
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~MotionPlanner(void);
    
    /**
     *@brief Run motion-planning algorithm for at most tmax seconds
     *
     *@param tmax upper bound on the computation time for the motion-planning algorithm
     *@remark
     *  The graphical interface will call this function and pass to it a certain
     *  value for the time upper bound. Your implementation should
     *  not run for much longer than this upper bound.
     */
    void Solve(const double tmax);

    /**
     *@brief Get the solution path computed by the motion planner
     *
     *@param path vector where you can store your path
     *@remark
     *  After running your motion-planning algorithm for tmax seconds, the graphical
     *  interface will call this function to obtain the solution solution that your
     *  algorithm has found. 
     */
    void GetSolutionPath(std::vector<double> * path);

    /**
     *@brief Draw motion planner
     *@remark
     *  You can use this function to draw your motion planner. It may help you during
     *  debugging to figure out how the motion planner is performing.
     */
    void Draw(void);

    
protected:
    /**
     *@brief Pointer to simulator, which provides access to robot, obstacles, reward regions
     */
    Simulator  *m_simulator;

    /**
     *@brief Simple vertex class for tree planner implementation
     */
    struct Vertex
    {
	double *m_state;
	double  m_center[3];  
	int     m_parent;

	int     motion;
    
	// cost value to decide optimal path
    double  cost;
	int visit;
    };

    /**
     *@brief Vertices created by tree planner
     */
	std::vector<Vertex *> m_vertices;
	
	
	//Parameters for grid search
	
	double xdim;
	double ydim;
    double FullOrient;	
	int NCellx;
	int NCelly;
    int NAngle;
	double Grid_size_x;
    double Grid_size_y;

    double Grid_size_angle;
	int ***m_config_grid;
	
    
  
	int FOindex;
	Vertex *FOvertex ;


	double residue_error;
   
	// paramenter of generating successor state
    
	double step_speed;
    double step_steer;


    bool cmp_by_cost(Vertex* a, Vertex* b);
    

    /**
     *@brief Id of vertex where goal is reached
     */
    int m_vidGoal;
    
	// parameters for garage & parallel parking lot
	dGeomID GarageW1;
    dGeomID GarageW2;
	dGeomID ParkingLotW1;
    dGeomID ParkingLotW2;

	const dReal *GarageW1pos;
	const dReal *GarageW2pos;

	const dReal *ParkingLotW1pos;
	const dReal *ParkingLotW2pos;

	std::vector<dReal> GarageState; 
    std::vector<dReal> ParkingLotState;
   
	
    


    friend class Graphics;    
};

#endif
