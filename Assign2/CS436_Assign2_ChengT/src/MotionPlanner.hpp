/**
 *@file MotionPlanner.hpp
 *@brief Interface for the motion-planning algorithms
 */

#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"
//using namespace std;

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
     *  value for the time upper bound. Your implementation (which will be in
     *  different classes that extend MotionPlanner, e.g., RRT, PRM) should
     *  not run for much longer than this upper bound.
     */
    void Solve(const double tmax);

    /**
     *@brief Get the highest-reward path computed by the motion planner so far
     *
     *@param path vector where you can store your path
     *@remark
     *  After running your motion-planning algorithm for tmax seconds, the graphical
     *  interface will call this function to obtain the best solution that your
     *  algorithm has found. The best solution is the collision-free path with
     *  the highest accumulated total reward.
     *@remark
     *  The path will consist of a sequence of configurations (orientations + positions), i.e.,
     *  <CENTER>(theta_1, x_1, y_1), (theta_2, x_2, y_2), ...., (theta_n, x_n, y_n)</CENTER>
     *  You can push these values into the vector by using the push_back function, i.e.,
     * <CENTER>
     *   path->push_back(theta_1);
     *   path->push_back(x_1);
     *   path->push_back(y_1);
     *   ...and so on
     * </CENTER>
     *  
     */
    void GetHighestRewardCollisionFreePath(std::vector<double> * path);

    /**
     *@brief Draw motion planner
     *@remark
     *  You can use this function to draw your motion planner. It may help you during
     *  debugging to figure out how the motion planner is performing.
     *  You can use the functions from Graphics to do the drawing of points,
     *  edges, and so on.
     */
    void Draw(void);

	
protected:
    /**
     *@brief Pointer to simulator, which provides access to robot, obstacles, reward regions
     */
    Simulator  *m_simulator;
    
	const double *bmin;
    const double *bmax;

    
    // 2D VxV array to store distance & connectivity matrix
	double **dist;
	bool **Connect;

	// vector variable to store sampling configuration
	std::vector<double> sample;

	// vector variable to store sampling configuration after applying weight
    std::vector<double> Weight_sample;
	
	
    
	// vector to store the the degree of connection, weight value & accumulated weight summation to apply sampling based on weight probability
	std::vector<double> NConnect;
	std::vector<double> Weight;
	std::vector<double> SumOfWeight;

    
    // vector to store the checkpoint for subdivision collision free check
	std::vector<double> LocalCheckPoint;



	// vector variables for Dijkstra Graph Search 
	std::vector<double> D;   // Shortest Distance array for each node
	std::vector<int> Father;     // Ancestor for each node
	std::vector<int> VisitOrder;  // The order of "being visited" for each node
	std::vector<double> Path;     // The path found from initial point to first region
	std::vector<double> PathTemp;  // The path connecting region to region
	std::vector<bool> VisitS;      // The status of node, showing if being "visited" or not

	

	int Nf; // number of collision-free congfiguration 
    int Nr; // number of reward region
	
	
	int IndexDmin; // Node inde with minimum distance value in Dijkstra method
	int NrLeft;    // Number of remaining unreached rewarding region  
	
	int k;          
    int Ws;        // The index of the selected configureation while applying weighted sampling
	
	double Dmin; // initialized Distance value to begin Dijkstra method 
   
	double Random_WeightSum;  // Random value to decide which configuration to sample in weight sampling
    double WtRange;           // The range factor to "re-sample" the configuration around the selected point

	double rho_dist;          // The distance within which to connect the neighboring configuration
	
	double Resolution;        // the resolution of collision-free checking

    double Divide;            // the degree of division used in subdivision checking method
   
   
    bool Search;              // status for "solving" workflow 
	bool PFound;              // status of "if path found"
	bool CheckLocal;          // status showing if the local edge needs to be checked


    friend class Graphics;    
};

#endif
