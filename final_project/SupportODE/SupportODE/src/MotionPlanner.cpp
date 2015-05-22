#include "MotionPlanner.hpp"
#include "Utils.hpp"
#include "Graphics.hpp"
#include <GL/gl.h>

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;

//initialize your data structures/variables here

//add the root vertex based on the initial robot state
    Vertex *root  = new Vertex();
    root->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
    m_simulator->GetCurrentState(root->m_state);
    m_simulator->GetCurrentChassisCenter(root->m_center);
    root->m_parent = -1;
	
	root->cost = 0;
    
    m_vertices.push_back(root);    

//initially, the goal has not yet been reached
    m_vidGoal = -1;    
}

MotionPlanner::~MotionPlanner(void)
{
//delete tree vertices
    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	if(m_vertices[i])
	{
	    if(m_vertices[i]->m_state)
		free(m_vertices[i]->m_state);	    
	    delete m_vertices[i];	    
	}
    
}

void MotionPlanner::Solve(const double tmax)
{
    printf("run motion planner for %f seconds\n", tmax);
    
	// Geom(4) & Geom (5) are the garage walls
    GarageW1= m_simulator->GetGeomID(4);
    GarageW2= m_simulator->GetGeomID(5);
    
    GarageW1pos=dGeomGetPosition (GarageW1);
    GarageW2pos=dGeomGetPosition (GarageW2);
     
    // setting the goal configuration of garage
	for (int i=0; i<3; i++)
	  GarageState.push_back((GarageW1pos[i]+GarageW2pos[i])/2); 
    
	// setting the goal orientation of garage to the initial orientation of robot 
	for (int i=3; i<12; i++)
	GarageState.push_back(m_vertices[0]->m_state[i]);

	//printf("garage position: x= %f, y= %f, z= %f, theta= %f degrees\n",GarageState[0],GarageState[1],GarageState[2],GarageState[3]);
    
    
	// set the First vertex in OPEN to "root", from which to generate 3 successor states
    Vertex *root  = new Vertex();
    root->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
    m_simulator->GetCurrentState(root->m_state);
    m_simulator->GetCurrentChassisCenter(root->m_center);
    root->m_parent = -1;
	
	root->cost = 0;
    
	FOvertex = root;
	//FOvertex = m_vertices[0];
    
	
	for (int i=0; i<12; i++)
	printf("s %d = %f \n",i, FOvertex->m_state[i]);

	//printf("FOvertex: xcenter= %f, ycenter= %f, theta= %f degrees\n",FOvertex->m_center[0],FOvertex->m_center[1], FOvertex
    


	/*nL_state ;
    
	nR_state ;
    
	nS_state ;*/
   
     

/*
 * you can use the functionality provided in Utils.hpp to measure
 * running time, e.g.,
 *  Clock clk;
 *  ClockStart(&clk);
 *  while(m_vidGoal < 0 && ClockElapsed(&clk) < tmax)
 *  {
 *  
 *  }
 * If your algorithm obtains a solution earlier than the maximum time,
 * then you can break out of the while loop (using return or break statement)
 */   
}

void MotionPlanner::GetSolutionPath(std::vector<double> * path)
{
/* Example implementation. Requires m_vidGoal to be set to a nonzero vertex
 */
    if(m_vidGoal < 0)
	return;

//get reverse path from root to m_vidGoal
    std::vector<int> vids;
    int vid = m_vidGoal;
    while(vid >= 0)
    {
	vids.push_back(vid);
	vid = m_vertices[vid]->m_parent;
    }

//push the state vertices into the papth
    path->clear();    
    const int n = m_simulator->GetNrStateDimensions();
    for(int i = vids.size() - 1; i >= 0; --i)
    {
	const double *s = m_vertices[vids[i]]->m_state;
	for(int j = 0; j < n; ++j)
	    path->push_back(s[j]);
    }    
}

void MotionPlanner::Draw(void)
{
//example of drawing the motion planner vertices as points
//corresponding to the chassis center for each vertex

    glColor3d(0, 0, 0);    
    glBegin(GL_POINTS);    
    for(int i = 0; i < (int) m_vertices.size(); ++i)
	glVertex3dv(m_vertices[i]->m_center);
    glEnd();
}
