#include "MotionPlanner.hpp"
#include <algorithm>
#include <cmath>
#include "Utils.hpp"
#include "Graphics.hpp"
#include <GL/gl.h>
using namespace std;


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
	root->motion = -1;
	root->visit= 0;

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
    /*Vertex *realtime_q  = new Vertex();
	realtime_q->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	m_simulator->GetCurrentState(realtime_q->m_state);
	for (int i=3; i<7; i++)
    printf("FO: s %d = %f \n", i, realtime_q->m_state[i]);
	m_vidGoal=1;  
	*/// real-time state checking
   

    printf("run motion planner for %f seconds\n", tmax);
	GarageState.clear();

	step_speed= 0.2;
	step_steer= 0.1;
    
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
    

	//setting the grid for the whole space
    const double xdim = 30;
	const double ydim = 30;
    const double FullOrient = M_PI;	
	NCell= 30 ;
	Grid_size = round(xdim/NCell);
	//initialize the grid to 0 -> "unoccupied"
	m_config_grid.resize(3*NCell,0);

    //printf("pi= %f", M_PI);
    

int k=1;

while (m_vidGoal!=1)
{
    //printf("size = %d\n", m_vertices.size());
    
	double min_cost=100000; 	
	
    for (int i =0; i< m_vertices.size(); i++)
       {  
		  
		   if(m_vertices[i]->cost <= min_cost && !m_vertices[i]->visit )
		    { 
			  FOindex = i;
	          min_cost = m_vertices[i]->cost;}

	   } 
	 

	m_vertices[FOindex]->visit= 1;
    FOvertex = m_vertices[FOindex];

    // check if the taken configurate is occupied:
	int qxI = round(((m_vertices[FOindex]->m_center[0])+ (0.5*NCell*Grid_size))/Grid_size)-1;
	int qyI = round(((m_vertices[FOindex]->m_center[1])+ (0.5*NCell*Grid_size))/Grid_size)-1;
	//printf("m_center[0]= %f, qxI= %d\n",FOvertex->m_center[0], qxI);
    
	//if (m_config_grid[3*qxI])
	//{}

	residue_error = sqrt(pow((FOvertex->m_state[0]-GarageState[0]),2) + pow((FOvertex->m_state[1]-GarageState[1]),2) +pow((FOvertex->m_state[2]-GarageState[2]),2)) ;
    
	if(residue_error<= 2)
	{m_vidGoal=1;}
	 

 
    //printf("FO: s 0 = %f \n", FOvertex->m_state[0]);
     //printf("min_cost = %f \n ", m_vertices[FOindex]->cost ) ;

	//printf("FOindex= %d, residual error = %f, vidGoal= %d \n", FOindex, residue_error, m_vidGoal);

    
  if (m_vidGoal!= 1)
  {
   
  // generate 3 states from FOstate
	
	// turn left and go back
	/*m_simulator->SetCurrentState(FOvertex->m_state); 
	
	if (m_simulator->SimulateOneStep(-step_speed, -step_steer))
	{ 
      //printf("collision free\n");

	  Vertex *nL_state  = new Vertex();
      nL_state->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	  m_simulator->GetCurrentState(nL_state->m_state);
	  m_simulator->GetCurrentChassisCenter(nL_state->m_center);
	  m_vertices.push_back(nL_state);
	  
	  nL_state->m_parent = FOindex;
	  nL_state->motion = 0; // 0 == Left; 1 == Right; 2 == Straight
	  nL_state->visit= 0;

	  if (m_vertices[nL_state->m_parent]->motion ==1 || m_vertices[nL_state->m_parent]->motion ==2)
	  { nL_state->cost =m_vertices[nL_state->m_parent]->cost + 5;}
	  else
	  { nL_state->cost =m_vertices[nL_state->m_parent]->cost + 1;}

	  //printf("nL: s %d = %f \n",0, nL_state->m_state[0]); 
 
     }*/
     	
	    //for (int i=0; i<2; i++)
		//printf("nL: s %d = %f \n",i, nL_state->m_state[i]); 
   
    // turn right and go back
	m_simulator->SetCurrentState(FOvertex->m_state); // turn back to FO state ;

	if (m_simulator->SimulateOneStep(-step_speed, step_steer))
	{ 
	  //printf("collision free\n");
	  
	  Vertex *nR_state  = new Vertex();
      nR_state->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	  m_simulator->GetCurrentState(nR_state->m_state);
	  m_simulator->GetCurrentChassisCenter(nR_state->m_center);
	  m_vertices.push_back(nR_state);
	  
	  nR_state->m_parent = FOindex;
	  nR_state->motion = 1; // 0 == Left; 1 == Right; 2 == Straight
	  nR_state->visit= 0;

	  if (m_vertices[nR_state->m_parent]->motion ==0 || m_vertices[nR_state->m_parent]->motion ==2)
	  { nR_state->cost =m_vertices[nR_state->m_parent]->cost + 5;}
	  else
	  { nR_state->cost =m_vertices[nR_state->m_parent]->cost + 1;}

	  //printf("nR: s %d = %f \n",0, nR_state->m_state[0]);*/
 
     }

	 //for (int i=0; i<2; i++)
     //printf("nR: s %d = %f \n",i, nR_state->m_state[i]);	

	 

    // go straight back
	
	/*m_simulator->SetCurrentState(FOvertex->m_state); // turn the simulator back to FO state

	if (m_simulator->SimulateOneStep(-step_speed, 0))
	{ 
	  //printf("collision free\n");
	  
	  Vertex *nS_state  = new Vertex();
      nS_state->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	  m_simulator->GetCurrentState(nS_state->m_state);
	  m_simulator->GetCurrentChassisCenter(nS_state->m_center);
	  m_vertices.push_back(nS_state);
	  
	  nS_state->m_parent = FOindex;
	  nS_state->motion = 2; // 0 == Left; 1 == Right; 2 == Straight
	  nS_state->visit= 0; 
      
	  if (m_vertices[nS_state->m_parent]->motion ==0 || m_vertices[nS_state->m_parent]->motion ==1)
	  { nS_state->cost =m_vertices[nS_state->m_parent]->cost + 5;}
	  else
	  { nS_state->cost =m_vertices[nS_state->m_parent]->cost + 1;}
      
	 // printf("nS: s %d = %f \n",0, nS_state->m_state[0]); 
     } */

	      //for (int i=0; i<3; i++)
		  //printf("nS: s %d = %f \n",i, nS_state->m_state[i]);
	      //printf("nS: s %d = %f \n",7, nS_state->m_state[7]);
	
    
	
	
     

	/*for (int i=0; i<3; i++)
		printf("before sorting : %d th s0 = %f \n",i, m_vertices[i]->m_state[1]);
	// sort the m_vertices by cost value
	sort(m_vertices.begin(), m_vertices.end(),this->cmp_by_cost);
    //bool k= this->cmp_by_cost(m_vertices[0],m_vertices[1]);
    //printf("k= %d \n ", k);
    for (int i=0; i<3; i++)
		printf("after sorting : %d th s0 = %f \n"i, m_vertices[i]->m_state[1]);*/
    
	k++;
	if (k == 4200)
	{m_vidGoal=1;}

   }// end of "if (m_vidGoal!=1) 
   
     
}// end of while loop

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
    //int vid = m_vidGoal;
    
	int vid = FOindex;
	
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

bool MotionPlanner::cmp_by_cost(Vertex* a, Vertex* b){ return (a->cost < b->cost);}