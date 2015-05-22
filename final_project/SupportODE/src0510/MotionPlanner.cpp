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
	
	//setting the grid for the whole space
    xdim = 20;
	ydim = 10;
    FullOrient = 2*M_PI;	
	NCellx= 1500 ;    //1500
	NCelly= 750 ;   
    NAngle = 200;    //200 150
	Grid_size_x = xdim/NCellx;
    Grid_size_y = ydim/NCelly;
	Grid_size_angle = FullOrient/NAngle;

	//initialize the grid to 0 -> "unoccupied
	// building d x d x d grid matrix
    //m_config_grid=NULL; 
    m_config_grid= new int**[NCellx];
    for (int k=0;k<NCellx;k++)
	{m_config_grid[k] = new int*[NCelly];}

	for (int i=0;i<NCellx;i++)
	{for (int j=0;j<NCelly;j++)
	  {m_config_grid[i][j] = new int[NAngle];
	   for (int k=0; k <NAngle; k++)
	   m_config_grid[i][j][k]=0;
	  } 
	}//*/




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

	step_speed= 35; // 35
	step_steer= 0.2; //0.2,0.3, 0.5
    
	// Geom(4) & Geom (5) are the garage walls
    GarageW1= m_simulator->GetGeomID(4);
    GarageW2= m_simulator->GetGeomID(5);

	GarageW1pos=dGeomGetPosition (GarageW1);
    GarageW2pos=dGeomGetPosition (GarageW2);

	// Geom(6) & Geom (7) are the parallel parking walls
	ParkingLotW1= m_simulator->GetGeomID(6);
    ParkingLotW2= m_simulator->GetGeomID(7);
    
    ParkingLotW1pos=dGeomGetPosition (ParkingLotW1);
    ParkingLotW2pos=dGeomGetPosition (ParkingLotW2);//*/
     
    // setting the goal configuration of garage and parking lot
	for (int i=0; i<3; i++)
	{GarageState.push_back((GarageW1pos[i]+GarageW2pos[i])/2); 
	 ParkingLotState.push_back((ParkingLotW1pos[i]+ParkingLotW2pos[i])/2);
	} 
      
	//printf("Parking lot x=%f, y=%f", ParkingLotState[0],ParkingLotState[1])
    
	// setting the goal orientation of garage & parking lot to the initial orientation of robot 
	for (int i=3; i<13; i++)
	{GarageState.push_back(m_vertices[0]->m_state[i]);
	 ParkingLotState.push_back(m_vertices[0]->m_state[i]);
	 }

	//printf("garage position: x= %f, y= %f, z= %f, theta= %f degrees\n",GarageState[0],GarageState[1],GarageState[2],GarageState[3]);
    


    
  //********************************** beginning of search algorithm

int k=1;

while (m_vidGoal!=1)
{
    //printf("size = %d\n", m_vertices.size());
    
	double min_cost=10000000; 	
	
    for (int i =0; i< m_vertices.size(); i++)
       {  
		  
		   if((m_vertices[i]->cost <= min_cost) && (m_vertices[i]->visit==0) )
		    { 
			  FOindex = i;
	          min_cost = m_vertices[i]->cost;}

	   } 
	 

	m_vertices[FOindex]->visit= 1;
    FOvertex = m_vertices[FOindex];

    // check if the taken configuration is occupied:
	int qxI = round(((m_vertices[FOindex]->m_state[0])+ (0.5*NCellx*Grid_size_x))/Grid_size_x)-1;
	int qyI = round(((m_vertices[FOindex]->m_state[1])+ (0.5*NCelly*Grid_size_y))/Grid_size_y)-1;
	int qtheI =  round((2*asin(m_vertices[FOindex]->m_state[6]) + (0.5*NAngle*Grid_size_angle))/Grid_size_angle)-1;
	
	if (qtheI==-1)
	{qtheI=0;}

	//printf("q6= %f, theta = %f, qtheI = %d \n",m_vertices[FOindex]->m_state[6], 2*asin(m_vertices[FOindex]->m_state[6]), qtheI);
	//printf("theta = pi, qtheI= %d\n", round((M_PI + (0.5*NAngle*Grid_size_angle))/Grid_size_angle)-1);
    //printf("theta = -0.99pi, qtheI= %d\n", round((-0.99*M_PI + (0.5*NAngle*Grid_size_angle))/Grid_size_angle)-1);


	//printf("m_center[1]= %f, qyI= %d\n",FOvertex->m_center[1], qyI);
    
	
	//residue_error = sqrt(pow((FOvertex->m_state[0]-GarageState[0]),2) + pow((FOvertex->m_state[1]-GarageState[1]),2) +pow((FOvertex->m_state[2]-GarageState[2]),2)) ;
    residue_error = sqrt(pow((FOvertex->m_state[0]-ParkingLotState[0]),2) + pow((FOvertex->m_state[1]-ParkingLotState[1]),2) +pow((FOvertex->m_state[2]-ParkingLotState[2]),2));


	if(residue_error<= 1.5) //1.15,0.8
	{m_vidGoal=1;}

    
 
    //printf("FOindex= %d , x = %f, tree size = %d\n",FOindex, FOvertex->m_state[0],m_vertices.size());
     //printf("min_cost = %f \n ", m_vertices[FOindex]->cost ) ;

	
    printf("FOindex= %d, residual error = %f, size of T = %d \n", FOindex, residue_error, m_vertices.size());
    //printf("qxI=%d, qyI=%d\n",qxI,qyI);


  // if the taken configuration is not "occupied", generate 3 successor configurations, and set it to be "occupied"    
 if ( m_config_grid[qxI][qyI][qtheI]==0 && m_vidGoal!= 1)
  {
     //printf("m_config_grid[qxI][qyI][qtheI]= %d\n ", m_config_grid[qxI][qyI][qtheI]);
     

     m_config_grid[qxI][qyI][qtheI]=1;
    
	 // generate 3 states from FOstate
	 
    // turn left and go back
	m_simulator->SetCurrentState(FOvertex->m_state); 
	
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

	  if (m_vertices[nL_state->m_parent]->motion == 0)
	  { nL_state->cost =m_vertices[nL_state->m_parent]->cost + 1;}
	  else if (m_vertices[nL_state->m_parent]->motion == 1 || m_vertices[nL_state->m_parent]->motion == 2)
	  { nL_state->cost =m_vertices[nL_state->m_parent]->cost + 5;}
      else 
	  { //nL_state->cost =m_vertices[nL_state->m_parent]->cost + 5;
        nL_state->cost =m_vertices[nL_state->m_parent]->cost + 10;
	  }
      
    
	  //printf("nL: s %d = %f \n",1, nL_state->m_state[1]); 
 
     } 
     else 
	 {printf("in collision \n");} 	//*/
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

	  if (m_vertices[nR_state->m_parent]->motion ==1)
	  { nR_state->cost =m_vertices[nR_state->m_parent]->cost + 1;}
	  else if (m_vertices[nR_state->m_parent]->motion ==0 ||m_vertices[nR_state->m_parent]->motion ==2)
	  { nR_state->cost =m_vertices[nR_state->m_parent]->cost + 5;}
	  else 
      { //nR_state->cost =m_vertices[nR_state->m_parent]->cost + 5;
		nR_state->cost =m_vertices[nR_state->m_parent]->cost + 10;
	  }

	  //printf("nR: s %d = %f \n",1, nR_state->m_state[1]);
 
     } 
     else 
	{printf("in collision");} //*/
	 
	//for (int i=0; i<2; i++)
     //printf("nR: s %d = %f \n",i, nR_state->m_state[i]);	

    // go straight back
	m_simulator->SetCurrentState(FOvertex->m_state); // turn the simulator back to FO state

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
      
	  if (m_vertices[nS_state->m_parent]->motion ==2)
	  { nS_state->cost =m_vertices[nS_state->m_parent]->cost + 1;}
	  else if (m_vertices[nS_state->m_parent]->motion ==0 ||m_vertices[nS_state->m_parent]->motion ==1) 
	  { nS_state->cost =m_vertices[nS_state->m_parent]->cost + 5;}
	  else 
	  { //nS_state->cost =m_vertices[nS_state->m_parent]->cost + 5;
        nS_state->cost =m_vertices[nS_state->m_parent]->cost + 10;
	   }

      
	   //printf("nS: s %d = %f \n",0, nS_state->m_state[0]); 
      } 
	else 
	{printf("in collision");} //*/
   

    	
    m_simulator->SetCurrentState(FOvertex->m_state); // turn the simulator back to FO state
	
//*************************************************************************************************
     
    
	 // generate 3 more "forward" states from FOstate
	 
    // turn left and go forward
	m_simulator->SetCurrentState(FOvertex->m_state); 
	
	if (m_simulator->SimulateOneStep(0.5*step_speed, -step_steer))
	{ 
      //printf("collision free\n");

	  Vertex *pL_state  = new Vertex();
      pL_state->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	  m_simulator->GetCurrentState(pL_state->m_state);
	  m_simulator->GetCurrentChassisCenter(pL_state->m_center);
	  m_vertices.push_back(pL_state);
	  
	  pL_state->m_parent = FOindex;
	  pL_state->motion = 3; // back : {0 == Left; 1 == Right; 2 == Straight}, forward : {3 == Left; 4 == Right; 5 == Straight}
	  pL_state->visit= 0;

	  if (m_vertices[pL_state->m_parent]->motion ==3)
	  { pL_state->cost =m_vertices[pL_state->m_parent]->cost + 2;}
	  else if (m_vertices[pL_state->m_parent]->motion ==4 || m_vertices[pL_state->m_parent]->motion ==5)
	  { pL_state->cost =m_vertices[pL_state->m_parent]->cost + 5;}
	  else 
	  { //pL_state->cost =m_vertices[pL_state->m_parent]->cost + 5;
	    pL_state->cost =m_vertices[pL_state->m_parent]->cost + 10;
	  }
 
     } 
     else 
	 {printf("in collision \n");} 	//*/
    
	// turn right and go forward
	m_simulator->SetCurrentState(FOvertex->m_state); // turn back to FO state ;

	if (m_simulator->SimulateOneStep(0.5*step_speed, step_steer))
	{ 
	  //printf("collision free\n");
	  
	  Vertex *pR_state  = new Vertex();
      pR_state->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	  m_simulator->GetCurrentState(pR_state->m_state);
	  m_simulator->GetCurrentChassisCenter(pR_state->m_center);
	  m_vertices.push_back(pR_state);
	  
	  pR_state->m_parent = FOindex;
	  pR_state->motion = 4; //  back : {0 == Left; 1 == Right; 2 == Straight}, forward : {3 == Left; 4 == Right; 5 == Straight}
	  pR_state->visit= 0;

	  if (m_vertices[pR_state->m_parent]->motion ==4)
	  { pR_state->cost =m_vertices[pR_state->m_parent]->cost + 2;}
	  else if (m_vertices[pR_state->m_parent]->motion ==3 ||m_vertices[pR_state->m_parent]->motion ==5)
	  { pR_state->cost =m_vertices[pR_state->m_parent]->cost + 5;}
	  else 
      { //pR_state->cost =m_vertices[pR_state->m_parent]->cost + 5;
	    pR_state->cost =m_vertices[pR_state->m_parent]->cost + 10;
	  }
 
     } 
     else 
	{printf("in collision");} //*/


    // go straight forward
	m_simulator->SetCurrentState(FOvertex->m_state); // turn the simulator back to FO state

	if (m_simulator->SimulateOneStep(0.5*step_speed, 0))
	{ 
	  
	  Vertex *pS_state  = new Vertex();
      pS_state->m_state = (double *) calloc(m_simulator->GetNrStateDimensions(), sizeof(double));
	  m_simulator->GetCurrentState(pS_state->m_state);
	  m_simulator->GetCurrentChassisCenter(pS_state->m_center);
	  m_vertices.push_back(pS_state);
	  
	  pS_state->m_parent = FOindex;
	  pS_state->motion = 5; // back : {0 == Left; 1 == Right; 2 == Straight}, forward : {3 == Left; 4 == Right; 5 == Straight}
	  pS_state->visit= 0; 
      
	  if (m_vertices[pS_state->m_parent]->motion ==5)
	  { pS_state->cost =m_vertices[pS_state->m_parent]->cost + 2;}
	  else if (m_vertices[pS_state->m_parent]->motion == 3 || m_vertices[pS_state->m_parent]->motion == 4)
	  { pS_state->cost =m_vertices[pS_state->m_parent]->cost + 5;}
	  else 
      {   //pS_state->cost =m_vertices[pS_state->m_parent]->cost + 5;
         pS_state->cost =m_vertices[pS_state->m_parent]->cost + 10;
	  }

     } 
	else 
	{printf("in collision");} //*/
    	
    m_simulator->SetCurrentState(FOvertex->m_state); // turn the simulator back to FO state
    


   }// end of "if  m_config_grid[qxI][qyI][qtheI]==0 && m_vidGoal!= 1) 
   
   k++;
	if (k== 20000 )//|| m_vertices.size()>= 17942 ) // 1477,14173,15107,1181
   	{m_vidGoal=1;}  //*/

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
    double path_length= 0;
	
	path->clear();    
    const int n = m_simulator->GetNrStateDimensions();
    for(int i = vids.size() - 1; i >= 0; --i)
    {
	const double *s = m_vertices[vids[i]]->m_state;
	 for(int j = 0; j < n; ++j)
	 {path->push_back(s[j]);}
     
	 if (i>0)
     path_length= path_length + sqrt(pow(m_vertices[vids[i]]->m_state[0] -m_vertices[vids[i-1]]->m_state[0],2) +pow(m_vertices[vids[i]]->m_state[1] -m_vertices[vids[i-1]]->m_state[1],2));

    }    

    double straight_distance =  sqrt(pow((m_vertices[vids[vids.size() - 1]]->m_state[0]-m_vertices[vids[0]]->m_state[0]),2) + pow((m_vertices[vids[vids.size() - 1]]->m_state[1]-m_vertices[vids[0]]->m_state[1]),2));

	printf("striaght distance= %f, path length = %f \n", straight_distance,path_length);
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