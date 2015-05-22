#include "BugAlgorithms.hpp"
#include <stdio.h>
#include <math.h>


BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator; 
	
	// initialization of parameters, including the initial distance from robot to goal, the initial position of robot and goal, and the sensor radius 
    DistRGL=m_simulator->GetDistanceFromRobotToGoal(); 
	iRCx = m_simulator->GetRobotCenterX();
	iRCy = m_simulator->GetRobotCenterY();			
	GCx=  m_simulator->GetGoalCenterX();
    GCy=  m_simulator->GetGoalCenterY();
	SeR = m_simulator->GetSensorRadius();
    
	// initial value of hit point position
	xH=0;
	yH=0;
	
	// For Bug 1, initial value of Obstacle circumferrence, and the distance to goal at closest point on the obstacle boundary  
	Circumferrence=0;
	Search_path=0;
	
	// default value of Bug's status boolean logic
	search =0 ;
    hit = 0 ;


	// changeable parameters, including sensor's sensible range factor, straight line moving speed, and the m-line equation residue error for Bug2
   
	LineSpeedFactor=21;
    // Speed factor for straight line speed toward the goal

	ObSensitivity= 0.8;
	//The sensitivity of sensor to detect obstacle, 0<sensitivity<=1                      
    
	LineEqnResidue = 0.05;
    // the residule error after plugging robot position into m-line equation, to determine if the robot is on the m-line
	
    
    //add your initialization of other variables
    //that you might have declared
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
    //free/delete other memory that you have allocated from the heap
}

void BugAlgorithms::Restart(void)
{   
	// Re-initialize the parameters if user changes the parameters 
	DistRGL=m_simulator->GetDistanceFromRobotToGoal(); 
	iRCx = m_simulator->GetRobotCenterX();
	iRCy = m_simulator->GetRobotCenterY();			
	GCx=  m_simulator->GetGoalCenterX();
    GCy=  m_simulator->GetGoalCenterY();
	SeR = m_simulator->GetSensorRadius();
    
	// initial value of hit point position
	xH=0;
	yH=0;
	
	// For Bug 1, initial value of Obstacle circumferrence, and the distance to goal at closest point on the obstacle boundary  
	Circumferrence=0;
	Search_path=0;
	
	// initial value of Bug's status boolean logic
	search =0 ;
    hit = 0 ;
	
}

void BugAlgorithms::Bug1Strategy(double &dx, double &dy)
{   
	
	DminToObs = m_simulator->TakeSensorReading(xmin,ymin);	
    // get sensor reading for distance to obstacle and the nearest point position on the boundary
	
	
	printf("search= %i, hit= %i, xL= %lf, Search_path=%lf, Circumferrence=%lf \n", search,hit, xL, Search_path, Circumferrence);
	// for debugging use, display the status, leave point position, and accumulated circumferrence
	
    //status 1: if hit= faulse, search =faulse => move straight toward the goal until hitting the obstacle
    if (!hit && !search)
	{
		StepMoveStraightTowardGoal(dx, dy);

	     if (DminToObs <= ObSensitivity*SeR) // when hitting the obstacle,   
	     {
		   hit =1;
	       search=1;          
	       circle =0;        // set hit=true, search = true, circle = 0
	       Circumferrence=0; // initialization of obstacle circumferrence
	       Search_path=0;    // initialization of the path length from hitting point to the searched closest point 
	       xH=xmin;           
           yH=ymin;          // set the (xmin,ymin) as the hitting point when the obstacle is first detected
	  
	  }

	}
    
    // status 2: when the status of search & hit = true, begin to follow the boundary to search for the leave point of shortest disdance to goal:
	else if (search && hit)
	{
          
	   StepFollowObstacleBoundary(dx, dy); // follow the boundary
       Circumferrence = Circumferrence + sqrt(pow(dx,2) + pow(dy,2)); // accumulate each step length to caculate the boundary circumferrence    
       DistRGonBoundary = m_simulator->GetDistanceFromRobotToGoal();  // update the distance to goal during the search process
	
	   DminToObs = m_simulator->TakeSensorReading(xmin,ymin);  
		  
	      if (circle && (fabs(xmin-xH)<=0.05 && fabs(ymin-yH)<=0.05)) // If the bug circles back to the hitting point, |x-xH|< acceptable error && |y-yH|< acceptable error,
			                                                          // terminate the search process, set search= false
		  {
			 search = 0; 

		  }
	   
	   
	     if (DistRGonBoundary < DistRGL && !(DminToObs==SeR)) // If the updated distance to goal on the boundary is shorter than the current closest point,    
		 {                                                    
		  DistRGL= DistRGonBoundary;
		  Search_path=Circumferrence;                         // replace the closest pointthe with the updated value, 

          xL=xmin;
		  yL=ymin;                                            // and record the updated location (xmin,ymin) as the leave point
 
		  }  
          
		  circle = 1; // until the bug circles back to the hit point (finishes a complete circle), set the status circle = true

	}
	
    // status 3: if the bug complete a circle and finishes the searching for the closest point, 
	//           still follow the boundary but be ready to leave the obstacle at recorded leaving point   
	else if (hit && !search)
    { 
	   
	   StepFollowObstacleBoundary(dx, dy);

        if (fabs(xmin-xL)<=0.05 && fabs(ymin-yL)<=0.05) //If the bug re-meets the leaving point: |x-xL|< acceptable error && |y-yL|< acceptable error,
	    {
	    hit = 0;                                       // set hit = false to go back to case 1, and leave the obstacle in the direction of goal
	    }
	}
	
	//add your implementation
}

void BugAlgorithms::Bug2Strategy(double &dx, double &dy)
{
    
	DminToObs = m_simulator->TakeSensorReading(xmin,ymin);	
	// get sensor reading for distance to obstacle and the nearest point position on the boundary

	printf("search= %i, xL= %lf,xmin=%lf, eqn=%f \n", search, xL, xmin, fabs(LineEqn));	
	// for debugging use, display the status, leave point, and residue error of m-line equation

    //status 1: if search == faulse => move straight toward the goal until hitting the obstacle
	if (!search)  
	{
		StepMoveStraightTowardGoal(dx, dy);

	   if (DminToObs < ObSensitivity*SeR)  // when hitting the obstacle, 
	   {	     
		   search=1;                       //set search= true, and set the (xmin,ymin) as the hitting point when the obstacle is first detected
		   xH=xmin;
		   yH=ymin;  
	  }

	}
    
	//status 2: after hitting the obstacle, follow the boundary until meeting the m-line intersected point
	else
	{

	   StepFollowObstacleBoundary(dx, dy);
	   
	   DminToObs = m_simulator->TakeSensorReading(xmin,ymin);
	   // get sensor reading for distance to obstacle and the nearest point position on the boundary

	   LineEqn = (RCy-iRCy)-((GCy-iRCy)/(GCx-iRCx)*(RCx-iRCx)); 
	   // plug the Robot Center Position (RCx, RCy) into the  m-line equation: (y-iRCx)- (GCy-IRCy)/(GCx-iRCx)* (x-iRCx))=0, 
	   // which is  determined by Goal & initial Robot position, 
	                                                              	      
	    if (fabs(LineEqn) < LineEqnResidue)                   // check if the reached location is withing the acceptable range of m-line equation: 
	                                                          // | m-line(RCx, RCy) | < residue error of m-line equation.
		  {
		   xL=xmin;                                           // set (xmin,ymin) as leave point, and set search = false to go back to status 1,
		   yL=ymin;                                           // and leave the obstacle 
		   search = 0;
		  }	   
	     
	}
	
	//add your implementation
}


void BugAlgorithms::MyBugStrategy(double &dx, double &dy)
{
   DminToObs = m_simulator->TakeSensorReading(xmin,ymin);	
   // get sensor reading for distance to obstacle and the nearest point position on the boundary

   printf("search= %i, DistRGH= %lf \n", search, DistRGH);// for debugging use, display the search status and updated distance to goal on the boundary	
   
   //status 1: if search == faulse => move straight toward the goal until hitting the obstacle
   if (!search)
	{
		StepMoveStraightTowardGoal(dx, dy);

		if (DminToObs < ObSensitivity*SeR) // when hitting the obstacle,
	   {
		   search=1;                       // set search= true
		   
		   DistRGH = m_simulator->GetDistanceFromRobotToGoal(); // record the distance to goal at hitting point on the boundary    
		   xH=xmin;
		   yH=ymin;                                             // set the (xmin,ymin) as the hitting point when the obstacle is first detected
	   }
	}
    
   //status 2: follow the boundary until finding a point of shorter diatnce to goal than that at hitting point
	else
	{
	   
	   StepFollowObstacleBoundary(dx, dy);
       
	   DistRGonBoundary = m_simulator->GetDistanceFromRobotToGoal(); 
	   
	      if (DistRGonBoundary < DistRGH)  // check if the distance to goal at current point is shorter than that at hitting point 
		  {
		   search = 0;                     // if it is, set search = false to go back to status 1
		  }	   
	     
	}

    //add your implementation
}

void BugAlgorithms::StepMoveStraightTowardGoal(double &dx, double &dy)
{
	DminToObs = m_simulator->TakeSensorReading(xmin,ymin);
	// get sensor reading for distance to obstacle and nearest point on the boundary
	
	RCx = m_simulator->GetRobotCenterX(); // updating the position of robot center
	RCy = m_simulator->GetRobotCenterY();
    

	if (DminToObs >= ObSensitivity*SeR)  // If the detected distance to obstacle is larger than or equal to "sensitivity factor * radius of sensor", 
	{                                    // no obstacle is detected, and the robot can move straight toward the goal; otherwise, it should stop moving
	
	dx= (GCx-RCx)/(LineSpeedFactor*sqrt((GCx-RCx)*(GCx-RCx) +(GCy-RCy)*(GCy-RCy))); // the robot's step vector determined by 
	dy= (GCy-RCy)/(LineSpeedFactor*sqrt((GCx-RCx)*(GCx-RCx) +(GCy-RCy)*(GCy-RCy))); // vector : v(r) = (goal(r)-robot(r)) / LineSpeedFactor * || goal(r)-robot(r) ||  
	  
	}

	//add your implementation
}

void BugAlgorithms::StepFollowObstacleBoundary(double &dx, double &dy)
{
    DminToObs = m_simulator->TakeSensorReading(xmin,ymin);
    // get sensor reading for distance to obstacle and nearest point on the boundary
		
	RCx = m_simulator->GetRobotCenterX();  // updating the position of robot center
	RCy = m_simulator->GetRobotCenterY();
	
	
	if (DminToObs < SeR)     // If the detected distance to obstacle is less than radius of sensor,                                     
	{                        // obstacle is detected, robot should move in the "tagnent direction" relative to the contact point on the obstacle
	 contactx = xmin;        
     contacty = ymin;        // updating contact point as (xmin,ymin)
	 dx=(contacty-RCy)/11;  
	 dy=-(contactx-RCx)/11;  // compute the tangent direction ( orthogonal to the vector between robot center and contact point )
    
	  if (!search && (Circumferrence-Search_path < Search_path) ) // for Bug 1 only, after complete a circle, take shorter route back to the closest leaving point found
	  {dx=-dx; 
	   dy=-dy;}

	}
	
	else                    // when moving circularly at certain sharp corner of obstacle, the obstacle might temporarily be out of sensing range,
	{                       // and then a centripetal vector (parallel to the vector between robot center and contact point) should be applied to robot, 
	 dx=(contactx-RCx)/200; // so that robot can always be pulled close to the obstacle while following the boundary.        
	 dy=(contacty-RCy)/200;	         
	}

	
	//add your implementation
}

