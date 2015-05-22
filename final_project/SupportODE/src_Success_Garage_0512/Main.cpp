#include "Graphics.hpp"
#include "Utils.hpp"

void AddBoundaries(Simulator *sim, 
		   const double xdim, 
		   const double ydim, 
		   const double zdim,
		   const double thickness)
{
    dGeomID geom;
    
    geom = dCreateBox(sim->GetSpaceID(), xdim, thickness, zdim);
    dGeomSetPosition(geom, 0, -0.5 * ydim, 0.5 * zdim);
    sim->AddGeometryAsObstacle(geom);

    geom = dCreateBox(sim->GetSpaceID(), xdim, thickness, zdim);
    dGeomSetPosition(geom, 0, 0.5 * ydim, 0.5 * zdim);
    sim->AddGeometryAsObstacle(geom);
    
    geom = dCreateBox(sim->GetSpaceID(), thickness, ydim, zdim);
    dGeomSetPosition(geom, -0.5 * xdim, 0, 0.5 * zdim);
    sim->AddGeometryAsObstacle(geom);

    geom = dCreateBox(sim->GetSpaceID(), thickness, ydim, zdim);
    dGeomSetPosition(geom, 0.5 * xdim, 0, 0.5 * zdim);
    sim->AddGeometryAsObstacle(geom);
}

//you should create your own world
void CreateWorld1(Simulator * sim)
{
    const double zdim = 2.0;
    const double xdim = 20;
	const double ydim = 10;

    AddBoundaries(sim, xdim, ydim, zdim, 0.2);

    dGeomID geom;
    dGeomID geom_gara1;
    dGeomID geom_gara2;

	dGeomID geom_lot1;
    dGeomID geom_lot2;

    dQuaternion q; //for rotations
    
	// create a parking garage - 1 st wall
    geom_gara1 = dCreateBox(sim->GetSpaceID(), 3, 0.2, zdim);              //the length of wall = 3  or 2.5 (when the inital car position  
    dGeomSetPosition(geom_gara1, -13.5 + 0.5*(30-xdim), 0.5, 0.5 * zdim);  // is closer: x=-13.75 + 0.5*(30-xdim)
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI * 1);
    dGeomSetQuaternion(geom_gara1, q);
    sim->AddGeometryAsObstacle(geom_gara1);
    
	// creat a pakring garage - 2nd wall
	geom_gara2 = dCreateBox(sim->GetSpaceID(), 3, 0.2, zdim);              // the length of wall = 3  or 2.5 (when the inital car position  
    dGeomSetPosition(geom_gara2, -13.5+ 0.5*(30-xdim), -2, 0.5 * zdim);     // is closer :x=-13.75 + 0.5*(30-xdim)
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI * 1);
    dGeomSetQuaternion(geom_gara2, q);
    sim->AddGeometryAsObstacle(geom_gara2);
    
    // create a parallel parking lot - 1 st wall
    geom_lot1 = dCreateBox(sim->GetSpaceID(), 2, 0.2, zdim);              
    dGeomSetPosition(geom_lot1, -5, -4, 0.5 * zdim);  
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI * 1.5);
    dGeomSetQuaternion(geom_lot1, q);
    sim->AddGeometryAsObstacle(geom_lot1);

    
	// create a parallel parking lot - 2 st wall
    geom_lot1 = dCreateBox(sim->GetSpaceID(), 2, 0.2, zdim);              
    dGeomSetPosition(geom_lot1, -1, -4, 0.5 * zdim);  
    dQFromAxisAndAngle(q, 0, 0, 1, M_PI * 1.5);
    dGeomSetQuaternion(geom_lot1, q);
    sim->AddGeometryAsObstacle(geom_lot1);//*/



    /*geom = dCreateBox(sim->GetSpaceID(), 9.0, 0.2, zdim);
    dGeomSetPosition(geom, 6, -4, 0.5 * zdim);
    dQFromAxisAndAngle(q, 0, 0, 1, -M_PI * 0.25);
    dGeomSetQuaternion(geom, q);
    sim->AddGeometryAsObstacle(geom);

    geom = dCreateSphere(sim->GetSpaceID(), 0.5 * zdim);
    dGeomSetPosition(geom, 0, -4, 0.5 * zdim);
    sim->AddGeometryAsObstacle(geom);//*/
}


int main(int argc, char **argv)
{
    PseudoRandomSeed();
  
    dInitODE();

//create simulator and set robot position at (0, -8)  
    Simulator sim(0, -8);    

//create motion planner
    MotionPlanner mp(&sim);    

//create graphics
    Graphics graphics(&mp);
    
//create obstacles and terrain
    CreateWorld1(&sim);
    
//if argument specified, set as maximum motion planning time
    if(argc > 1)
	graphics.SetMotionPlannerMaxTime(atof(argv[1]));    

//print help messages
    graphics.HandleEventOnHelp();

//enter event loop
    graphics.MainLoop();
    
    return 0;    
}
