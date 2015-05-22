#include "MotionPlanner.hpp"
#include "Graphics.hpp"
#include <iostream>
#include <algorithm>
using namespace std;

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
      
	m_simulator = simulator;
   
	//initialize your data structures/variables here

	double tmax=120; // set the maximum running time for solving the path

	Nf=1000;// set the total number of sampling configuration for roadmap
	Nr=m_simulator->GetNrRewardRegions(); // get the number of reward regions
 
     
	sample.resize(3 * (Nf+Nr+1));  // resize the sample point vector with the size = (inital point + sampling configuration + rewarding regions)  
	
	// resize the vector of Weighted Sample, Connectivity, Weight with the size equal to the number of sampling configuration
	Weight_sample.resize(3 * Nf); 
	NConnect.resize(Nf,0);         
	Weight.resize(Nf);            
	SumOfWeight.resize(Nf+1,0);

    // the parameters used for building roammap
	rho_dist= 8 ;     // the radius of distance to connect the neighboring node
	Resolution= 0.8;  // the resolution of edge for collision check
	WtRange=1.5;      // the range around the selected configuration to get weighted sample
   	
	LocalCheckPoint.resize(3); // point used for local path collision checking
    
   	// building VxV distance matrix
    dist=NULL; 
    dist= new double*[Nf+Nr+1];
    for (int k=0;k<Nf+Nr+1;k++)
	dist[k] = new double[Nf+Nr+1];
    
	// building VxV connectivity matrix
	Connect=NULL;
    Connect= new bool*[Nf+Nr+1];
    for (int k=0;k<Nf+Nr+1;k++)
	Connect[k] = new bool[Nf+Nr+1];

    // parameters used for Dijkstra graph search
	D.resize(Nf+Nr+1, 10000);  // the Distance 1-D array recording the distance of each node
	Father.resize(Nf+Nr+1,-1);   // the array recording each node's ancestor 
	VisitS.resize(Nf+Nr+1,0);    // the array recording if the node has been "visited"

}

MotionPlanner::~MotionPlanner(void)
{
}

void MotionPlanner::Solve(const double tmax)
{
    printf("run motion planner for %f seconds\n", tmax);
	
	// Get the boudary value of the scene to generate the uniform random samples
	bmin = m_simulator->GetBoundingBoxMin();
	bmax = m_simulator->GetBoundingBoxMax();
	//printf("bmin= %f, bmax= %f", *bmin, *bmax);
    
	// put initial point and the configuration of all reward points into the sampling configuration vector
    // sample [0] represents the initial point configuration

	sample[0] = m_simulator->GetRobotAngleInRadians();
	sample[1] = m_simulator->GetRobotCenterX(); 
	sample[2] = m_simulator->GetRobotCenterY();

    // sample[1] to sample [Nr] represent the configuration of rewarding regions		
	for (int i=1; i< Nr+1; i++)
	{
	sample[3*i] = 0;//m_simulator->GetRewardRegionRadius(i-1);
	sample[3*i+1] = m_simulator->GetRewardRegionCenterX(i-1);
	sample[3*i+2] = m_simulator->GetRewardRegionCenterY(i-1);	
	}   
	

// calculate the time used to solve the path
Clock clk;
ClockStart(&clk);

// Search = true - > Keep Solving the path
Search = 1;

// Begining of the path solving loop------------------
while (Search)
{
	// re-initialize the parameters including Connectivity array, 
	Connect=NULL;
    Connect= new bool*[Nf+Nr+1];
    for (int k=0;k<Nf+Nr+1;k++)
	Connect[k] = new bool[Nf+Nr+1];
    
	// clear and initialize the vectors variables used for weighted sampling 
	NConnect.clear();
	NConnect.resize(Nf,0);
    
	SumOfWeight.clear();
	SumOfWeight.resize(Nf+1,0);

    // clear and the path found until best path is solved
	Path.clear();
	VisitOrder.clear();

// Pre-sampling the configuration using uniform distribution, 
// to determine weight of each local region: --------------------
		
	for (int i=Nr+1;i<Nf+Nr+1;i++)
	{ 
	
	  sample[3*i] = PseudoRandomUniformReal(-3.14159265, 3.14159265);
	  sample[3*i+1] = PseudoRandomUniformReal(bmin[0], bmax[0]);
	  sample[3*i+2] = PseudoRandomUniformReal(bmin[1], bmax[1]);
      
	  // check if the robot is collisiion when placed at the sampled point 
	   m_simulator->SetRobotOrientationAndPosition(sample[3*i],sample[3*i+1],sample[3*i+2]);
	   if (m_simulator->IsRobotInCollision())
	   {i--;}
	 
	 }
    // put robot back to initial configuration after collision free check for each sampled configuration
	m_simulator->SetRobotOrientationAndPosition(sample[0],sample[1],sample[2]);

    //Construct VxV distance arry (for pre-sampled configuration)
	for (int i=0; i <Nf+Nr+1; i++)
	{
	  for (int j=0; j <i+1; j++ )
	  { 
	      // compute the distance between each configuration pair
		  dist[i][j]= 0.0001*fabs(sample[3*i]-sample[3*j]) + sqrt(pow((sample[3*i+1]-sample[3*j+1]),2) + pow((sample[3*i+2]-sample[3*j+2]),2));
		  dist[j][i]= dist[i][j] ;
        
	   if (dist[i][j]<=rho_dist) // if the distance shorter than the set radius , connect the configuration and check collision free 
	   {  
	      // set connectivity presumably true before collision free checking
		  Connect[i][j]=1;
          Connect[j][i]=1; 
		   
          //Check if the local path is collision free by subdivision method 
		   Divide=2; 
           CheckLocal=1;
          while ( (dist[i][j]/Divide >= Resolution) && (CheckLocal))
		  { 
			 for (int k=1; k < Divide; k=k+2) 
			 {
		     LocalCheckPoint[0]= ((1-(k/Divide))*sample[3*i]) + (k/Divide*sample[3*j]);
			 LocalCheckPoint[1]= ((1-(k/Divide))*sample[3*i+1]) + (k/Divide*sample[3*j+1]);
             LocalCheckPoint[2]= ((1-(k/Divide))*sample[3*i+2]) + (k/Divide*sample[3*j+2]);			 
			 
			 // set the robot position back to initial value after checking
			 m_simulator->SetRobotOrientationAndPosition(LocalCheckPoint[0],LocalCheckPoint[1],LocalCheckPoint[2]);
	         
			 // if the edge is in collision with obstacle, set the connectivity to false 		
			    if (m_simulator->IsRobotInCollision())
	            {Connect[i][j]=0;
			     Connect[j][i]=0;
				 CheckLocal= 0;
			     break;}	 			  
			 }
             
			 Divide = Divide*2; // subdivide the edge segment  			 
		   } 
       
	   }
	   else 
	   {
		Connect[i][j]=0;  // if the distance is longer than "rho_dist", set connectivity as false
        Connect[j][i]=0;
	   }	    
	    
	 }  

	}// End of connecting the edge in pre-sampled stage based on uniform distribution
    

// Sampling based on weight-----------------

// compute the degree of connectivity of each sample 
	for (int i=0; i <Nf; i++)
	{ 
	  for (int j=0; j <Nf+Nr+1; j++)
	  {NConnect[i] = NConnect[i] + Connect[i+Nr+1][j];
	  }
	  
	  Weight[i]= 1/NConnect[i];//take reciprocal of degree as weight
	  
	  SumOfWeight[i+1]= SumOfWeight[i]+Weight[i]; // accumulate the sum of weight 
     
	}
   
// re-sampling according to weight	
	
	for (int i=0;i<Nf;i++)
	{ 
	  // generate a random number between (0, sum of all weight value)
	  Random_WeightSum= PseudoRandomUniformReal(0, SumOfWeight[Nf]);
      
	  for (int j=1;j<Nf+1;j++)
	  {   
		  // based on the random number, locate the accumulated summation it belongs, 
		  // so that the sample will be selected with the probability in proportion to weight 
    	  if ( (Random_WeightSum > SumOfWeight[j-1]) && (Random_WeightSum <= SumOfWeight[j]) )
		  { Ws= j-1;		     			
		    break;}
	  }
      
	  // generate local sample around the selected configuration
	  Weight_sample[3*i] = PseudoRandomUniformReal(sample[3*(Ws+Nr+1)]-WtRange*Resolution, sample[3*(Ws+Nr+1)]+WtRange*Resolution);   
	  Weight_sample[3*i+1] = PseudoRandomUniformReal(sample[3*(Ws+Nr+1)+1]-WtRange*Resolution, sample[3*(Ws+Nr+1)+1]+WtRange*Resolution);
	  Weight_sample[3*i+2] = PseudoRandomUniformReal(sample[3*(Ws+Nr+1)+2]-WtRange*Resolution, sample[3*(Ws+Nr+1)+2]+WtRange*Resolution);
      

	  // check if the robot is collisiion when placed at the locally generated configuration 
	   m_simulator->SetRobotOrientationAndPosition(Weight_sample[3*i],Weight_sample[3*i+1],Weight_sample[3*i+2]);
	   if (m_simulator->IsRobotInCollision())
	   {i--;}
	      
	 }
    

    // replace the pre-sampled configuration with the "weighted sampled configuration"
	for (int i=Nr+1;i<Nf+Nr+1;i++)
	{
	    sample[3*i]=Weight_sample[3*(i-Nr-1)];
        sample[3*i+1]=Weight_sample[3*(i-Nr-1)+1];
		sample[3*i+2]=Weight_sample[3*(i-Nr-1)+2];
	}


// re-initialize the connectivity array for sampled configuration based on weighting 
	Connect=NULL;
    Connect= new bool*[Nf+Nr+1];
    for (int k=0;k<Nf+Nr+1;k++)
	Connect[k] = new bool[Nf+Nr+1];
    
//Re-Construct VxV distance arry & connectivity array
for (int i=0; i <Nf+Nr+1; i++)
	{
	  for (int j=0; j <i+1; j++ )
	  { 
	       // compute the distance between each configuration pair
		  dist[i][j]= 0.0001*fabs(sample[3*i]-sample[3*j]) + sqrt(pow((sample[3*i+1]-sample[3*j+1]),2) + pow((sample[3*i+2]-sample[3*j+2]),2));
		  dist[j][i]= dist[i][j] ;
        
	   if (dist[i][j]<=rho_dist)
	   {  
	   
		  Connect[i][j]=1;
          Connect[j][i]=1; 
		  
		  //Check if the local path is collision free by subdivision method  
		   Divide=2; 
           CheckLocal=1;

          while ( (dist[i][j]/Divide >= Resolution) && (CheckLocal))
		  { 
			 for (int k=1; k < Divide; k=k+2) 
			 {
		     LocalCheckPoint[0]= ((1-(k/Divide))*sample[3*i]) + (k/Divide*sample[3*j]);
			 LocalCheckPoint[1]= ((1-(k/Divide))*sample[3*i+1]) + (k/Divide*sample[3*j+1]);
             LocalCheckPoint[2]= ((1-(k/Divide))*sample[3*i+2]) + (k/Divide*sample[3*j+2]);			 
			 
			 // set the robot position back to initial value after collision-free checking
			 m_simulator->SetRobotOrientationAndPosition(LocalCheckPoint[0],LocalCheckPoint[1],LocalCheckPoint[2]);
		
			    if (m_simulator->IsRobotInCollision())
	            {Connect[i][j]=0;
			     Connect[j][i]=0;
				 CheckLocal= 0;
			     break;}	 			  
			 }
             
			 Divide = Divide*2; 
			 
		   } 
       
	   }
	   else 
	   {Connect[i][j]=0;
        Connect[j][i]=0;
	   }
	    
	  }  
   }/// End of Re-constructing VxV Connectivity & distance array for sampled configuration based on weighting

// End of weighted-sampling stage 



// Graph Search to find the shortest path using Dijkstra method------------------	

// Initialize the parameters for Dijkstra method:
	m_simulator->SetRobotOrientationAndPosition(sample[0],sample[1],sample[2]);
	for (int i=0; i <Nf+Nr+1; i++)
    {
	D[i]=10000;
	VisitS[i]=0;
	Father[i]=-1;
	}
    
    
// applying Dijistra to check if the solution exists : all rewarding regions are connected to the initial point of robot 

	NrLeft= Nr; // recording the number of remaining rewarding regioned "unreached"
    
	// take the initial point of robot as the first source
	D[0]=0;
    Father[0]=-1;
    
	for (int i=0; i< Nf+Nr+1; i++)
	{
	  
	  PFound =0;
      Dmin=10000; // initial minimum distance to initiate the Dijkstra method
	  
	        
		 for(int j=0; j<Nf+Nr+1; j++)  // find the node with minimum disance value
		   {  if (D[j]<Dmin && !VisitS[j]) 
		        {Dmin=D[j];
		         IndexDmin=j;} 
		   }
            
		 // check if any "rewarding region" has been "visited", if it is, record the order of visiting for future "path assembly" 
		 if ( (IndexDmin>0) && (IndexDmin < Nr+1) && !VisitS[IndexDmin])
		 {
			 VisitOrder.push_back(IndexDmin);
			 NrLeft= NrLeft-1; // reduce the number of remaining regions by one after being visited
		 }
		 
		 // when the node is visited, set status "visit=true";
		    VisitS[IndexDmin]=1;
 		 
		 // relaxation action to update the "shortest distance" value & the ancestor of each node
         for(int j=0; j<Nf+Nr+1; j++)
		 {if (Connect[IndexDmin][j])
		    { if (D[j] > D[IndexDmin] + dist[IndexDmin][j]) 
		       {D[j]= D[IndexDmin] + dist[IndexDmin][j];
		        Father[j] = IndexDmin;  
		        } 
		        
		     }
		  }
		 
	
		 // if all rewarding regions have been visited & connected--> best solution is found  	 
		 if(NrLeft==0) 
		 {PFound=1;}

		
         
         // terminate the Dijkstra loop if: 1. solution has been found /  2. elapsed time is reached maximum time :tmax
         if (PFound || ClockElapsed(&clk) > tmax)
		 {
		   cout<<"NrLeft= "<< NrLeft <<"\n";
		   
		   Search= 0;

		   if (PFound)
		   {
		     cout<< " Congratulation ! Maximal rewarding path has been solved !" << endl;
            for (int i=0; i< Nr;i++)
		    {cout<<"Order of Region reached = "<<VisitOrder[i]<<"\n";}
		   }

		   else  
		   {
			cout<<" The time elapsed has exceeded the upper bound..\n current path found is used as solution\n";
		   }

		   break;		   
		 }
		 

        	   
	}  /// End of Graph Search for loop	    

 } // End of "Solve" while loop
     
   
	
/*
 * you can use the functionality provided in Utils.hpp to measure
 * running time, e.g.,
 *  Clock clk;
 *  ClockStart(&clk);
 *  while(ClockElapsed(&clk) < tmax)
 *  {
 *  }
 * If your algorithm obtains a solution earlier than the maximum time,
 * then you can break out of the while loop (using return or break statement)
 */   

}

void MotionPlanner::GetHighestRewardCollisionFreePath(std::vector<double> * path)
{
 path->clear();

// After solution is found, run Dijkstra to find the path to reach all rewarding regions

// 1st path segment : initial to nearest rewarding region:
// 1st Dijkstra: to get the first segment and the order of visiting (take initial position of robot as the source again)

for (int l=0; l <Nf+Nr+1; l++)
    {D[l]=10000;
	 VisitS[l]=0;
	 Father[l]=-1;}
	
 NrLeft=Nr;
 D[0]=0;
 Father[0]=-1;

for (int i=0; i< Nf+Nr+1; i++)
	{
      Dmin=10000;  
		 for(int j=0; j<Nf+Nr+1; j++)
		   {  if (D[j]<Dmin && !VisitS[j]) 
		        {Dmin=D[j];
		         IndexDmin=j;} 
		   }
            
		 // recording the order of "visiting" according to distance between each rewarding region and the initial point 
		 if ( (IndexDmin>0) && (IndexDmin < Nr+1) && !VisitS[IndexDmin])
		 {
			 VisitOrder.push_back(IndexDmin);
			 NrLeft= NrLeft-1;
		 }
		 
		 // when the node is dequeued, set status visit=true;
		    VisitS[IndexDmin]=1;
 		 
		 // relaxation
         for(int j=0; j<Nf+Nr+1; j++)
		 {if (Connect[IndexDmin][j])
		    { if (D[j] > D[IndexDmin] + dist[IndexDmin][j]) 
		       {D[j]= D[IndexDmin] + dist[IndexDmin][j];
		        Father[j] = IndexDmin;
		        } 
		     }
		  }

	} // End of 1st Dijkstra for loop

//Updating the true Number of "reachable" rewarding region 
Nr=Nr-NrLeft;

//Find the remaing path segment, and assemble the path segment
for (int i=0; i<Nr; i++)
{ 

  // first put the 1st computed path segment(initial to nearest rewarding region) into the vector : Path 
  if(i==0)
  {  
	 Path.insert(Path.begin(),sample[3*VisitOrder[i]+2]);
	 Path.insert(Path.begin(),sample[3*VisitOrder[i]+1]);
     Path.insert(Path.begin(),sample[3*VisitOrder[i]]);
     k=VisitOrder[i];
           // trace back the edge from "goal" to "source" to form the shortest path
           while (Father[k]!=-1)
	       { 
		    Path.insert(Path.begin(),sample[3*Father[k]+2]);
	        Path.insert(Path.begin(),sample[3*Father[k]+1]);
		    Path.insert(Path.begin(),sample[3*Father[k]]);
            k=Father[k];
	        }
  }
  
   // Apply Dijkstra iteratively to find the shortest path from region to region:
  //  finding 2nd segment to (K-1)th segment of the whole robo path (K represents the number of reachable rewarding region)
  else 
  {

    PathTemp.clear();

	//initialization of Dijkstra 
	for (int l=0; l <Nf+Nr+1; l++)
    {D[l]=10000;
	 VisitS[l]=0;
	 Father[l]=-1;}
    
	// set (VisitOrder[i]) region to be the source
	 
	 D[VisitOrder[i-1]]=0;
     Father[VisitOrder[i-1]]=-1;
     
	 // run Dijkstra until the path from ith region to (i+1)th region is found:  
	 // (if the visited status of any rewarding region is "false", the while loop will keep running)
	 while(!VisitS[VisitOrder[i]])
	 {
		 Dmin=10000;
		 
		 for(int j=0; j<Nf+Nr+1; j++)
		   {  if (D[j]<Dmin && !VisitS[j]) 
		        {Dmin=D[j];
		         IndexDmin=j;} 
		   }
            
		 
		 // when the node is visited, set status visit=true;
		    VisitS[IndexDmin]=1;
            
		 
		 // relaxation
         for(int j=0; j<Nf+Nr+1; j++)
		 {if (Connect[IndexDmin][j])
		    { if (D[j] > D[IndexDmin] + dist[IndexDmin][j]) 
		       {D[j]= D[IndexDmin] + dist[IndexDmin][j];
		        Father[j] = IndexDmin;
		        }    
		     }
		  }

		
	 } //End of "while" loop to find path segment from from ith region to (i+1)th region
    
	 // then put all the remaining computed path segments into a vector : PathTemp
     PathTemp.insert(PathTemp.begin(),sample[3*VisitOrder[i]+2]);
	 PathTemp.insert(PathTemp.begin(),sample[3*VisitOrder[i]+1]);
     PathTemp.insert(PathTemp.begin(),sample[3*VisitOrder[i]]);
     k=VisitOrder[i];

           // trace back the edge from "goal" to "source" to form the shortest path
           while (Father[k]!=-1)
	       { 
		    PathTemp.insert(PathTemp.begin(),sample[3*Father[k]+2]);
	        PathTemp.insert(PathTemp.begin(),sample[3*Father[k]+1]);
		    PathTemp.insert(PathTemp.begin(),sample[3*Father[k]]);
            k=Father[k];
	        }

      // Combining all segments to be a single path
      PathTemp.insert(PathTemp.begin(),Path.begin(),Path.end());
      Path=PathTemp;  

  }//End of "else" statement block
        
  
}// End of Second Dijkstra for loop 


// push_back the computed "Path" into the "path" for graphic visualization
   for (int i=0; i< Path.size() / 3  ;i++)
   {
	   path->push_back(Path[3*i]);
       path->push_back(Path[3*i+1]);
	   path->push_back(Path[3*i+2]);
   }


/*
 * CS336: since there is only one goal region in your case, this function needs to simply 
 *        get the path to the goal region with index 0
 */

/*
 * CS436: since you are dealing with multiple goal, you should try to get the path
 *        with the highest accumulated reward
 */

}

void MotionPlanner::Draw(void)
{
    //you can use the draw functions to draw your roadmap or tree
    //e.g.,

	// draw all sampled configuration after applying weight
    DrawColor(0, 0, 1);
    for (int i=0; i <Nf+Nr+1; i++)
	{  
	  DrawPoint2D(sample[3*i+1], sample[3*i+2]);
	}
	
	//draw segments that are collision free 
	for (int i=0; i <Nf+Nr+1; i++)
	{  for (int j=0; j <Nf+Nr+1; j++ )
	   { if (Connect[i][j])
	     DrawSegment2D(sample[3*i+1], sample[3*i+2], sample[3*j+1], sample[3*j+2]);
	         
	   }
	} 
   

}
