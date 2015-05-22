#include "Simulator.hpp"

Simulator::Simulator(void)
{
//default values
    m_robotRadius  = 0.5;
    m_robotCenterX = -20;
    m_robotCenterY = 13;
    
    m_sensorRadius = 2 * m_robotRadius;

    m_goalRadius   = 1.2 * m_robotRadius;
    m_goalCenterX  = 16;
    m_goalCenterY  = -12;    
}

Simulator::~Simulator(void)
{
    const int n = m_obstacles.size();
    for(int i = 0; i < n; ++i)
	if(m_obstacles[i])
	    delete m_obstacles[i];    
}


void Simulator::ReadObstacles(const char fname[])
{
//file with obstacles
//file format is just a sequence of polygons

    FILE *in = fopen(fname, "r");
    if(in)
    {
	int       n;	
	Obstacle *obst;
	
	while(fscanf(in, "%d", &n) == 1)
	{
	    obst = new Obstacle();
	    obst->m_vertices.resize(2 * n);	    
	    for(int i = 0; i < 2 * n; ++i)
		fscanf(in, "%lf", &(obst->m_vertices[i]));
	    obst->m_triangles.resize(3 * (n - 2));
	    for(int i = 0; i < (int) obst->m_triangles.size(); ++i)
		fscanf(in, "%d", &(obst->m_triangles[i]));
	    m_obstacles.push_back(obst);	    
	}
	fclose(in);	    
    }	
    else
	printf("..could not open file <%s>\n", fname);
    
}

double PointSegmentDistanceSquare(const double p[],
				  const double s0[],
				  const double s1[],
				  double pmin[]);


double PointPolygonDistanceSquare(const double p[],
				  const int    n,
				  const double poly[],
				  double pmin[]);

double Simulator::TakeSensorReading(double &xmin, double &ymin) const
{
    const double  c[2] = {m_robotCenterX, m_robotCenterY};
    double        dmin = HUGE_VAL;
    double        d    = 0;
    double        pmin[2];
    
    for(int i = 0; i < (int) m_obstacles.size(); ++i)
    {
	const int     nv   = m_obstacles[i]->m_vertices.size() / 2;
	const double *poly = &(m_obstacles[i]->m_vertices[0]);
	
	if((d = PointPolygonDistanceSquare(c, nv, poly, pmin)) < dmin)
	{
	    dmin = d;
	    xmin = pmin[0];
	    ymin = pmin[1];
	}   
    }

    if(dmin < HUGE_VAL)
    {
	dmin = sqrt(dmin);
	if(dmin > m_sensorRadius)
	{
	    dmin = m_sensorRadius;
	    xmin = ymin = HUGE_VAL;
	}
    }
    return dmin;
}

bool Simulator::IsRobotInCollision(void) const
{
    const double  c[2] = {m_robotCenterX, m_robotCenterY};
    double        pmin[2];
    
    for(int i = 0; i < (int) m_obstacles.size(); ++i)
    {
	const int     nv   = m_obstacles[i]->m_vertices.size() / 2;
	const double *poly = &(m_obstacles[i]->m_vertices[0]);
	
	for(int j = 0; j < nv; ++j)	    
	    if(PointSegmentDistanceSquare(c, &poly[2 * j], &poly[2 * ((j+1) % nv)], pmin) < m_robotRadius * m_robotRadius)
		return true;
    }
    return false;    
}

double PointSegmentDistanceSquare(const double p[],
				  const double s0[],
				  const double s1[],
				  double pmin[])
{
    double a, b;
    double v[2] = {s1[0] - s0[0], s1[1] - s0[1]};
    double w[2] = {p [0] - s0[0], p [1] - s0[1]};
    
    if((a = (v[0] * w[0] + v[1] * w[1])) <= 0)
    {
	pmin[0] = s0[0];
	pmin[1] = s0[1];
	return w[0] * w[0] + w[1] * w[1];
    }	
    if((b = (v[0] * v[0] + v[1] * v[1])) <= a)
    {
	pmin[0] = s1[0];
	pmin[1] = s1[1];
	
	return (p[0] - s1[0]) * (p[0] - s1[0]) +
	       (p[1] - s1[1]) * (p[1] - s1[1]);
    }
    a   /= b;
    v[0] = s0[0] + a * v[0];
    v[1] = s0[1] + a * v[1];
    
    pmin[0] = v[0];
    pmin[1] = v[1];
    
    return (p[0] - v[0]) * (p[0] - v[0]) +  (p[1] - v[1]) * (p[1] - v[1]);
}

double PointPolygonDistanceSquare(const double p[],
				  const int    n,
				  const double poly[],
				  double pmin[])
{
    double dmin = PointSegmentDistanceSquare(p, &poly[2 * n - 2], &(poly[0]), pmin);
    double d;
    double lpmin[2];
    int    i;
    
    for(i = 0; i < n - 1; ++i)
	if((d = PointSegmentDistanceSquare(p, &poly[2 * i], &poly[2 * i + 2], lpmin)) < dmin)
	{
	    pmin[0] = lpmin[0];
	    pmin[1] = lpmin[1];
	    dmin = d;
	}
    return dmin;
}
