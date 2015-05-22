#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;

Graphics::Graphics(const char fname[]) 
{
    m_menuSelItem    = -1;
    m_menuId         = -1;
    m_sensorPoint[0] = m_sensorPoint[1] = HUGE_VAL;
    m_timer          = 10;

    m_simulator.ReadObstacles(fname);
    m_bugAlgorithms = new BugAlgorithms(&m_simulator);
}

Graphics::~Graphics(void)
{
    if(m_bugAlgorithms)
	delete m_bugAlgorithms;
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Bug Algorithms");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutIdleFunc(NULL);
    glutTimerFunc(0, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);
    glutSpecialFunc(CallbackEventOnSpecialKeyPress);

//create menu
    m_menuId = glutCreateMenu(CallbackEventOnMenu);
 
    MENU_STOP                = 1;
    MENU_RESTART             = 2;
    MENU_RUN_BUG1            = 3;
    MENU_RUN_BUG2            = 4;
    MENU_RUN_MYBUG           = 5;
    MENU_RUN_MOVE_STRAIGHT   = 6;
    MENU_RUN_FOLLOW_OBSTACLE = 7;

    
    MENU_SET_ROBOT_CENTER    = 8;
    MENU_SET_ROBOT_RADIUS    = 9;
    MENU_SET_SENSOR_RADIUS   = 10;
    MENU_SET_GOAL_CENTER     = 11;
    MENU_SET_GOAL_RADIUS     = 12;
    MENU_SET_TIMER           = 13;
    MENU_CLEAR_PATH          = 14;
    
    
    glutAddMenuEntry("Stop bug",           MENU_STOP);
    glutAddMenuEntry("Restart bug",        MENU_RESTART);
    glutAddMenuEntry("Run Bug1",           MENU_RUN_BUG1);
    glutAddMenuEntry("Run Bug2",           MENU_RUN_BUG2);
    glutAddMenuEntry("Run MyBug",          MENU_RUN_MYBUG);
    glutAddMenuEntry("Run MoveStraight",   MENU_RUN_MOVE_STRAIGHT);
    glutAddMenuEntry("Run FollowObstacle", MENU_RUN_FOLLOW_OBSTACLE);


    glutAddMenuEntry("Set robot center",  MENU_SET_ROBOT_CENTER);
    glutAddMenuEntry("Set robot radius",  MENU_SET_ROBOT_RADIUS);
    glutAddMenuEntry("Set sensor radius", MENU_SET_SENSOR_RADIUS);
    glutAddMenuEntry("Set goal center",   MENU_SET_GOAL_CENTER);
    glutAddMenuEntry("Set goal radius",   MENU_SET_GOAL_RADIUS);
    glutAddMenuEntry("Set timer",         MENU_SET_TIMER);
    glutAddMenuEntry("Clear path",        MENU_CLEAR_PATH);


    glutAttachMenu(GLUT_RIGHT_BUTTON);	

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{	
    double dx =0.0;
    double dy =0.0;
    
    if(m_menuSelItem != MENU_STOP &&
       !m_simulator.IsRobotInCollision() &&
       !m_simulator.HasRobotReachedGoal())
    {
	if(m_menuSelItem == MENU_RUN_BUG1)
	    m_bugAlgorithms->Bug1Strategy(dx, dy);
	else if(m_menuSelItem == MENU_RUN_BUG2)
	    m_bugAlgorithms->Bug2Strategy(dx, dy);
	else if(m_menuSelItem == MENU_RUN_MYBUG)
	    m_bugAlgorithms->MyBugStrategy(dx, dy);
	else if(m_menuSelItem == MENU_RUN_MOVE_STRAIGHT)
	    m_bugAlgorithms->StepMoveStraightTowardGoal(dx, dy);
	else if(m_menuSelItem == MENU_RUN_FOLLOW_OBSTACLE)
	    m_bugAlgorithms->StepFollowObstacleBoundary(dx, dy);
	m_simulator.SetRobotCenter(m_simulator.GetRobotCenterX() + dx, 
				   m_simulator.GetRobotCenterY() + dy);
    }


    m_simulator.TakeSensorReading(m_sensorPoint[0], m_sensorPoint[1]);
} 

void Graphics::HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY)
{
    if(m_menuSelItem == MENU_SET_ROBOT_CENTER)
	m_simulator.SetRobotCenter(mousePosX, mousePosY);
    else if(m_menuSelItem == MENU_SET_ROBOT_RADIUS)
    {	    
	const double x = m_simulator.GetRobotCenterX();
	const double y = m_simulator.GetRobotCenterY();
	
	m_simulator.SetRobotRadius(sqrt((x - mousePosX) * (x - mousePosX) + 
					(y - mousePosY) * (y - mousePosY)));
    }	
    else if(m_menuSelItem == MENU_SET_SENSOR_RADIUS)
    {	    
	const double x = m_simulator.GetRobotCenterX();
	const double y = m_simulator.GetRobotCenterY();
	
	m_simulator.SetSensorRadius(sqrt((x - mousePosX) * (x - mousePosX) + 
					 (y - mousePosY) * (y - mousePosY)));
    }	
    else if(m_menuSelItem == MENU_SET_GOAL_CENTER)
	m_simulator.SetGoalCenter(mousePosX, mousePosY);
    else if(m_menuSelItem == MENU_SET_GOAL_RADIUS)
    {	    
	const double x = m_simulator.GetGoalCenterX();
	const double y = m_simulator.GetGoalCenterY();
	
	m_simulator.SetGoalRadius(sqrt((x - mousePosX) * (x - mousePosX) + 
				       (y - mousePosY) * (y - mousePosY)));
    }	
}

void Graphics::HandleEventOnMenu(const int item)
{
    m_menuSelItem = item;
    
    if(item == MENU_SET_TIMER)
    {
	printf(" current timer interval = %d [ms]\n", m_timer);
	printf(" enter new value (the smaller the value, the faster the redisplay rate): ");
	scanf("%d", &m_timer);
    }
    else if(item == MENU_RESTART)
	m_bugAlgorithms->Restart();
    else if(item == MENU_CLEAR_PATH)
	m_simulator.m_path.clear();
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    printf("pressed key = %d\n", key);
    
    switch(key)
    {
    case 27: //escape key
	exit(0);
	
    case GLUT_KEY_F1: 
	HandleEventOnHelp();	
	break;
    }
}

void Graphics::HandleEventOnHelp(void)
{
    printf("Help: right click to display menu\n\n");
    printf(" From the menu:\n");
    printf("   * Stop bug\n");
    printf("   * Restart bug\n");
    printf("   * Run various bug algorithms\n");
    printf(" Setup:\n");
    printf("   after selecting a setup option\n");	    
    printf("   * left-click to desired place to set up the value for\n");
    printf("     robot, sensor, and goal\n");
    printf("   * set timer by typing in the\n");
    printf("     desired value in the terminal window\n");
    
}


void Graphics::HandleEventOnDisplay(void)
{
//draw robot
    glColor3f(1, 0, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    DrawCircle2D(m_simulator.GetRobotCenterX(), m_simulator.GetRobotCenterY(), m_simulator.GetRobotRadius());
    
//draw sensor range
    glColor3f(0, 0, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);	
    DrawCircle2D(m_simulator.GetRobotCenterX(), m_simulator.GetRobotCenterY(), m_simulator.GetSensorRadius());

//draw sensor segment
    if(m_sensorPoint[0] != HUGE_VAL && m_sensorPoint[1] != HUGE_VAL)
    {
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	glVertex2d(m_simulator.GetRobotCenterX(), m_simulator.GetRobotCenterY());
	glVertex2d(m_sensorPoint[0], m_sensorPoint[1]);
	glEnd();
    }

//draw goal
    glColor3f(0, 1, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    DrawCircle2D(m_simulator.GetGoalCenterX(), m_simulator.GetGoalCenterY(), m_simulator.GetGoalRadius());

//draw trajectory    
    glColor3f(0, 0, 1);
    glLineWidth(3.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < (int) m_simulator.m_path.size(); i += 2)
	glVertex2dv(&m_simulator.m_path[i]);
    glEnd();	
    glLineWidth(1.0);	

//draw obstacles
    glColor3f(0.45, 0.34, 0.76);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    glBegin(GL_TRIANGLES);
    for(int i = 0; i < (int) m_simulator.m_obstacles.size(); ++i)
    {
	Simulator::Obstacle *obst = m_simulator.m_obstacles[i];
	const int            ntri = obst->m_triangles.size();
    
	for(int j = 0; j < ntri; j += 3)
	{
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 0]]);
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 1]]);
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 2]]);
	}
    }
    glEnd();
}


void Graphics::DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}


void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);	
	
	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-22, 22, -14, 14, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    
	
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseLeftBtnDown(mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(m_graphics->m_timer, CallbackEventOnTimer, id);
	glutPostRedisplay();	    
    }
}

void Graphics::CallbackEventOnMenu(int item)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnMenu(item);
	glutPostRedisplay();
    }    
}

void Graphics::CallbackEventOnSpecialKeyPress(int key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

int main(int argc, char **argv)
{
    if(argc < 2)
    {
	printf("missing obstacle file argument\n");		
	printf("  RunBug <obstacle_file.txt>\n");
	return 0;		
    }

    Graphics graphics(argv[1]);
    
    graphics.HandleEventOnHelp();
    graphics.MainLoop();
    
    return 0;    
}
