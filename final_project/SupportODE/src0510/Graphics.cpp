#include "Graphics.hpp"
#include "Utils.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


Graphics *m_graphics = NULL;

Graphics::Graphics(MotionPlanner * const motionPlanner) 
{
    m_motionPlanner          = motionPlanner;
    m_motionPlannerMaxTime   = 10;  //seconds
    m_motionPlannerTotalTime = 0;
    m_motionPlannerDraw      = true;
    
    m_solutionPathAnimate      = false;
    m_solutionPathAnimateTimer = 5; //milliseconds
    m_solutionPathCurrIndex    = -1;

    m_camera.SetLookAt(0, 0, 35, 0, 0, 0, 0, 1, 0);
    m_camera.RotateAroundRightAxisAtCenter(45 * M_PI / 180.0);

    m_keyboardControlEnabled = true;
    m_speedControl = 0.0;
    m_steeringAngleControl = 0.0;    
}

Graphics::~Graphics(void)
{
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
    glutCreateWindow("Window");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutMotionFunc(CallbackEventOnMouseMove);
    glutIdleFunc(NULL);
    glutTimerFunc(0, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);
    glutSpecialFunc(CallbackEventOnSpecialKeyPress);

//create menu
    glutCreateMenu(CallbackEventOnMenu);

    MENU_RUN_MOTION_PLANNER          = 1;
    MENU_SET_MAX_TIME_MOTION_PLANNER = 2;    
    MENU_GET_SOLUTION_PATH               = 3;
    MENU_SET_TIMER_ANIMATE_SOLUTION_PATH = 4;
    MENU_DRAW_MOTION_PLANNER         = 5;
 
    glutAddMenuEntry("Run motion planner",                 MENU_RUN_MOTION_PLANNER);
    glutAddMenuEntry("Set max time to run motion planner", MENU_SET_MAX_TIME_MOTION_PLANNER);
    glutAddMenuEntry("Get solution path",                  MENU_GET_SOLUTION_PATH);
    glutAddMenuEntry("Set timer to animate solution path",     MENU_SET_TIMER_ANIMATE_SOLUTION_PATH);
    glutAddMenuEntry("Draw motion planner [yes/no]",       MENU_DRAW_MOTION_PLANNER);
    glutAddMenuEntry("Keyboard control [yes/no]",          MENU_KEYBOARD_CONTROL);

    glutAttachMenu(GLUT_RIGHT_BUTTON);	

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{	    
    if(m_solutionPathAnimate && m_solutionPath.size() > 0)
    {	
	const int n = m_motionPlanner->m_simulator->GetNrStateDimensions();
	
	++m_solutionPathCurrIndex;
	if(n * m_solutionPathCurrIndex >= m_solutionPath.size())
	    m_solutionPathCurrIndex = (m_solutionPath.size()/n) - 1;

	const double *state = &m_solutionPath[m_solutionPathCurrIndex * n];
	
	m_motionPlanner->m_simulator->SetCurrentState(state);
    }
    else if(m_keyboardControlEnabled)
	m_motionPlanner->m_simulator->SimulateOneStep(m_speedControl, m_steeringAngleControl);
    
} 


void Graphics::HandleEventOnMouseMove(const int mousePosX, const int mousePosY)
{	
    const double thetax = 2 * M_PI * (mousePosY - m_mousePrevY) / glutGet(GLUT_WINDOW_HEIGHT);
    const double thetay = 2 * M_PI * (mousePosX - m_mousePrevX) / glutGet(GLUT_WINDOW_WIDTH);	    
    const double thetaz = thetay;
    
    m_camera.RotateAroundUpAxisAtCenter(thetay);
    m_camera.RotateAroundRightAxisAtCenter(thetax);
    m_camera.RotateAroundForwardAxisAtCenter(thetaz);
   
//    m_camera.RotateAroundUpAxisAtEye(thetay);
//    m_camera.RotateAroundRightAxisAtEye(thetax);
//    m_camera.RotateAroundForwardAxisAtEye(thetaz);
}

void Graphics::HandleEventOnMenu(const int item)
{
    if(item == MENU_RUN_MOTION_PLANNER)
    {
	Clock clk;
	ClockStart(&clk);
	m_motionPlanner->Solve(m_motionPlannerMaxTime);
	m_motionPlannerTotalTime += ClockElapsed(&clk);
	printf("Motion planner has been run for %f seconds\n", m_motionPlannerTotalTime);	
    }
    else if(item == MENU_SET_MAX_TIME_MOTION_PLANNER)
    {
	printf(" current max time for motion planner = %f [s]\n", m_motionPlannerMaxTime);
	printf(" enter new value: ");
	scanf("%lf", &m_motionPlannerMaxTime);
    }
    else if(item == MENU_DRAW_MOTION_PLANNER)
    {
	m_motionPlannerDraw = !m_motionPlannerDraw;
	printf("DRAW_MOTION_PLANNER: %s\n", m_motionPlannerDraw ? "YES" : "NO");	
    }    
    else if(item == MENU_GET_SOLUTION_PATH)
    {
	m_solutionPath.clear();
	m_motionPlanner->GetSolutionPath(&m_solutionPath);
	m_solutionPathAnimate   = true;
	m_solutionPathCurrIndex = -1;
    }
    else if(item == MENU_SET_TIMER_ANIMATE_SOLUTION_PATH)
    {
	printf(" current timer interval to animate solution path = %d [ms]\n", m_solutionPathAnimateTimer);
	printf(" enter new value (the smaller the value, the faster the redisplay rate): ");
	scanf("%d", &m_solutionPathAnimateTimer);
    }    
    else if(item == MENU_KEYBOARD_CONTROL)
    {
	m_keyboardControlEnabled = !m_keyboardControlEnabled;
	printf("KEYBOARD_CONTROL_ENABLED = %s\n", m_keyboardControlEnabled ? "YES":"NO");	
    }
    
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    switch(key)
    {
    case 27: //escape key
	exit(0);

    case 'i':
	m_speedControl += 0.1;
	printf("speed control = %f\n", m_speedControl);
	break;
    case 'k':
	m_speedControl -= 0.1;
	printf("speed control = %f\n", m_speedControl);
	break;
    case 'l':
	m_steeringAngleControl += 0.1;
	printf("steering angle control = %f\n", m_steeringAngleControl);
	break;
    case 'j':
	m_steeringAngleControl -= 0.1;
	printf("steering angle control = %f\n", m_steeringAngleControl);
	break;
    case GLUT_KEY_F1: 
	HandleEventOnHelp();	
	break;
	
    }
}

void Graphics::HandleEventOnSpecialKeyPress(const int key)
{
    const double cameraMove = 0.1;
    
    switch(key)
    {
    case GLUT_KEY_UP:
	m_camera.MoveForward(cameraMove);
	glutPostRedisplay();
	break;
		
    case GLUT_KEY_DOWN:
	m_camera.MoveForward(-cameraMove);
	glutPostRedisplay();
	break;
		
    case GLUT_KEY_LEFT:
	m_camera.MoveRight(-cameraMove);
	glutPostRedisplay();
	break;
		
    case GLUT_KEY_RIGHT:
	m_camera.MoveRight(cameraMove);
	glutPostRedisplay();
	break;

    case GLUT_KEY_PAGE_UP:
	m_camera.MoveUp(cameraMove);
	glutPostRedisplay();
	break;
	
    case GLUT_KEY_PAGE_DOWN:
	m_camera.MoveUp(-cameraMove);
	glutPostRedisplay();
	break;
    }    
}


void Graphics::HandleEventOnHelp(void)
{
    printf("Help: right click to display menu\n");
    printf("--when KEYBOARD_CONTROLLER is enabled, you can\n");
    printf("  drive the vehicle by using the following keys:\n");
    printf("   i : increase velocity\n");
    printf("   k : decrease velocity\n");
    printf("   j : increase steering angle\n");
    printf("   l : decrease steering angle\n");
    printf("--use the arrow keys and PageUp/PageDown to move camera\n");
    printf("--press left mouse button and move mouse to rotate camera\n");
}


void Graphics::HandleEventOnDisplay(void)
{
    if(m_motionPlannerDraw)
	m_motionPlanner->Draw();
    
    m_motionPlanner->m_simulator->DrawRobot();
    m_motionPlanner->m_simulator->DrawEnvironment();
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

	gluPerspective(45, 
		       (double) glutGet(GLUT_WINDOW_WIDTH) / glutGet(GLUT_WINDOW_HEIGHT),
		       0.05, 1000);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    

	double m[16];
	m_graphics->m_camera.GetModelViewMatrixOpenGL(m);
	glMultMatrixd(m);
	
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	// Initialize lights 
	const GLfloat lmodel_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	const GLfloat light0_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glEnable(GL_LIGHT0);
	const GLfloat light1_diffuse[] = { 1, 1, 1, 1.0 };
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
	glEnable(GL_LIGHT1);
	
	// Set lights
	const GLfloat light0_position[] = { 3.0, 4.0, 5.0, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	const GLfloat light1_position[] = { -3.0, -2.0, -3.0, 0.0 };
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
		
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics)
    {
	m_graphics->m_mousePrevX = x;
	m_graphics->m_mousePrevY = y;
    }
}

void Graphics::CallbackEventOnMouseMove(int x, int y)
{
    if(m_graphics)
    {		
	m_graphics->HandleEventOnMouseMove(x, y);
	m_graphics->m_mousePrevX = x;
	m_graphics->m_mousePrevY = y;
    }
}


void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(m_graphics->m_solutionPathAnimateTimer, CallbackEventOnTimer, id);
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
	m_graphics->HandleEventOnSpecialKeyPress(key);	
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

