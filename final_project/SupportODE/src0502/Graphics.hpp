/**
 *@file Graphics.hpp
 *@author Erion Plaku 
 *@brief Graphics for running simulation and setting up problem
 */

#ifndef  GRAPHICS_HPP_
#define  GRAPHICS_HPP_

#include "MotionPlanner.hpp"
#include "Simulator.hpp"
#include "Camera.hpp"

/**
 *@author Erion Plaku 
 *@brief  Graphics for running simulation and setting up problem
 */
class Graphics
{   
public:
   /**
    *@brief Initialize data and variables
    *
    *@param motionPlanner pointer to motion planner
    */
    Graphics(MotionPlanner  * const motionPlanner);
    
   /**
    *@brief Destroy window
    */
    ~Graphics(void);

   /**
    *@brief Print help information
    */
    void HandleEventOnHelp(void);
    
   /**
    *@brief Main event loop
    */
    void MainLoop(void);

   /**
    *@brief Maximum time to run motion planner
    *@param tmax maximum run time
    */
    void SetMotionPlannerMaxTime(const double tmax)
    {
	m_motionPlannerMaxTime = tmax;
    }
    
    
protected:
   /**
    *@brief Perform simulation step
    */
    void HandleEventOnTimer(void);
    
   /**
    *@brief Main rendering function
    */
    void HandleEventOnDisplay(void);
    
   /**
    *@brief Respond to event when mouse moves
    *
    *@param mousePosX x-position
    *@param mousePosY y-position
    */
    void HandleEventOnMouseMove(const int mousePosX, const int mousePosY);
    
   /**
    *@brief Respond to key presses
    *
    *@param key key pressed
    */
    void HandleEventOnKeyPress(const int key);

   /**
    *@brief Respond to special key presses
    *
    *@param key key pressed
    */
    void HandleEventOnSpecialKeyPress(const int key);
    
   /**
    *@brief Respond to menu-item selections
    *
    *@param item selected item
    */
    void HandleEventOnMenu(const int item);

   /**
    *@name GLUT callback functions
    *@{
    */

    static void CallbackEventOnDisplay(void);
    static void CallbackEventOnMouseMove(int x, int y);
    static void CallbackEventOnMouse(int button, int state, int x, int y);
    static void CallbackEventOnTimer(int id);
    static void CallbackEventOnMenu(int item);
    static void CallbackEventOnKeyPress(unsigned char key, int x, int y);
    static void CallbackEventOnSpecialKeyPress(int key, int x, int y);	
    static void MousePosition(const int x, const int y, double *posX, double *posY);

   /**
    *@}
    */

   /**
    *@name Motion planner data
    *@{
    */
    
   /**
    *@brief Pointer to an instance of the motion planner
    */
    MotionPlanner *m_motionPlanner;
    
    /**
    *@brief Maximum time to run motion planner
    */
    double m_motionPlannerMaxTime;

   /**
    *@brief Total time motion planner has been running
    */
    double m_motionPlannerTotalTime;

   /**
    *@brief Boolean variable to indicate whether or not motion planner should be drawn
    */
    bool m_motionPlannerDraw;
    
    /**
     *@}
     */

    /**
     *@name Solution path
     *@{
     */
    
    /**
     *@brief Sequence of states constituting the 
     *       trajectory computed by the motion planner     
     */
    std::vector<double> m_solutionPath;
    
    /**
     *@brief Boolean variable to indicate whether the solution path 
     *       should be displayed (in animation) or not
     */
    bool m_solutionPathAnimate;    
    
    /**
     *@brief Redisplay rate (in milliseconds) for playing back the solution path
     */
    int m_solutionPathAnimateTimer;

    /**
     *@brief Current index in the solution path animation, i.e.,
     *       which configuration of the solution path should be currently displayed
     */
    int m_solutionPathCurrIndex;

    /**
     *@}
     */

    
   /**
    *@name Different menu options
    *@{
    */

    int MENU_RUN_MOTION_PLANNER;
    int MENU_SET_MAX_TIME_MOTION_PLANNER;    
    int MENU_GET_SOLUTION_PATH;
    int MENU_SET_TIMER_ANIMATE_SOLUTION_PATH;
    int MENU_DRAW_MOTION_PLANNER;
    int MENU_KEYBOARD_CONTROL;
    
    /**
     *@}
     */

    Camera m_camera;
    int    m_mousePrevX;
    int    m_mousePrevY;

    bool   m_keyboardControlEnabled;
    double m_speedControl;
    double m_steeringAngleControl;
};

#endif
