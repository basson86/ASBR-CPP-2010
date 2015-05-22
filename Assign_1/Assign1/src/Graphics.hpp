/**
 *@file Graphics.hpp
 *@author Erion Plaku 
 *@brief Graphics for running simulation and setting up problem
 */

#ifndef  GRAPHICS_HPP_
#define  GRAPHICS_HPP_

#include "BugAlgorithms.hpp"
#include "Simulator.hpp"

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
    *@param fname name of file with obstacles
    */
    Graphics(const char fname[]);
    
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
    *@brief Respond to event when left button is clicked
    *
    *@param mousePosX x-position
    *@param mousePosY y-position
    */
    void HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY);
    
   /**
    *@brief Respond to key presses
    *
    *@param key key pressed
    */
    void HandleEventOnKeyPress(const int key);
    
   /**
    *@brief Respond to menu-item selections
    *
    *@param item selected item
    */
    void HandleEventOnMenu(const int item);


   /**
    *@brief Draw circle
    *
    *@param cx x position of circle center
    *@param cy y position of circle center
    *@param r circle radius
    */
    void DrawCircle2D(const double cx, const double cy, const double r);

   /**
    *@name GLUT callback functions
    *@{
    */

    static void CallbackEventOnDisplay(void);
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
    *@brief An instance of the simulator
    */
    Simulator m_simulator;
    
   /**
    *@brief Pointer to an instance of the bug algorithms
    */
    BugAlgorithms *m_bugAlgorithms;
    
   /**
    *@brief Time interval for callback
    */
    int m_timer;

   /**
    *@brief Nearest point from the obstacles to the robot center (if within sensor range)
    */
    double m_sensorPoint[2];
    
   /**
    *@name Different menu options
    *@{
    */

    int MENU_STOP;
    int MENU_RESTART;
    int MENU_RUN_BUG1;
    int MENU_RUN_BUG2;
    int MENU_RUN_MYBUG;
    int MENU_RUN_MOVE_STRAIGHT;
    int MENU_RUN_FOLLOW_OBSTACLE;

    int MENU_SET_ROBOT_CENTER;
    int MENU_SET_ROBOT_RADIUS;
    int MENU_SET_SENSOR_RADIUS;
    int MENU_SET_GOAL_CENTER;
    int MENU_SET_GOAL_RADIUS;
    int MENU_SET_TIMER;
    int MENU_CLEAR_PATH;
    
    
    /**
     *@}
     */
    
   /**
    *@brief Menu id
    */
    int m_menuId;

   /**
    *@brief Selected menu item
    */
    int m_menuSelItem;
};

#endif
