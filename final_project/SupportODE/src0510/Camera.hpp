/**
 *@author Erion Plaku
 *@file Camera.hpp
 *@brief Camera interface
 */

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

/**
 *@author Erion Plaku
 *@brief Camera interface
 */
class Camera
{
public:
    /**
     *@author Erion Plaku
     *@brief Initialize data elements to appropriate default
     *       values 
     */
    Camera(void);	
    
    
    /**
     *@author Erion Plaku
     *@brief Free the memory allocated to different data elements 
     */
    virtual ~Camera(void);
    
    /**
     *@name Get camera configuration
     *@{
     */
    
    /**
     *@author Erion Plaku
     *@brief Get a copy of the camera configuration matrix for OpenGL
     *@param m configuration matrix (16 doubles)
     */
    void GetModelViewMatrixOpenGL(double * const m);
    
    /**
     *@author Erion Plaku
     *@brief Get camera eye position
     */
    const double* GetEye(void) const
    {
	return m_eye;
    }
    
    
    /**
     *@author Erion Plaku
     *@brief Get center point
     *@param center center point
     */
    const double* GetCenter(void) const
    {
	return m_center;
    }
    
    /**
     *@author Erion Plaku
     *@brief Get up vector
     */
    const double* GetUpAxis(void) const
    {
	return m_up;
    }
    
    /**
     *@author Erion Plaku
     *@brief Get right vector
     */
    const double* GetRightAxis(void) const
    {
	return m_right;
    }
    
    /**
     *@author Erion Plaku
     *@brief Get up vector
     */
    const double* GetForwardAxis(void) const
    {
	return m_forward;
    }
    
    /**
     *@}
     */
    
    /**
     *@name Setup camera
     *@{
     */
    
    /**
     *@author Erion Plaku
     *@brief Set camera eye position, center point, and up direction
     *@param eye_x x position of camera eye
     *@param eye_y y position of camera eye
     *@param eye_z z position of camera eye
     *@param center_x x position of camera center point
     *@param center_y y position of camera center point
     *@param center_z z position of camera center point
     *@param up_x x coordinate of camera up vector
     *@param up_y y coordinate of camera up vector
     *@param up_z z coordinate of camera up vector
     */
    void SetLookAt(const double eye_x,
		   const double eye_y,
		   const double eye_z,
		   const double center_x,
		   const double center_y,
		   const double center_z,
		   const double up_x,
		   const double up_y,
		   const double up_z);
    
    /**
     *@author Erion Plaku
     *@brief Standard camera look-at setup
     */
    void SetLookAtStandardPosition(void);
    
    /**
     *@}
     */
    
    /**
     *@name Move camera
     *@{
     */
    
    /**
     *@author Erion Plaku
     *@brief Move camera forward
     *@param d translation amount
     */
    void MoveForward(const double d);
    
    /**
     *@author Erion Plaku
     *@brief Move camera right
     *@param d translation amount
     */
    void MoveRight(const double d);
    
    /**
     *@author Erion Plaku
     *@brief Move camera up
     *@param d translation amount
     */
    void MoveUp(const double d);
    
    
    void Move(const double df, const double dr, const double du);
    
    /**
     *@}
     */
    
    /**
     *@name Rotate camera
     *@{
     */
    
    /**
     *@author Erion Plaku
     *@brief Rotate camera using camera eye as origin of
     *       rotation  
     *@param R camera rotation matrix (9 doubles)
     */
    void RotateAtEye(const double * const R)
    {
	RotateAtEye(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
    }
    
    void RotateAtEye(const double r0, const double r1, const double r2,
		     const double r3, const double r4, const double r5,
		     const double r6, const double r7, const double r8);
    void RotateAtCenter(const double * const R)
    {
	RotateAtCenter(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
    }
    void RotateAtCenter(const double r0, const double r1, const double r2,
			const double r3, const double r4, const double r5,
			const double r6, const double r7, const double r8)
    {
	RotateAtPoint(r0, r1, r2, r3, r4, r5, r6, r7, r8, m_center[0], m_center[1], m_center[2]);
    }
    
    
    void RotateAtPoint(const double * const R, 
		       const double px, const double py, const double pz)
    {
	RotateAtPoint(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], px, py, pz);
    }
    
    void RotateAtPoint(const double r0, const double r1, const double r2,
		       const double r3, const double r4, const double r5,
		       const double r6, const double r7, const double r8,
		       const double px, const double py, const double pz);
    
    void RotateAroundAxisAtEye(const double theta, const double vx, const double vy, const double vz)
    {
	RotateAroundAxisAtPoint(theta, vx, vy, vz, m_eye[0], m_eye[1], m_eye[2]);
    }
    
    void RotateAroundAxisAtCenter(const double theta, const double vx, const double vy, const double vz)
    {
	RotateAroundAxisAtPoint(theta, vx, vy, vz, m_center[0], m_center[1], m_center[2]);
    }
    
    void RotateAroundAxisAtPoint(const double theta, 
				 const double vx, const double vy, const double vz,
				 const double px, const double py, const double pz);
    
    void RotateAroundRightAxisAtEye(const double theta)
    {
	RotateAroundAxisAtEye(theta, m_right[0], m_right[1], m_right[2]);
    }
    void RotateAroundRightAxisAtCenter(const double theta)
    {
	RotateAroundAxisAtCenter(theta, m_right[0], m_right[1], m_right[2]);
    }
    void RotateAroundRightAxisAtPoint(const double theta, 
				      const double px, const double py, const double pz)
    {
	RotateAroundAxisAtPoint(theta, m_right[0], m_right[1], m_right[2], px, py, pz);
    }
    
    void RotateAroundUpAxisAtEye(const double theta)
    {
	RotateAroundAxisAtEye(theta, m_up[0], m_up[1], m_up[2]);
    }
    void RotateAroundUpAxisAtCenter(const double theta)
    {
	RotateAroundAxisAtCenter(theta, m_up[0], m_up[1], m_up[2]);
    }
    void RotateAroundUpAxisAtPoint(const double theta, 
				   const double px, const double py, const double pz)
    {
	RotateAroundAxisAtPoint(theta, m_up[0], m_up[1], m_up[2], px, py, pz);
    }
    
    
    void RotateAroundForwardAxisAtEye(const double theta)
    {
	RotateAroundAxisAtEye(theta, m_forward[0], m_forward[1], m_forward[2]);
    }
    void RotateAroundForwardAxisAtCenter(const double theta)
    {
	RotateAroundAxisAtCenter(theta, m_forward[0], m_forward[1], m_forward[2]);
    }
    void RotateAroundForwardAxisAtPoint(const double theta, 
					const double px, const double py, const double pz)
    {
	RotateAroundAxisAtPoint(theta, m_forward[0], m_forward[1], m_forward[2], px, py, pz);
    }
    
    /**
     *@}
     */
    
protected:
    /**
     *@author Erion Plaku
     *@brief Camera configuration
     */
    double m_center[3];
    double m_eye[3];
    double m_up[3];
    double m_right[3];
    double m_forward[3];
};

#endif









