/*
 * CADCamera.h
 *
 *  Created on: Jul 24, 2018
 *      Author: cagcoach
 * inspired by: behley
 *
 */

#ifndef INCLUDE_RV_CADCAMERA_H_
#define INCLUDE_RV_CADCAMERA_H_

#include <glow/util/GlCamera.h>

#include <chrono>
#include <mutex>

/** \brief Blender-like camera.
 *
 *  \author christian
 */
class CADCamera : public glow::GlCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** \brief update the view matrix and return it. **/
  const Eigen::Matrix4f& matrix() override;

  /** \brief set camera to specified transformation matrix. **/
  void setMatrix(const Eigen::Matrix4f& m) override;

  /** \brief set location of the camera. **/
  void setPosition(float x, float y, float z) override;

  /** \brief get current location of the camera. **/
  Eigen::Vector4f getPosition() const override;

  /** \brief set camera directions such that the reference point is centered **/
  void lookAt(float x_ref, float y_ref, float z_ref) override;
  /** \brief set camera to specified position looking at given reference point. **/
  void lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) override;

  /** \brief set yaw angle.
   *
   *  yaw = rotation about y-axis.
   ***/
  void setYaw(float yaw);

  /** \brief set pitch angle.
   *
   *  pitch = rotation about x-axis.
   **/
  void setPitch(float pitch);

  /** \brief get camera parameters.
   *
   *  \see setPosition, setYaw, setPitch.
   **/
  void getCameraParameters(float& x, float& y, float& z, float& yaw, float& pitch);

  /** \brief process mouse pressed at position (x,y) with given KeyModifier
   *
   * \return true, if event was processed. false, otherwise.*
   **/
  bool mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) override;

  /** \brief process mouse released at position (x,y) with given KeyModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  bool mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) override;

  /** \brief process mouse moved event at position (x,y) with given KeyModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  bool mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) override;

  /**  \brief process mouse wheel events by delta values, i.e., how much the wheel position changed. **/
  bool wheelEvent(float delta, KeyboardModifier modifier) override;
  bool keyPressed(KeyboardKey key, KeyboardModifier modifier) override;
  bool keyReleased(KeyboardKey key, KeyboardModifier modifier) override;

 protected:
  void translate(float forward, float up, float sideways);
  void rotate(float yaw, float pitch);

  std::chrono::system_clock::time_point startTime_;  // of mouse pressed.
  float startx_{0.0f}, starty_{0.0f};
  float startyaw_{0.0f}, startpitch_{0.0f};

  float x_{0.0f}, y_{0.0f}, z_{0.0f};
  float startcx_{0.0f}, startcy_{0.0f}, startcz_{0.0f};
  float yaw_{0.0f}, pitch_{0.0f};

  bool startdrag_{false};

  // velocity vector:
  float forwardVel_{0.0f}, upVel_{0.0f}, sideVel_{0.0f}, turnVel_{0.0f};

  // TODO: expose other parameters (sensitivity, etc.)

  std::mutex mutex_;
};

#endif /* INCLUDE_RV_CADCAMERA_H_ */
