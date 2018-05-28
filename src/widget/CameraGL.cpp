/*****************************************************************************\
 Mapper3D

 CameraGL.cpp
 Author: roehling

 Copyright (c) 2008 FGAN/FKIE/FUS. All rights reserved.

\*****************************************************************************/
#include "CameraGL.h"
#include <cmath>
#include <GL/gl.h>
#include <iostream>

/*****************************************************************************/

void CameraGL::translate (float forward, float up, float sideways)
{
  mZ -= forward * cosf(mYaw) + sideways * sinf(mYaw);
  mX -= forward * sinf(mYaw) - sideways * cosf(mYaw);
  mY += up;
}

/*****************************************************************************/

void CameraGL::rotate (float yaw, float pitch)
{
  mYaw += yaw;
  mPitch += pitch;
  if (mPitch < -M_PI_2) mPitch = -M_PI_2;
  if (mPitch > M_PI_2) mPitch = M_PI_2;
}

/*****************************************************************************/

void CameraGL::setVelocity (float forward, float up, float sideways, float turn)
{
  mVelForward = forward;
  mVelUp = up;
  mVelSide = sideways;
  mVelTurn = turn;
}

/*****************************************************************************/

void CameraGL::applyVelocity (float dt)
{
  rotate (mVelTurn * dt, 0.0f);
  translate (mVelForward * dt, mVelUp * dt, mVelSide * dt);
}

/*****************************************************************************/

bool CameraGL::hasVelocity() const
{
  return fabsf(mVelForward) > 0.0f || fabsf(mVelUp) > 0.0f || fabsf(mVelSide) > 0.0f || fabsf(mVelTurn) > 0.0f;
}

/*****************************************************************************/

void CameraGL::applyGL()
{
  glRotatef (-180.0f * mYaw / M_PI, 0.0f, 1.0f, 0.0f);
  glRotatef (-180.0f * mPitch / M_PI, cosf(mYaw), 0.0f, -sinf(mYaw));
  glTranslatef (-mX, -mY, -mZ);
}
