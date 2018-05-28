/***/
#ifndef CAMERAGL_H_
#define CAMERAGL_H_

class CameraGL {
  public:
    CameraGL() :
      mX(0.0f), mY(0.0f), mZ(0.0f), mYaw(0.0f), mPitch(0.0f),
      mVelForward(0.0f), mVelUp(0.0f), mVelSide(0.0f), mVelTurn(0.0f) {}
    void setXYZ (float x, float y, float z) { mX = x; mY = y; mZ = z; }
    void setYaw (float yaw) { mYaw = yaw; }
    void setPitch (float pitch) { mPitch = pitch; }
    void translate (float forward, float up, float sideways);
    void rotate (float yaw, float pitch);
    void applyGL();
    void setVelocity (float forward, float up, float sideways, float turn);
    void applyVelocity (float dt = 1.0f);
    float x() const { return mX; }
    float y() const { return mY; }
    float z() const { return mZ; }
    float yaw() const { return mYaw; }
    float pitch() const { return mPitch; }
    bool hasVelocity() const;
  private:
    float mX, mY, mZ, mYaw, mPitch;
    float mVelForward, mVelUp, mVelSide, mVelTurn;
};

#endif /* CAMERAGL_H_ */
