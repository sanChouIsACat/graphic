#pragma once
namespace GComponent {
// response for accept 2D input (unit: pixel) and move camera.
class CameraControlI {
public:
  virtual ~CameraControlI(){};
  // when user move to left (press 'a'),input will always be -1.
  virtual void keyBoardX(int x, long long timestamp) = 0;
  // when user move to down (press 'a'),input will always be -1.
  virtual void keyBoardZ(int y, long long timestamp) = 0;
  // mouse movement(usually means rotate cameras)
  virtual void mouse(int x, int y) = 0;
  // window resize
  virtual void resize(int width, int height) = 0;
  // when mouse move into screen
  virtual void setEnterCoords(int x, int y) = 0;
  ;
};
} // namespace GComponent
