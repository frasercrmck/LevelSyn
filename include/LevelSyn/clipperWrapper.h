//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-06-24
//  email:  chongyangm@gmail.com
//  info: wrapper of the Clipper library
// --------------------------------------------------------------

#ifndef CLIPPERWRAPPER_H
#define CLIPPERWRAPPER_H

#include <clipper.hpp>

#include "RoomLayout.h"

#include "LevelMath.h"

class CClipperWrapper {
public:
  ClipperLib::Paths FindIntersection(const CRoom &room1, const CRoom &room2);

  float ComputeCollideArea(const CRoom &room1, const CRoom &room2);

  float ComputeRoomArea(const CRoom &room);

  static float m_scalingFactor;

private:
  ClipperLib::long64 ConvertFloatToLong64(float f);

  float ConvertLong64ToFloat(ClipperLib::long64 i);

  float ConvertDoubleAreaToFloat(double a);
};

#endif // CLIPPERWRAPPER_H
