//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-06-15
//  email:  chongyangm@gmail.com
//  info: wrapper for basic math operations
// --------------------------------------------------------------

#ifndef LEVELMATH_H
#define LEVELMATH_H

#include "Room.h"
#include <optional>

namespace level_math {
typedef struct PrSort {
  v2i m_pr;
  int m_dp; // dot product
} PrSort;

const float g_numericalTolerance = 0.0f; // 1e-4f; //1e-6;

int PointToSegmentSqDistance(const v2i &pt, const CLineBase &line);

int PointToLineSqDistance(const v2i &pt, const CLineBase &line);

int PointToLineSqDistance(const v2i &pt, const v2i &p1, const v2i &p2);

int RoomPerimeter(const CRoom &room1);

int MaxRoomContact(const CRoom &room1, const CRoom &room2,
                   size_t *edgeIdx1 = nullptr, size_t *edgeIdx2 = nullptr);

int EdgeContact(const CLineBase &line1, const CLineBase &line2);

int RoomDistance(const CRoom &room1, const CRoom &room2);

std::optional<v2i> SegmentIntersection(const CLineBase &line1,
                                       const CLineBase &line2);

std::optional<v2i> SegmentIntersection(v2i pa, v2i pb, v2i pc, v2i pd);

std::optional<v2i> SegmentIntersection(int Ax, int Ay, int Bx, int By, int Cx,
                                       int Cy, int Dx, int Dy);

std::optional<v2i> LineIntersection(v2i pa, v2i pb, v2i pc, v2i pd);

std::optional<v2i> LineIntersection(int Ax, int Ay, int Bx, int By, int Cx,
                                    int Cy, int Dx, int Dy);

bool ComparePrSmallerFirst(const PrSort &pr1, const PrSort &pr2);

void SortVecPr(std::vector<v2i> &vecPr);
} // namespace level_math

#endif // LEVELMATH_H
