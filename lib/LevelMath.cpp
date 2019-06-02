
#include "LevelMath.h"
#include <algorithm>

namespace level_math {
int PointToSegmentSqDistance(const v2i &pt, const CLineBase &line) {
  if (line.GetSqLength() == 0) {
    return mag2(pt - line.GetPos1());
  }
  float d1 = mag2(pt - line.GetPos1());
  float d2 = mag2(pt - line.GetPos2());
  v2i pe = line.GetPos2() - line.GetPos1();
  v2i pd = pt - line.GetPos1();
  float dp = dot(pe, pd);
  float r = dp / mag2(pe);
  float d;
  if (r >= 1.f) {
    d = d2;
  } else if (r <= 0.f) {
    d = d1;
  } else {
    v2i peNew = v2i(pe[1], -pe[0]);
    d = std::abs(dot(pd, peNew) / mag(peNew));
    d = d * d;
  }
  return d;
}

int PointToLineSqDistance(const v2i &pt, const CLineBase &line) {
  return PointToLineSqDistance(pt, line.GetPos2(), line.GetPos1());
}

int PointToLineSqDistance(const v2i &pt, const v2i &p1, const v2i &p2) {
  v2i pe = p2 - p1;
  v2i peNorm = normalize(pe);
  v2i pr = pt - p1;
  int cp = cross(peNorm, pr);
  float d = sqr(cp);
  return d;
}

int RoomPerimeter(const CRoom &room1) {
  int contactArea = 0;
  for (size_t i = 0; i < room1.GetNumOfEdges(); i++) {
    CRoomEdge edge1 = room1.GetEdge(i);
    contactArea += edge1.GetLength();
  }

  return contactArea;
}

int RoomContact(const CRoom &room1, const CRoom &room2) {
  int contactArea = 0;
  for (size_t i = 0; i < room1.GetNumOfEdges(); i++) {
    CRoomEdge edge1 = room1.GetEdge(i);
    for (size_t j = 0; j < room2.GetNumOfEdges(); j++) {
      CRoomEdge edge2 = room2.GetEdge(j);
      if (edge1.GetDoorFlag() == false || edge2.GetDoorFlag() == false) {
        continue;
      }
      int contactAreaTmp = EdgeContact(edge1, edge2);
      contactArea += contactAreaTmp;
    }
  }

  return contactArea;
}

int RoomContact(const CRoom &room1, const CRoom &room2, size_t &edgeIdx1,
                size_t &edgeIdx2) {
  int contactAreaMax = 0.f;
  for (size_t i = 0; i < room1.GetNumOfEdges(); i++) {
    CRoomEdge edge1 = room1.GetEdge(i);
    for (size_t j = 0; j < room2.GetNumOfEdges(); j++) {
      CRoomEdge edge2 = room2.GetEdge(j);
      if (edge1.GetDoorFlag() == false || edge2.GetDoorFlag() == false) {
        continue;
      }
      int contactAreaTmp = EdgeContact(edge1, edge2);
      if (contactAreaTmp > contactAreaMax) {
        contactAreaMax = contactAreaTmp;
        edgeIdx1 = i;
        edgeIdx2 = j;
      }
    }
  }

  return contactAreaMax;
}

int EdgeContact(const CLineBase &line1, const CLineBase &line2) {
  v2i pr1 = line1.GetPos2() - line1.GetPos1();
  v2i pr2 = line2.GetPos2() - line2.GetPos1();
  int cp = cross(pr1, pr2);
  if (sqr(cp) != 0) {
    return 0;
  }
  v2i posMin1 = min_union(line1.GetPos1(), line1.GetPos2());
  v2i posMax1 = max_union(line1.GetPos1(), line1.GetPos2());
  v2i posMin2 = min_union(line2.GetPos1(), line2.GetPos2());
  v2i posMax2 = max_union(line2.GetPos1(), line2.GetPos2());
  for (int j = 0; j < 2; j++) {
    if (posMax1[j] < posMin2[j] || posMin1[j] > posMax2[j]) {
      return 0;
    }
  }
  int d1 = PointToLineSqDistance(line2.GetPos1(), line1);
  int d2 = PointToLineSqDistance(line2.GetPos2(), line1);
  if (d1 > 0 || d2 > 0) {
    return 0;
  }
  // Now the two edges should in the same line anyway...
  float len1 = mag(pr1);
  float len2 = mag(pr2);
  float d11 = mag2(line1.GetPos1() - line2.GetPos1());
  float d21 = mag2(line1.GetPos2() - line2.GetPos1());
  float d12 = mag2(line1.GetPos1() - line2.GetPos2());
  float d22 = mag2(line1.GetPos2() - line2.GetPos2());
  float dMax = sqrt(std::max({d11, d21, d12, d22}));
  dMax = std::max({dMax, len1, len2});
  // On a grid, the overlap between two points is 1, not 0. Add 1
  int contactArea = len1 + len2 - dMax + 1;
  contactArea = std::max(contactArea, 0);
  return contactArea;
}

int RoomDistance(const CRoom &room1, const CRoom &room2) {
  int d = std::numeric_limits<int>::max();
  for (size_t i = 0; i < room1.GetNumOfVertices(); i++) {
    v2i pt = room1.GetVertex(i);
    for (size_t j = 0; j < room2.GetNumOfEdges(); j++) {
      CRoomEdge edge = room2.GetEdge(j);
      int dTmp = PointToSegmentSqDistance(pt, edge);
      d = std::min(d, dTmp);
    }
  }
  d = sqrt(d);
  return d;
}

std::optional<v2i> SegmentIntersection(const CLineBase &line1,
                                       const CLineBase &line2) {
  return SegmentIntersection(line1.GetPos1(), line1.GetPos2(), line2.GetPos1(),
                             line2.GetPos2());
}

std::optional<v2i> SegmentIntersection(v2i pa, v2i pb, v2i pc, v2i pd) {
  return SegmentIntersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0],
                             pd[1]);
}

// Based on the example under
// http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
std::optional<v2i> SegmentIntersection(int Ax, int Ay, int Bx, int By, int Cx,
                                       int Cy, int Dx, int Dy) {
  int Rx = Bx - Ax;
  int Ry = By - Ay;
  int Sx = Dx - Cx;
  int Sy = Dy - Cy;
  int QPx = Cx - Ax;
  int QPy = Cy - Ay;
  int rs = Rx * Sy - Ry * Sx;
  if (rs == 0) {
    return std::nullopt;
  }
  float t = static_cast<float>(QPx * Sy - QPy * Sx) / rs;
  float u = static_cast<float>(QPx * Ry - QPy * Rx) / rs;
  if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
    return v2i(Ax + t * Rx, Ay + t * Ry);
  } else {
    return std::nullopt;
  }
}

std::optional<v2i> LineIntersection(v2i pa, v2i pb, v2i pc, v2i pd) {
  return LineIntersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0],
                          pd[1]);
}

std::optional<v2i> LineIntersection(int Ax, int Ay, int Bx, int By, int Cx,
                                    int Cy, int Dx, int Dy) {
  int Rx = Bx - Ax;
  int Ry = By - Ay;
  int Sx = Dx - Cx;
  int Sy = Dy - Cy;
  int QPx = Cx - Ax;
  int QPy = Cy - Ay;
  int rs = Rx * Sy - Ry * Sx;
  if (rs == 0) {
    return std::nullopt;
  }
  float t = static_cast<float>(QPx * Sy - QPy * Sx) / rs;
  float u = static_cast<float>(QPx * Ry - QPy * Rx) / rs;
  return v2i(Ax + t * Rx, Ay + t * Ry);
}

bool ComparePrSmallerFirst(const PrSort &pr1, const PrSort &pr2) {
  return (pr1.m_dp < pr2.m_dp);
}

void SortVecPr(std::vector<v2i> &vecPr) {
  if (vecPr.size() < 2) {
    return;
  }
  v2i pd = vecPr[1] - vecPr[0];
  std::vector<PrSort> vecPrSort(vecPr.size());
  for (int i = 0; i < int(vecPrSort.size()); i++) {
    vecPrSort[i].m_pr = vecPr[i];
    vecPrSort[i].m_dp = dot(pd, vecPr[i] - vecPr[0]);
  }
  sort(vecPrSort.begin(), vecPrSort.end(), ComparePrSmallerFirst);
  for (int i = 0; i < int(vecPrSort.size()); i++) {
    vecPr[i] = vecPrSort[i].m_pr;
  }
}
} // namespace level_math
