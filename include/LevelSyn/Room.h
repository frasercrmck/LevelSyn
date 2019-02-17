//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-03-07
//  email:  chongyangm@gmail.com
//  info: class declaration of a single room
// --------------------------------------------------------------

#ifndef ROOM_H
#define ROOM_H

#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <vector>

#include "RoomEdge.h"
#include "vec.h"

typedef struct AABB2i {
  v2i m_posMin;
  v2i m_posMax;
} AABB2i;

typedef CLineBase RoomWall;
typedef CLineBase RoomDoor;

class CRoom {
public:
  CRoom();

  std::vector<v2i> &GetVertices() { return m_vertices; }

  void SetVertices(std::vector<v2i> &vertices) { m_vertices = vertices; }

  void SetVertex(v2i &pos, unsigned idx) { m_vertices[idx] = pos; }

  void SetCenterShift(v2i &shift) { m_centerShift = shift; }

  v2i GetCenterShift() const { return m_centerShift; }

  v2i GetVertex(size_t idx) const { return m_vertices[idx]; }

  CRoomEdge GetEdge(size_t idx) const;

  size_t GetNumOfVertices() const { return m_vertices.size(); }

  size_t GetNumOfEdges() const { return m_vertices.size(); }

  v2i GetRoomCenter() const;

  v2i GetShiftedRoomCenter();

  void TranslateRoom(v2i trans);

  void RotateRoom(float rad);

  void ScaleRoom(float scaling);

  void ScaleRoom(v2f scaling);

  AABB2i GetRoomBoundingBox() const;

  float GetEnergy() { return m_energy; }
  void SetEnergy(float energy) { m_energy = energy; }
  void ResetEnergy() { SetEnergy(1.f); }
  void UpdateEnergy(float factor) { m_energy *= factor; }

  void PrintRoom() const;

  bool HasWalls() const { return (m_walls.empty() == false); }

  void InitWalls();

  bool EraseWall(int idx);

  void InsertWall(RoomWall &wall) { m_walls.push_back(wall); }

  size_t GetNumOfWalls() const { return m_walls.size(); }

  RoomWall &GetWall(size_t idx) { return m_walls[idx]; }
  const RoomWall &GetWall(size_t idx) const { return m_walls[idx]; }

  int GetTemplateType() const { return m_templateType; }
  void SetTemplateType(int type) { m_templateType = type; }

  int GetBoundaryType() const { return m_boundaryType; }
  void SetBoundaryType(int type) { m_boundaryType = type; }

  bool GetFlagFixed() const { return m_flagFixed; }
  void SetFlagFixed(bool flagFixed) { m_flagFixed = flagFixed; }

  void ResetDoorFlags();
  void SetDoorFlag(int edgeIdx, bool doorFlag);
  bool GetDoorFlag(int edgeIdx) const;
  std::vector<bool> GetDoorFlags() const;
  bool HasRestrictedDoorPosition() const;

  const std::vector<RoomWall> &walls() const { return m_walls; }


private:
  std::vector<v2i> m_vertices;

  v2i m_centerShift;

  std::vector<RoomWall> m_walls;

  float m_energy;

  int m_templateType;

  bool m_flagFixed;

  int m_boundaryType;

  std::vector<bool> m_doorFlags;
};

#endif // ROOM_H
