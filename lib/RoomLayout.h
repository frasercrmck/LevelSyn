//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-03-07
//  email:  chongyangm@gmail.com
//  info: class declaration of a room layout, i.e. a set of rooms
// --------------------------------------------------------------

#ifndef ROOMLAYOUT_H
#define ROOMLAYOUT_H

//#define DUMP_INTERMEDIATE_OUTPUT
//#define DUMP_PARTIAL_SOLUTION
#define PERFORMANCE_TEST

#include "Room.h"
#include <map>

typedef CLineBase CorridorWall;

struct RoomConnection {
  v2i p;
  size_t room_idx0;
  size_t room_idx1;
};

class CRoomLayout {
public:
  void ClearLayout() { m_rooms.clear(); }

  void AddRoom(CRoom &room) { m_rooms.push_back(room); }
  void AddRoomConnection(RoomConnection &connection) {
    m_connections.push_back(connection);
  }

  size_t GetNumOfRooms() const { return m_rooms.size(); }

  unsigned GetNumOfVertices();

  unsigned GetNumOfEdges();

  CRoom &GetRoom(size_t idx) { return m_rooms[idx]; }

  const CRoom &GetRoom(size_t idx) const { return m_rooms[idx]; }

  AABB2i GetLayoutBoundingBox() const;

  void MoveToSceneCenter();

  std::vector<v2i> GetRoomPositions();

  void ResetRoomEnergies();

  void PrintLayout() const;

  bool SaveLayoutAsSVG(const char *fileName, int wd = 400, int ht = 400,
                       bool writeOnlyVisited = false,
                       class CPlanarGraph *graphBest = nullptr,
                       bool labelFlag = true);

  static int ConvertPos(int p, int pMin, int pMax, int sz);

  static int ConvertPosX(int p, int pMin, int pMax, int sz);

  static int ConvertPosY(int p, int pMin, int pMax, int sz);

  void InsertCorridorWall(CorridorWall &wall) {
    m_corridorWalls.push_back(wall);
  }

  size_t GetNumOfCorridorWalls() const { return m_corridorWalls.size(); }

  RoomWall &GetCorridorWall(unsigned idx) { return m_corridorWalls[idx]; }

  std::map<std::pair<int, int>, float> cachedCollisionEnergies;
  std::map<std::pair<int, int>, int> cachedConnectivities;
  std::map<std::pair<int, int>, int> cachedContacts;

  const std::vector<CRoom> &rooms() const { return m_rooms; }
  const std::vector<RoomConnection> &connections() const {
    return m_connections;
  }

private:
  std::vector<CRoom> m_rooms;
  std::vector<RoomConnection> m_connections;
  std::vector<CorridorWall> m_corridorWalls;
};

#endif // ROOMLAYOUT_H
