//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-07-15
//  email:  chongyangm@gmail.com
//  info: class declaration of a configuration space
// --------------------------------------------------------------

#ifndef CONFIGSPACE_H
#define CONFIGSPACE_H

#include "LevelMath.h"
#include "Room.h"
#include "clipperWrapper.h"

class CConfigLine : public CLineBase {
public:
  CConfigLine(v2i pos1, v2i pos2) : CLineBase(pos1, pos2) {}

  CConfigLine(v2i pos) : CLineBase(pos) {}

  v2i RandomlySampleConfigLine();

  v2i RandomlySampleConfigLineDiscrete();

  int GetConfigLineLength() const;

  int GetConfigLineSqLength() const;

  void PrintConfigLine();

  void TranslateConfigLine(v2i trans);
};

class CConfigSpace {
public:
  CConfigSpace() {}

  // The configuration space to put room2 around room1...
  CConfigSpace(const CRoom &room1, const CRoom &room2);

  CConfigSpace(std::vector<CConfigLine> &vecConfigLines);

  void AddConfigLine(CConfigLine &line) { m_vecConfigLine.push_back(line); }

  v2i RandomlySampleConfigSpace();

  v2i RandomlySampleConfigSpaceContinuous();

  v2i RandomlySampleConfigSpaceDiscrete();

  std::vector<v2i> SmartlySampleConfigSpace();

  size_t GetNumOfLines() { return m_vecConfigLine.size(); }

  bool IsEmpty() { return m_vecConfigLine.empty(); }

  void SetConfigLines(std::vector<CConfigLine> &vecConfigLine) {
    m_vecConfigLine = vecConfigLine;
  }

  std::vector<CConfigLine> &GetConfigLines() { return m_vecConfigLine; }

  CConfigLine &GetConfigLine(unsigned idx) { return m_vecConfigLine[idx]; }

  static CConfigSpace FindIntersection(CConfigSpace &configSpace1,
                                       CConfigSpace &configSpace2);

  static CConfigSpace FindUnion(CConfigSpace &configSpace,
                                CConfigLine &configLine);

  void SelfMerge();

  int GetConfigSpaceSize();

  void PrintConfigSpace();

  void TranslateConfigSpace(v2i trans);

  static bool CompareConfigLineLength(const CConfigLine &line1,
                                      const CConfigLine &line2);

  static void PrecomputeTable(const std::vector<CRoom> &vecRooms);

  static std::vector<std::vector<CConfigSpace>> m_precomputedTable;

  static bool m_flagPrecomputed;

private:
  std::vector<CConfigLine> m_vecConfigLine;
};

#endif // CONFIGSPACE_H
