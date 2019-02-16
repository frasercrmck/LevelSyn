//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-08-22
//  email:  chongyangm@gmail.com
//  info: base class declaration of a line-shaped object
// --------------------------------------------------------------

#ifndef LINEBASE_H
#define LINEBASE_H

#include "vec.h"

class CLineBase {
public:
  CLineBase() {
    m_pos1 = v2i(0);
    m_pos2 = v2i(0);
  }

  CLineBase(v2i pos) {
    SetPos1(pos);
    SetPos2(pos);
  }

  CLineBase(v2i pos1, v2i pos2) {
    SetPos1(pos1);
    SetPos2(pos2);
  }

  int GetLength() const { return mag(m_pos2 - m_pos1); }

  int GetSqLength() const { return mag2(m_pos2 - m_pos1); }

  v2i GetPos1() const { return m_pos1; }
  v2i GetPos2() const { return m_pos2; }

  void SetPos1(v2i pos1) { m_pos1 = pos1; }
  void SetPos2(v2i pos2) { m_pos2 = pos2; }

protected:
  v2i m_pos1;
  v2i m_pos2;
};

#endif // LINEBASE_H
