#ifndef TRACKING_H
#define TRACKING_H

#include "common_include.h"

namespace SOFT
{
class System;
class Frame;
class Tracking
{

public:
  typedef std::shared_ptr<Tracking> Ptr;

  
public:
    Tracking();
    ~Tracking();
    
    void TrackOne();

};

}

#endif