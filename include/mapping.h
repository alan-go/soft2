#ifndef MAPPING_H
#define MAPPING_H

#include "common_include.h"
#include "frame.h"

namespace SOFT
{


class Tracking;
class Frame;
class System;

class Mapping
{  

public:
  typedef std::shared_ptr<Mapping> Ptr;
 // std::unordered_map<unsigned long, Frame::Ptr> frames;
  //std::unordered_map<unsigned long, Frame::Ptr> keyFrames;
  
public:
    Mapping();
    ~Mapping();
    
    
    void RunMapping();
    
};
}

#endif