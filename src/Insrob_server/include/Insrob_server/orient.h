#ifndef ORIENT_H
#define ORIENT_H
#include "Insrob_server/global.h"

class Orient{
public:
    Orient();
	Orient(const char* mark);
    ~Orient(){}
    bool operator ==(const Orient& rhs){
        if(rhs.mark == this->mark)
            return true;
        else
            return false;
    }

public:
    State state;
    string mark;
};

#endif // ORIENT_H
