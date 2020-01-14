#include "Insrob_server/orient.h"

Orient::Orient(){
    state = ROBOT_STATIC;
}

Orient::Orient(const char* mark){
	state = ROBOT_STATIC;
	this->mark = mark;
}

