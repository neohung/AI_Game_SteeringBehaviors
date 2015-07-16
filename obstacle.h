#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "BaseClass.h"
class Obstacle : public BaseClass
{
private:

public:
	Obstacle(V2D pos, double radius);
	~Obstacle(){}
	void Update(double time_elapsed); 
	void Render();

};
#endif