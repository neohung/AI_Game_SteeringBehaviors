#include "obstacle.h"
#include "neogdi.h"
Obstacle::Obstacle(V2D pos, double radius) : BaseClass(0, pos, radius)
{
}

void Obstacle::Update(double time_elapsed)
{

}
void Obstacle::Render()
{
	ngdi->Circle(Pos(), BRadius());
}