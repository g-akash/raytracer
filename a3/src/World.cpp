/*
 * World.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "World.hpp"

World::World()
{
}

World::~World()
{
  // TODO Auto-generated destructor stub
}

Primitive *
World::intersect(Ray & r) const
{
	Primitive* inter=NULL;
	for(int i=0;i<primitives_.size();i++)
	{
		if(primitives_[i]->intersect(r))
		{

			inter = primitives_[i];

		}
	}
	return inter;
  IMPLEMENT_ME(__FILE__, __LINE__);
}

void
World::addPrimitive(Primitive * p)
{
  primitives_.push_back(p);
}

void
World::addLight(Light * l)
{
  lights_.push_back(l);
}

void
World::setAmbientLightColor(RGB ambientColor)
{
  ambientLight_.setColor(ambientColor);
}

RGB
World::getAmbientLightColor() const
{
  return ambientLight_.getColor();
}

void
World::printStats() const
{
  std::cout << "World data:" << std::endl;
  std::cout << " primitives: " << primitives_.size() << std::endl;
  std::cout << " lights: " << lights_.size() << std::endl;
}
