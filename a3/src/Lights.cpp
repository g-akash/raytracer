/*
 * Lights.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "Lights.hpp"

Light::Light()
{
  RGB black(0, 0, 0);
  illumination_ = black;
  falloff_ = 0;
  angular_falloff_ = 0;
  dead_distance_ = 1;
}

Light::Light(RGB const & illumination)
{
  illumination_ = illumination;
}

Light::Light(RGB const & illumination, double falloff, double dead_distance)
{
  illumination_ = illumination;
  falloff_ = falloff;
  dead_distance_ = dead_distance;
}

Light::~Light()
{}

RGB Light::getColor() const
{
  return illumination_;
}

RGB Light::getColor(Vec3 const & p) const
{
  return illumination_;
}

void
Light::setColor(RGB const & c)
{
  illumination_ = c;
}

AmbientLight::AmbientLight()
{
  // intentionally empty
}

AmbientLight::AmbientLight(RGB const & illumination) : Light(illumination)
{
  // intentionally empty
}

Vec3
AmbientLight::getIncidenceVector(Vec3 const & position) const
{
  throw "AMBIENT LIGHTS DO NOT HAVE A SENSE OF DIRECTION OR POSITION`";
}

Ray AmbientLight::getShadowRay(Vec3 const & position, bool & use_dist, double width, double height, double length, double diff, int pos) const
{
  throw "AMBIENT LIGHTS DO NOT HAVE A SENSE OF DIRECTION OR POSITION";
}

PointLight::PointLight(RGB const & illumination) : Light(illumination)
{
  // intentionally empty
}

PointLight::PointLight(RGB const & illumination, double falloff, double dead_distance)
: Light(illumination, falloff, dead_distance)
{
  // intentionally empty
}

RGB
PointLight::getColor(Vec3 const & p) const
{
	
	Vec3 loc = p-pos_;




	double scaling_fac = 1.0/pow((loc.length()+dead_distance_),falloff_);

	return illumination_*scaling_fac;
  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

void
PointLight::setPosition(Vec3 const & pos)
{
  pos_ = pos;
}

Vec3
PointLight::getIncidenceVector(Vec3 const & position) const
{
	
	Vec3 Incidence = pos_-position;
	return Incidence.normalize();
  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

double frand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

Ray
PointLight::getShadowRay(Vec3 const & position, bool & use_dist, double width, double height, double length, double diff, int pos) const
{
	Ray shadowray;
  if(int(width*height*length)==1)
  {
    Vec3 dr = pos_-position;
    shadowray = Ray::fromOriginAndDirection(position+0.001*dr,dr,1.0);
    return shadowray;
  }
  Vec3 lpos(pos_.x()-width*diff, pos_.y()-height*diff, pos_.z()-length*diff);
  double xrand=frand(0.0,diff), yrand=frand(0.0,diff), zrand=frand(0.0,diff);
  Vec3 fpos(lpos.x() + double((pos%int(width*length))%int(width))*diff + xrand, 
            lpos.y() +double(pos/int(width*length))*diff+ yrand,
            lpos.z() + double((pos%int(width*length))/int(width))*diff + zrand);
	Vec3 dir = fpos-position;
  //std::cout<<fpos<<" "<<pos<<std::endl;
	shadowray = Ray::fromOriginAndDirection(position+0.001*dir,dir,1.0);
	return shadowray;
  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

DirectionalLight::DirectionalLight(RGB const & illumination) : Light(illumination)
{
  // intentionally empty
}

void
DirectionalLight::setDirection(Vec3 const & dir)
{
  dir_ = dir;
  dir_.normalize();
}

Vec3
DirectionalLight::getIncidenceVector(Vec3 const & position) const
{
	return -1.0*dir_;
  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

Ray
DirectionalLight::getShadowRay(Vec3 const & position, bool & use_dist, double width, double height, double length, double diff, int pos) const
{
	Ray shadowray;
	Vec3 dir = -1*dir_;
	shadowray = Ray::fromOriginAndDirection(position+0.001*dir,dir,std::numeric_limits<double>::infinity());

	return shadowray;
  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}
