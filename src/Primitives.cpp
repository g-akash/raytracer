/*
 * Primitive.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "Primitives.hpp"

Primitive::Primitive(RGB const & c, Material const & m, Mat4 const & modelToWorld)
{
  c_ = c;
  m_ = m;
  modelToWorld_ = modelToWorld;
  worldToModel_ = modelToWorld.inverse();
}

Primitive::~Primitive()
{
}

Sphere::Sphere(double radius, RGB const & c, Material const & m, Mat4 const & modelToWorld): Primitive(c, m, modelToWorld)
{
  r_ = radius;
}

bool
Sphere::intersect(Ray & ray) const
{
	 Ray transformed = ray;
   transformed.transform(worldToModel_);
   double radius = r_;
   Vec3 rayorigin = transformed.start();
   Vec3 raydirection = transformed.direction();
   double discriminant = (raydirection*rayorigin)*(raydirection*rayorigin) - (raydirection*raydirection)*(rayorigin*rayorigin-radius*radius);
   double negb = -1.0*(raydirection*rayorigin);
   double denom = raydirection*raydirection;

   if(discriminant<0)
   {
    return false;
   }
   double rootdiscriminant = sqrt(discriminant);
   double lowertime = (negb-rootdiscriminant)/denom;
   double uppertime = (negb+rootdiscriminant)/denom;

   if(lowertime<ray.minT()&&lowertime>0)
   {
    ray.setMinT(lowertime);
    return true;
   }
   if(uppertime<ray.minT()&&uppertime>0)
   {
    ray.setMinT(uppertime);
    return true;
   } 
  return false;
  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

Vec3
Sphere::calculateNormal(Vec3 const & position) const
{
	
	Vec3 center(0.0,0.0,0.0);
	center = modelToWorld_*center;
	Vec3 normal = position-center;
	normal.normalize();
	return normal; 

  // TODO for 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

bool 
Sphere::isinside(Vec3 pos)
{
	Vec3 center(0.0,0.0,0.0);
	return (pos-modelToWorld_*center).length()<=r_;
}

std::string
Sphere::getshape()
{
	return "sphere";
}
//=============================================================================================================================
// Triangle and other primitives are for Assignment 3b, after the midsem. Do not do this for 3a.
//=============================================================================================================================

Triangle::Triangle(Vec3 const & v0, Vec3 const & v1, Vec3 const & v2, RGB const & c, Material const & m,
                   Mat4 const & modelToWorld)
: Primitive(c, m, modelToWorld)
{
  verts[0] = v0;
  verts[1] = v1;
  verts[2] = v2;
}

bool
Triangle::intersect(Ray & ray) const
{
  // TODO for 3b, NOT 3a
  Ray transformed = ray;
  transformed.transform(worldToModel_);
  Vec3 o, p, d, e1, e2, h, s, q;
  float a, f, u, v, t;
  e1 = verts[1]-verts[0];
  e2 = verts[2]-verts[0];
  d = transformed.direction();
  o = transformed.start();
  p = d^e2;
  a = e1*p;
  if (a > -0.0000001 && a < 0.0000001)
    return false;
  f = 1.0/a;
  s=o-verts[0];
  u=f*s*p;
  if (u < 0.0 || u > 1.0)
    return false;
  q=s^e1;
  v=f*d*q;
  if (v < 0.0 || u + v > 1.0)
    return false;
  t=f*e2*q;
  if (t > 0.00000&&t<ray.minT()) // ray intersection
    {
      ray.setMinT(t);
      return true;
    }
  return false;
  IMPLEMENT_ME(__FILE__, __LINE__);
}

Vec3
Triangle::calculateNormal(Vec3 const & position) const
{

  Vec3 p, d, e1, e2, h, s, q;
  float a, f, u, v;
  e1 = verts[1]-verts[0];
  e2 = verts[2]-verts[0];
  h = e1^e2;
  Vec4 temp(h,0.0);
  temp = modelToWorld_*temp;
  h=Vec3(temp,3);
  h.normalize();
  
  return h;
  // TODO for 3b, NOT 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}

bool 
Triangle::isinside(Vec3 pos)
{
	return true;
}

std::string
Triangle::getshape()
{
	return "triangle";
}
