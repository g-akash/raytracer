#include "Globals.hpp"
#include "View.hpp"
#include "World.hpp"
#include "Frame.hpp"
#include "Lights.hpp"
#include "core/Scene.hpp"

using namespace std;

//****************************************************
// Global Variables
//****************************************************

Scene * scene = NULL;
World * world = NULL;
View * view = NULL;
Mat4 viewToWorld = identity3D();
Frame * frame = NULL;
int max_trace_depth = 2;
double width = 3;
double height = 3;
double length = 1;
double diff = 0.6;
int tot_points = int(width*height*length);

// Get the shaded appearance of the primitive at a given position, as seen along a ray. The returned value should be the sum of
// the shaded colors w.r.t. each light in the scene. DO NOT include the result of recursive raytracing in this function, just
// use the ambient-diffuse-specular formula. DO include testing for shadows, individually for each light.
RGB
getShadedColor(Primitive const & primitive, Vec3 const & pos, Ray const & ray)
{
	
	{


    RGB objectcolor = primitive.getColor();
    RGB ambientcolor = world->getAmbientLightColor();
    Material mat = primitive.getMaterial();
    double ambientcoeff = mat.getMA();
    double metalness = mat.getMSM();
    double lambertian = mat.getML();
    double specular = mat.getMS();
    double MSP = mat.getMSP();
    RGB finambientcolor = objectcolor*ambientcolor*ambientcoeff;



    RGB finlambertiancolor = RGB(0.0,0.0,0.0);
    RGB finspecularcolor = RGB(0.0,0.0,0.0);
    std::vector<Light *>::const_iterator  it = world->lightsBegin();
    std::vector<Light *>::const_iterator last = world->lightsEnd();
    
    //need point of intersection for this.
    Vec3 intersect = pos;
    int countsources=0;
    while(it!=last)
    {
      countsources++;
      bool ishitting;
      for(int i=0;i<tot_points;i++){
        Ray shadowray = (*it)->getShadowRay(pos,ishitting, width, height, length, diff, i);
        Primitive* inter = world->intersect(shadowray);
        if(inter==NULL)
        {
          Vec3 reflectance;
          Vec3 viewingray;
          RGB S = metalness*(objectcolor)+(1.0-metalness)*(RGB(1.0,1.0,1.0));
          Vec3 Incidence = (*it)->getIncidenceVector(intersect);
          Incidence.normalize();


          RGB intensity = (*it)->getColor(intersect);

          Vec3 normal=primitive.calculateNormal(intersect);
          normal.normalize();
          reflectance = -1.0*Incidence+2.0*(Incidence*normal)*normal;

          viewingray = ray.direction();
          viewingray.normalize();
          finlambertiancolor += lambertian*(objectcolor*intensity*max(Incidence*normal,0.0));
          finspecularcolor += specular*(S*intensity*pow(max(-1.0*reflectance*viewingray,0.0),MSP));
        }
      }

      it++;
    }

    return finambientcolor+finlambertiancolor/double(tot_points)+finspecularcolor/double(tot_points);
    return RGB(1.0,1.0,1.0);
  }
  return RGB(0.0,0.0,0.0);




	
	
	
  IMPLEMENT_ME(__FILE__, __LINE__);

  // Use the "world" global variable to access the lights in the input file.
  //  Please be sure to read the following classes in Types.hpp:
  //   - RGB
  //   - Material
  //   - Ray
}

// Raytrace a single ray backwards into the scene, calculating the total color (summed up over all reflections/refractions) seen
// along this ray.
RGB
traceRay(Ray & ray, int depth)
{
  ray.setMinT(std::numeric_limits<double>::infinity());

  if (depth > max_trace_depth)

    return RGB(0.0, 0.0, 0.0);
  /** 
  *Some sample implementation to check if a few things works or not.
  *
  */

  Primitive* inter = world->intersect(ray);
  Vec3 intersect = ray.getPos(ray.minT());
  RGB shadedcolor(0.0,0.0,0.0),reflectedcolor(0.0,0.0,0.0), refractedcolor(0.0,0.0,0.0);
  Ray reflectedray,refractedray;
  double reflectiveproperty, refractiveproperty;
  if(inter!=NULL)
  {

    Material mat = inter->getMaterial();
    RGB objectcolor = inter->getColor();
    reflectiveproperty = mat.getMR();

    shadedcolor = getShadedColor(*inter,intersect,ray);
    Vec3 normal=inter->calculateNormal(intersect);
    normal.normalize();

    Vec3 Incidence = ray.direction();
    Incidence.normalize();
    Vec3 reflecteddir = Incidence-2.0*(Incidence*normal)*normal;
    reflecteddir.normalize();
    Vec3 startpos = ray.getPos(ray.minT())+0.001*reflecteddir; 
    
    reflectedray = Ray::fromOriginAndDirection(startpos,reflecteddir,std::numeric_limits<double>::infinity());
    reflectedcolor = traceRay(reflectedray,depth+1);
    
    reflectedcolor=(reflectiveproperty*objectcolor*reflectedcolor);


    if(inter->getshape()=="sphere")
    {
    	double cost1 = (-1.0)*(Incidence*normal)/(Incidence.length()*normal.length());

        if(cost1<0){cost1*=-1.0;normal*=-1.0;}
        double sint1 = sqrt(1.0-cost1*cost1);
        double eta1,eta2;
        //if((ray.start()-inter->getcenter()).length()<inter->getradius())
        if(inter->isinside(ray.start()))
        {
        	eta1=mat.getMTN();
        	eta2=1.0;
        }
        else{
        	eta1=1.0;
        	eta2=mat.getMTN();
        }
        double sint2sq = pow((eta1/eta2),2)*(sint1*sint1);
        if(sint2sq<=1.0)
        {
        	double cost2 = sqrt(1.0-sint2sq);
        	refractiveproperty = mat.getMT();
        	Vec3 refracteddir;
    
        	refracteddir = (eta1/eta2)*(1.0)*Incidence+((eta1/eta2)*cost1-cost2)*normal;
    
        	Vec3 refractedstartpos = ray.getPos(ray.minT())+0.001*refracteddir;
        	refractedray = Ray::fromOriginAndDirection(refractedstartpos,refracteddir,std::numeric_limits<double>::infinity());
        	refractedcolor = traceRay(refractedray,depth+1);
        	refractedcolor=(refractiveproperty*objectcolor*refractedcolor);
        }
    }

  }
  

  return shadedcolor+reflectedcolor+refractedcolor;

  IMPLEMENT_ME(__FILE__, __LINE__);

  // Use the "world" global variable to access the primitives in the input file.
  //  IMPORTANT:
  //  Please start all bounce rays at a small non-zero t value such as 0.001 - this has the effect of slightly offsetting
  //  bounce rays from the surface they're bouncing from, and prevents bounce rays from being occluded by their own surface.
}

// Main rendering loop.
void
renderWithRaytracing()
{
  Sample sample;   // Point on the view being sampled.
  Ray ray;         // Ray being traced from the eye through the point.
  RGB c;           // Color being accumulated per pixel.

  int const rpp = view->raysPerPixel();

  for (int yi = 0; yi < view->height(); ++yi)
  {
    for (int xi = 0; xi < view->width(); ++xi)
    {
      c = RGB(0, 0, 0);
      for (int ri = 0; ri < rpp; ++ri)
      {
        view->getSample(xi, yi, ri, sample);
        ray = view->createViewingRay(sample);  // convert the 2d sample position to a 3d ray
        ray.transform(viewToWorld);            // transform this to world space
        c += traceRay(ray, 0);
      }

      frame->setColor(sample, c / (double)rpp);
    }
  }
}

// This traverses the loaded scene file and builds a list of primitives, lights and the view object. See World.hpp.
void
importSceneToWorld(SceneInstance * inst, Mat4 localToWorld, int time)
{
  if (inst == NULL)
    return;

  Mat4 nodeXform;
  inst->computeTransform(nodeXform, time);
  localToWorld = localToWorld * nodeXform;
  SceneGroup * g = inst->getChild();

  if (g == NULL)   // for example if the whole scene fails to load
  {
    std::cout << "ERROR: We arrived at an instance with no child?!" << std::endl;
    return;
  }

  int ccount = g->getChildCount();

  for (int i = 0; i < ccount; i++)
  {
    importSceneToWorld(g->getChild(i), localToWorld, 0);
  }

  CameraInfo f;

  if (g->computeCamera(f, time))
  {
    viewToWorld = localToWorld;

    if (view != NULL)
      delete view;

    Vec3 eye(0.0, 0.0, 0.0);
    Vec3 LL(f.sides[FRUS_LEFT], f.sides[FRUS_BOTTOM], -f.sides[FRUS_NEAR]);
    Vec3 UL(f.sides[FRUS_LEFT], f.sides[FRUS_TOP], -f.sides[FRUS_NEAR]);
    Vec3 LR(f.sides[FRUS_RIGHT], f.sides[FRUS_BOTTOM], -f.sides[FRUS_NEAR]);
    Vec3 UR(f.sides[FRUS_RIGHT], f.sides[FRUS_TOP], -f.sides[FRUS_NEAR]);
    view = new View(eye, LL, UL, LR, UR, IMAGE_WIDTH, IMAGE_HEIGHT, RAYS_PER_PIXEL_EDGE);
  }

  LightInfo l;

  if (g->computeLight(l, time))
  {
    if (l.type == LIGHT_AMBIENT)
    {
      RGB amb = world->getAmbientLightColor();
      world->setAmbientLightColor(amb + l.color);
    }
    else if (l.type == LIGHT_DIRECTIONAL)
    {
      DirectionalLight * li = new DirectionalLight(l.color);
      Vec3 dir(0, 0, -1);
      li->setDirection(localToWorld * dir);
      world->addLight(li);
    }
    else if (l.type == LIGHT_POINT)
    {
      PointLight * li = new PointLight(l.color, l.falloff, l.deadDistance);
      Vec3 pos(0, 0, 0);
      li->setPosition(localToWorld * pos);
      world->addLight(li);
    }
    else if (l.type == LIGHT_SPOT)
    {
      throw "oh no";
    }
  }

  double r;
  MaterialInfo m;

  if (g->computeSphere(r, m, time))
  {
    Material mat(m.k[0], m.k[1], m.k[2], m.k[3], m.k[4], m.k[MAT_MS], m.k[5], m.k[6]);
    Sphere * sph = new Sphere(r, m.color, mat, localToWorld);
    world->addPrimitive(sph);
  }

  TriangleMesh * t;

  if (g->computeMesh(t, m, time))
  {
    Material mat(m.k[0], m.k[1], m.k[2], m.k[3], m.k[4], m.k[MAT_MS], m.k[5], m.k[6]);

    for (vector<MeshTriangle *>::iterator it = t->triangles.begin(); it != t->triangles.end(); ++it)
    {
      Triangle * tri = new Triangle(
        t->vertices[ (**it).ind[0] ]->pos,
        t->vertices[ (**it).ind[1] ]->pos,
        t->vertices[ (**it).ind[2] ]->pos,
        m.color, mat, localToWorld);
      world->addPrimitive(tri);
    }
  }

  std::cout << "Imported scene file" << std::endl;
}

int
main(int argc, char ** argv)
{
  if (argc < 3)
  {
    std::cout << "Usage: " << argv[0] << " scene.scd output.png [max_trace_depth]" << std::endl;
    return -1;
  }

  if (argc >= 4)
    max_trace_depth = atoi(argv[3]);

  if (argc >= 5)
    width = atoi(argv[4]);
  
  if (argc >= 6)
    height = atoi(argv[5]);
  

  if (argc >= 7)
    length = atoi(argv[6]);

  tot_points = int(width*height*length);

  cout << "Max trace depth = " << max_trace_depth << endl;
  cout << "Max trace depth = " << width << endl;
  cout << "Max trace depth = " << height << endl;
  cout << "Max trace depth = " << length << endl;

  // Load the scene from the disk file
  scene = new Scene(argv[1]);

  // Setup the world object, containing the data from the scene
  world = new World();
  importSceneToWorld(scene->getRoot(), identity3D(), 0);
  world->printStats();

  // Set up the output framebuffer
  frame = new Frame(IMAGE_WIDTH, IMAGE_HEIGHT);

  // Render the world
  renderWithRaytracing();

  // Save the output to an image file
  frame->save(argv[2]);
  std::cout << "Image saved!" << std::endl;
}
