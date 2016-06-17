//
// raytrace.cpp
//

#define _CRT_SECURE_NO_WARNINGS
#include "matm.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
using namespace std;

int g_width;
int g_height;

struct Ray
{
  vec4 origin;
  vec4 dir;
};

struct Sphere
{
  Sphere(string name, vec3 pos, vec3 scale, vec3 color, float k_a, float k_d, float k_s, float k_r, float n)
    :name(name), M(Translate(pos) * Scale(scale)), pos(pos), color(color), k_a(k_a), k_d(k_d), k_s(k_s), k_r(k_r), n(n)
  {
      InvertMatrix(M, M_i);
  }

  string name;
  mat4 M;
  mat4 M_i;
  vec3 pos;
  vec3 color;
  float k_a, k_d, k_s, k_r, n;
};

vector<Sphere> spheres;

struct Light
{
  Light(string name, vec3 pos, vec3 intensity)
    :name(name), pos(pos), intensity(intensity)
  {
  }

  string name;
  vec3 pos;
  vec3 intensity;
};

vector<Light> lights;

struct Intersection
{
  bool hit;
  vec3 pos;
  Sphere *sphere;
  float time;
};

vec3 g_back;
vec3 g_ambient;
char* g_output;

vector<vec4> g_colors;

float g_left;
float g_right;
float g_top;
float g_bottom;
float g_near;


// -------------------------------------------------------------------
// Input file parsing

vec4 toVec4(const string& s1, const string& s2, const string& s3)
{
  stringstream ss(s1 + " " + s2 + " " + s3);
  vec4 result;
  ss >> result.x >> result.y >> result.z;
  result.w = 1.0f;
  return result;
}

float toFloat(const string& s)
{
  stringstream ss(s);
  float f;
  ss >> f;
  return f;
}

void parseLine(const vector<string>& vs)
{
  if (vs[0] == "NEAR")
  {
    g_near = toFloat(vs[1]);
  }

  if (vs[0] == "LEFT")
  {
    g_left = toFloat(vs[1]);
  }

  if (vs[0] == "RIGHT")
  {
    g_right = toFloat(vs[1]);
  }

  if (vs[0] == "BOTTOM")
  {
    g_bottom = toFloat(vs[1]);
  }

  if (vs[0] == "TOP")
  {
    g_top = toFloat(vs[1]);
  }

  if (vs[0] == "RES")
  {
    g_width = (int)toFloat(vs[1]);
    g_height = (int)toFloat(vs[2]);
    g_colors.resize(g_width * g_height);
  }

  if (vs[0] == "SPHERE")
  {
    string new_name = vs[1];
    vec3 new_pos = vec3(toFloat(vs[2]), toFloat(vs[3]), toFloat(vs[4]));
    vec3 new_scale = vec3(toFloat(vs[5]), toFloat(vs[6]), toFloat(vs[7]));
    vec3 new_color = vec3(toFloat(vs[8]), toFloat(vs[9]), toFloat(vs[10]));
    float new_k_a = toFloat(vs[11]);
    float new_k_d = toFloat(vs[12]);
    float new_k_s = toFloat(vs[13]);
    float new_k_r = toFloat(vs[14]);
    float new_n = toFloat(vs[15]);
    Sphere new_sphere = Sphere(new_name, new_pos, new_scale, new_color, new_k_a, new_k_d, new_k_s, new_k_r, new_n);
    spheres.push_back(new_sphere);
  }

  if (vs[0] == "LIGHT")
  {
    string new_name = vs[1];
    vec3 new_pos = vec3(toFloat(vs[2]), toFloat(vs[3]), toFloat(vs[4]));
    vec3 new_intensity = vec3(toFloat(vs[5]), toFloat(vs[6]), toFloat(vs[7]));
    Light new_light = Light(new_name, new_pos, new_intensity);
    lights.push_back(new_light);
  }

  if (vs[0] == "BACK")
  {
    g_back = vec3(toFloat(vs[1]), toFloat(vs[2]), toFloat(vs[3]));
  }

  if (vs[0] == "AMBIENT")
  {
    g_ambient = vec3(toFloat(vs[1]), toFloat(vs[2]), toFloat(vs[3]));
  }

  if (vs[0] == "OUTPUT")
  {
    g_output = new char[vs[1].length() + 1];
    strcpy(g_output, vs[1].c_str());
    g_output[vs[1].length()] = '\0';
  }
}

void loadFile(const char* filename)
{
  ifstream is(filename);
  if (is.fail())
  {
    cout << "Could not open file " << filename << endl;
    exit(1);
  }
  string s;
  vector<string> vs;
  while(!is.eof())
  {
    vs.clear();
    getline(is, s);
    istringstream iss(s);
    while (!iss.eof())
    {
      string sub;
      iss >> sub;
      vs.push_back(sub);
    }
    parseLine(vs);
  }
}


// -------------------------------------------------------------------
// Utilities

void setColor(int ix, int iy, const vec4& color)
{
  int iy2 = g_height - iy - 1; // Invert iy coordinate.
  g_colors[iy2 * g_width + ix] = color;
}

vec3 toVec3(vec4 in)
{
  return vec3( in[0], in[1], in[2] );
}

// -------------------------------------------------------------------
// Intersection routine

Intersection intersect(const Ray& ray, bool light = false, bool check_near = true)
{
  // Create an intersection object
  Intersection closest;
  closest.hit = false;
  for(int i = 0; i < spheres.size(); i++)
  {
    // Get the origin and direction of the ray
    vec3 S = toVec3(ray.origin);
    vec3 c = toVec3(ray.dir);

    // Get the inverses of the origin and direction of the ray
    vec3 S_p = toVec3(spheres[i].M_i * ray.origin);
    vec3 c_p = toVec3(spheres[i].M_i * ray.dir);

    // Calculate the discriminant
    float disc = dot(S_p, c_p) * dot(S_p, c_p) - dot(c_p, c_p) * (dot(S_p, S_p) - 1);

    // Create a temporary intersection object
    Intersection hit;
    if(disc >= 0.0f)
    {
      // Calculate intersection times
      float t1 = -dot(S_p, c_p)/dot(c_p, c_p) + sqrt(disc)/dot(c_p, c_p);
      float t2 = -dot(S_p, c_p)/dot(c_p, c_p) - sqrt(disc)/dot(c_p, c_p);

      // If the ray is a shadow ray, and an intersection time is within the set boundaries, immediately return
      if(light && ((t1 >= 0.0001 && t1 <= 1) || (t2 >= 0.0001 && t2 <= 1)))
      {
        closest.hit = true;
        return closest;
      }

      // If considering the near plane:
      if(check_near)
      {
        // Find the smallest intersection point that is past the near plane
        if(t1 <= 1 || (t1 > t2 && t2 > 1))
          t1 = t2;
        if(t1 <= 1)
          hit.hit = false;
        else
        {
          hit.pos = toVec3(S + t1 * c);
          if(closest.hit == false || t1 < closest.time)
          {
            closest.hit = true;
            closest.pos = hit.pos;
            closest.sphere = &spheres[i];
            closest.time = t1;
          }
        }
      }

      // If not considering the near plane:
      else
      {
        // Find the smallest intersection time that is greater than 0 
        //t1 = abs(t1);
        //t2 = abs(t2);
        if(t1 <= 0.0001 || (t1 > t2 && t2 > 0.0001))
          t1 = t2;
        if(t1 <= 0.0001)
          hit.hit = false;
        else
        {
          hit.pos = toVec3(S + t1 * c);
          if(closest.hit == false || t1 < closest.time)
          {
            closest.hit = true;
            closest.pos = hit.pos;
            closest.sphere = &spheres[i];
            closest.time = t1;
          }
        }
      }
    }
  }
  return closest;
}

bool shadow_ray(Light light, Intersection hit){

  // Create a ray from the hit position to the light position
  Ray ray;
  ray.origin = hit.pos;
  ray.dir = vec4(normalize(vec3(light.pos - hit.pos)), 0);

  // Check for intersections with objects
  Intersection shadow = intersect(ray, true);

  if(shadow.hit)
    return true;
  return false;
}

// -------------------------------------------------------------------
// Ray tracing

vec4 trace(const Ray& ray, int depth = 1)
{
  // Find the closest intersection object
  Intersection hit = intersect(ray, false, (depth == 1));
  vec4 pixel_color = vec4(0, 0, 0, 1);
  if(hit.hit)
  {
    // Get the sphere that was hit
    Sphere cur = *hit.sphere;

    // Set ambient color
    pixel_color = cur.k_a * g_ambient * cur.color;

    // Calculate normal vector
    vec3 S_p = toVec3(cur.M_i * ray.origin);
    vec3 c_p = toVec3(cur.M_i * ray.dir);
    vec3 N = normalize(toVec3(transpose(cur.M_i) * (S_p + hit.time * c_p)));

    // Calculating viewing vector
    vec3 V = normalize(vec3(toVec3(ray.origin) - hit.pos));

    // Create a light ray
    Ray light_ray;
    light_ray.origin = hit.pos;
    for(int i = 0; i < lights.size(); i++)
    {
      // Set the light ray direction
      light_ray.dir = normalize(toVec3(lights[i].pos - hit.pos));
      vec3 L = normalize(vec3(lights[i].pos - hit.pos));

      // Set the reflection ray
      vec3 R = normalize(vec3(toVec3(2*(dot(L, N) * N) - L)));

      // Check for intersections between the light ray and an object
      if(!shadow_ray(lights[i], hit))
      {
        // Set diffuse color
        if(dot(N, L) > 0)
          pixel_color += cur.k_d * lights[i].intensity * dot(N, L) * cur.color;
        
        // Set specular color
        if(dot(R, V) > 0)
          pixel_color += cur.k_s * lights[i].intensity * pow(dot(R, V), cur.n);
      }
    }

    // Set the reflection ray
    vec3 R = normalize(vec3(toVec3(2*(dot(V, N) * N) - V)));

    // Recursively check for reflections
    if(depth <= 3){
      Ray new_ray;
      new_ray.origin = vec4(hit.pos, 1);
      new_ray.dir = vec4(R, 0);
      pixel_color += cur.k_r * trace(new_ray, depth + 1);
    }

    return pixel_color;
  }
  if(depth == 1)
    return g_back;
  else
    return vec4(0, 0, 0, 0);
}

vec4 getDir(int ix, int iy)
{
  vec4 dir;
  float a = ((ix - g_width/2.)/g_width) * (g_right - g_left);
  float b = ((iy - g_height/2.)/g_height) * (g_top - g_bottom);
  float dist = sqrt(a*a + b*b + g_near*g_near);
  dist = 1;
  dir = vec4(a/dist, b/dist, -g_near/dist, 0.0f);
  return dir;
}

void renderPixel(int ix, int iy)
{
  Ray ray;
  ray.origin = vec4(0.0f, 0.0f, 0.0f, 1.0f);
  ray.dir = getDir(ix, iy);
  vec4 color = trace(ray);
  setColor(ix, iy, color);
}

void render()
{
  for (int iy = 0; iy < g_height; iy++)
    for (int ix = 0; ix < g_width; ix++)
      renderPixel(ix, iy);
}


// -------------------------------------------------------------------
// PPM saving

void savePPM(int Width, int Height, char* fname, unsigned char* pixels) 
{
  FILE *fp;
  const int maxVal=255;

  printf("Saving image %s: %d x %d\n", fname, Width, Height);
  fp = fopen(fname,"wb");
  if (!fp) {
    printf("Unable to open file '%s'\n", fname);
    return;
  }
  fprintf(fp, "P6\n");
  fprintf(fp, "%d %d\n", Width, Height);
  fprintf(fp, "%d\n", maxVal);

  for(int j = 0; j < Height; j++) {
    fwrite(&pixels[j*Width*3], 3, Width, fp);
  }

  fclose(fp);
}

void saveFile()
{
  // Convert color components from floats to unsigned chars.
  unsigned char* buf = new unsigned char[g_width * g_height * 3];
  for (int y = 0; y < g_height; y++)
    for (int x = 0; x < g_width; x++)
      for (int i = 0; i < 3; i++){
        if(g_colors[y*g_width+x][i] < 0)
          g_colors[y*g_width+x][i] *= -1;
        if(g_colors[y*g_width+x][i] > 1)
          g_colors[y*g_width+x][i] = 1;
        buf[y*g_width*3+x*3+i] = (unsigned char)(((float*)g_colors[y*g_width+x])[i] * 255.9f);
      }

  savePPM(g_width, g_height, g_output, buf);
  delete[] buf;
}


// -------------------------------------------------------------------
// Main

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    cout << "Usage: raytrace <input_file.txt>" << endl;
    exit(1);
  }
  loadFile(argv[1]);
  render();
  saveFile();
  return 0;
}

