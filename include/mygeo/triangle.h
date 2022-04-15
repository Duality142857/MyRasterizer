#include"geo.h"
namespace MyGeo{

struct Triangle
{
    union 
    {
        std::array<Vec3f,3> vertices;
        struct 
        {
            float x0,y0,z0,x1,y1,z1,x2,y2,z2;
        };
    };
    
    
    float getArea()
    {
        float s=(y0-y2)*(x1-x2)+(y1-y2)*(x2-x0);
        return 0.5f*s;
    }
};



}