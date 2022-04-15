#pragma once
#include"mat.h"
#include"vec.h"
#include<math.h>

namespace MyGeo{


float clamp(float r,float min, float max)
{
    if(r<min) return min;
    if(r>max) return max;
    return r;
}

Vec3f reflect(Vec3f l, Vec3f n)
{
    return -l+n*2*(l.dot(n));
}

Vec3f halfway(Vec3f l, Vec3f e)
{
    return (e+l)*(1.0f/(e+l).norm());
}











constexpr float toRad(float x)
{
    return x*3.1415927/180.0;
}
struct Camera;
static Mat4f viewMatrix(const Camera& cam);
static Mat4f translateMatrix(const Vec3f& v)
{
    return Mat4f{Vec4f{1,0,0,0},{0,1,0,0},{0,0,1,0},{v.x,v.y,v.z,1}};
}
static Mat4f scaleMatrix(const Vec3f& k)
{
    return Mat4f{Vec4f{k[0],0,0,0},{0,k[1],0,0},{0,0,k[2],0},{0,0,0,1}};
}
static Mat4f rotationMatrix(const Vec3f& axis, float angle)
{
    Vec3f nAxis=axis;
    nAxis.normalize();
    // std::cout<<nAxis<<std::endl;
    float C=std::cos(toRad(angle));
    float R=1-C;
    float S=std::sin(toRad(angle));
    float a1=nAxis.x;
    float a2=nAxis.y;
    float a3=nAxis.z;
    return Mat4f{Vec4f{C+R*a1*a1,R*a1*a2+S*a3,R*a1*a3-S*a2,0},
            Vec4f{R*a1*a2-S*a3,C+R*a2*a2,R*a2*a3+S*a1,0},
            Vec4f{R*a1*a3+S*a2,R*a2*a3-S*a1,C+R*a3*a3,0},
            Vec4f{0,0,0,1}
            };   
}

static Mat3f rotationMatrix3f(const Vec3f& axis, float angle)
{
    Vec3f nAxis=axis.normalize();
    float C=std::cos(toRad(angle));
    float R=1-C;
    float S=std::sin(toRad(angle));
    float a1=nAxis.x;
    float a2=nAxis.y;
    float a3=nAxis.z;

    return Mat3f{Vec3f{C+R*a1*a1,R*a1*a2+S*a3,R*a1*a3-S*a2},
            Vec3f{R*a1*a2-S*a3,C+R*a2*a2,R*a2*a3+S*a1},
            Vec3f{R*a1*a3+S*a2,R*a2*a3-S*a1,C+R*a3*a3}
            };  
    
}

struct Vertex3D_C
{
    union 
    {
        Vec3f position;
        struct 
        {
            float x,y,z;
        };
    };
    Vec3f color;
    Vertex3D_C(){}
    Vertex3D_C(const Vec3f& v,const Vec3f& c)
    {
        position=v;color=c;
    }
    Vertex3D_C(const Vertex3D_C& vc):position{vc.position},color{vc.color}{}
};

struct Point3D
{
    union 
    {
        Vec3f data;
        struct 
        {
            float x,y,z;
        };
    };
    bool operator==(const Point3D & p)
    {
        return data==p.data;
    }
    Point3D(){}
    Point3D(std::initializer_list<float> il):data{il}{}
    Point3D(float _x,float _y,float _z):x{_x},y{_y},z{_z}{}
    void operator+=(const Point3D& d)
    {
        data+=d.data;
    }
    Point3D(const Point3D& p)
    {
        data=p.data;    
    }
    Point3D(const Vec3f& v)
    {
        data=v;
    }

    Vec3f vec() const 
    {
        return data;
    }

    Point3D& operator=(const Point3D& p)
    {
        data=p.data;
        return *this;
    }

    Point3D translate(const Vec3f& v) const
    {
        Point3D p{*this};
        p.data+=v;
        return p;
    }


    Point3D rotate(const Vec3f& axis, float angle);
    Point3D scale(float k)
    {
        return Point3D{this->data*k};
    }

    Point3D viewtransform(const Camera& cam);

    Point3D perspectiveProj(Camera& cam);
    


    friend std::ostream& operator<<(std::ostream& ostr, const Point3D& p)
    {
        return ostr<<p.data;
    }
};

struct CoordinateSystem
{
    // std::array<Vec3f,3> axis;
    Mat3f axis;
    CoordinateSystem(std::initializer_list<Vec3f> il)
    {
        std::copy(il.begin(),il.end(),axis.col.begin());
    }
};



struct Camera
{
    Point3D position;
    Vec3f lookdirection;
    Vec3f updirection;
    Vec3f lookat;//look at point!
    float n=-0.1f;
    float f=-20.f;
    float l;
    float r;
    float t; 
    float b;
    Mat4f projMat;
    Mat4f viewMat;
    // Camera(const Point3D& pos, const Vec3f& look, const Vec3f& up): position{pos},lookdirection{look},updirection{up}
    // {
    //     lookdirection.normalize();
    //     updirection.normalize();
    // }

    Camera(const Point3D& pos, const Vec3f& lookat, const Vec3f& up): position{pos},lookat{lookat},updirection{up}
    {
        lookdirection=lookat-position.data;//-z

        lookdirection.normalize();
        updirection.normalize();
    }


    void setNearFar(float near,float far)
    {
        n=near;
        f=far;
    }
    //field of view, represented by view degree in Y direction and the ratio to get in X
    void setFov(float fovY,float aspect)
    {
        // l=0.5*n*std::sin(toRad(0.5*fovX));
        // r=-l;
        t=n*std::tan(toRad(0.5*fovY));
        b=-t;
        l=aspect*b;
        r=-l;
    }
    void updateMat()
    {
        //[0,1]
        // projMat=Mat4f{
        //         Vec4f{2*n/(r-l),0,0,0},
        //         Vec4f{0,2*n/(b-t),0,0},
        //         Vec4f{(l+r)/(l-r),(b+t)/(b-t),f/(n-f),-1},
        //         Vec4f{0,0,n*f/(f-n),0}
        //         };
        projMat=Mat4f{
                Vec4f{2*n/(r-l),0,0,0},
                Vec4f{0,2*n/(t-b),0,0},
                Vec4f{(l+r)/(l-r),(b+t)/(b-t),(f+n)/(f-n),1},
                Vec4f{0,0,2*n*f/(n-f),0}
                };
        viewMat=viewMatrix(*this);
    }
};

Point3D Point3D::rotate(const Vec3f& axis, float angle)
{

    float C=std::cos(toRad(angle));
    float R=1-C;
    float S=std::sin(toRad(angle));
    float a1=axis.x;
    float a2=axis.y;
    float a3=axis.z;
    Mat3f mr{Vec3f{C+R*a1*a1,R*a1*a2+S*a3,R*a1*a3-S*a2},
            Vec3f{R*a1*a2-S*a3,C+R*a2*a2,R*a2*a3-S*a1},
            Vec3f{R*a1*a3-S*a2,R*a2*a3+S*a1,C+R*a3*a3}
            };
    return Point3D{mr*this->data};
}


//roll: z(opengl -z) look direction  滚动
//yaw: y, up direction 偏航
//pitch: x, side direction 坠落（仰俯）
Point3D Point3D::viewtransform(const Camera& cam)
{
    Point3D p{*this};
    p=p.translate(-cam.position.vec());
    Vec3f sidedirection=cam.lookdirection.cross(cam.updirection).normalize();
    // Vec3f up_normalized=sidedirection.cross(cam.lookdirection);
    CoordinateSystem cs{sidedirection,cam.updirection,-cam.lookdirection};
    return {cs.axis*p.vec()};
}


Point3D Point3D::perspectiveProj(Camera& cam)
{
    Vec4f p{data.x,data.y,data.z,1};
    // std::cout<<"p: "<<p<<std::endl;
    Vec4f a=cam.projMat*p;
    // std::cout<<a<<std::endl;
    float rw=1/a.w;
    return Point3D{a.x*rw,a.y*rw,a.z*rw};
}

// Point3D Point3D::orthographicTransform(const Camera& cam)
// {
//     Point3D p{*this};
//     p=p.translate(Vec3f{cam.left+cam.right,cam.top+cam.bottom,cam.near+cam.far}*(-0.5f));
//     std::cout<<p<<std::endl;
//     p.data.scale(Vec3f{2.f/(cam.right-cam.left),2.f/(cam.top-cam.bottom),2.f/(cam.far-cam.near)});
//     std::cout<<p<<std::endl;
//     return p;
// }

static Mat4f viewMatrix(const Camera& cam)
{
    // Point3D p{*this};
    // p=p.translate(-cam.position.vec());
    Vec3f sidedirection=cam.lookdirection.cross(cam.updirection).normalize();
    Vec3f up_normalized=sidedirection.cross(cam.lookdirection);
    // std::cout<<"x "<<sidedirection<<std::endl;
    // std::cout<<"y "<<up_normalized<<std::endl;
    // std::cout<<"z "<<-cam.lookdirection<<std::endl;
    // std::cout<<"sidedirection: "<<sidedirection<<std::endl;
    // std::cout<<"updirection: "<<cam.updirection<<std::endl;
    // std::cout<<"-lookdirection: "<<-cam.lookdirection<<std::endl;
    // CoordinateSystem cs{{sidedirection.x,cam.updirection.x,-cam.lookdirection.x},cam.updirection,-cam.lookdirection};
    return Mat4f{
        Vec4f{sidedirection.x,up_normalized.x,-cam.lookdirection.x,0},
        Vec4f{sidedirection.y,up_normalized.y,-cam.lookdirection.y,0},
        Vec4f{sidedirection.z,up_normalized.z,-cam.lookdirection.z,0},
        {0,0,0,1}
        }*translateMatrix(-cam.position.vec());
}


}