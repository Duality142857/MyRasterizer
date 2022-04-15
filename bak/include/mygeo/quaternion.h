#include"vec.h"
template<class T>
struct Quaternion;
template<class T>
std::ostream& operator<<(std::ostream& ostr,const Quaternion<T>& q)
{
    ostr<<q.w<<" ("<<q.x<<','<<q.y<<','<<q.z<<')';
    return ostr;
}

template<class T>
struct Quaternion
{
//data
    T w;
    union
    {
        Vec<T,3> v;
        struct 
        {
            T x,y,z;
        };
    };
//
    Quaternion():w{0},v{0,0,0}{}
    Quaternion(T _w,std::initializer_list<T> vil):w{_w},v{vil}{}
    Quaternion(T _w,const Vec<T,3>& _v):w{_w},v{_v}{}
    Quaternion(const Quaternion& q):w{q.w},v{q.v}{}
    Quaternion& operator=(const Quaternion& q)
    {
        w=q.w;
        v=q.v;
    }

//friends
    friend std::ostream& operator<<<T>(std::ostream& ostr,const Quaternion& q);

};
