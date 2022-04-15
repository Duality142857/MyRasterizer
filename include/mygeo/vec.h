#pragma once
#include<iostream>
#include<array>
#include<assert.h>
#include<math.h>

namespace MyGeo{

template<class T,int N>
struct Vec;
using Vec3f=Vec<float,3>;
using Vec4f=Vec<float,4>;
using Vec3i=Vec<int,3>;
using Vec4i=Vec<int,4>;
using Vec2f=Vec<float,2>;
using Vec2i=Vec<int,2>;
using Vec3c=Vec<unsigned char,3>;

template<class T,int N>
std::ostream& operator<<(std::ostream& ostrm,const Vec<T,N>& v)
{
    for(const T& x:v.data)
    {
         ostrm<<x<<' ';
    }
    return ostrm;
}

template<class T,int N>
struct Vec
{
    union 
    {
        std::array<T,N> data;
        struct 
        {
            T x,y,z,w;
        };
        Vec<T,N-1> head;
    };
    Vec(){}  
    // Vec<T,3> head3()
    // {
    //     return Vec<T,m>{data[0],data[1],data[2]};
    // }
    bool nearZero()
    {
        for(const auto& x:data)
        {
            if(fabs(x)>1e-6) return false;
        }
        return true;
    }
    bool operator==(const Vec& v)
    {
        return data==v.data; 
    }
    bool operator!=(const Vec& v)
    {
        return !(operator==(v));
    }

    Vec(const Vec& v)
    {
        std::copy(v.data.begin(),v.data.end(),data.begin());
    }
    Vec(const Vec<T,N-1>& v,T x)
    {
        std::copy(v.data.begin(),v.data.end(),data.begin());
        data[N-1]=x;
    }

//行为不明，需要查
    Vec(std::initializer_list<T> il)
    {
        std::copy(il.begin(),il.end(),data.begin());
    }

	// Vec(const Vec& v) :data{ v.data } {}
	Vec& operator=(const Vec& v)
	{
		std::copy(v.data.begin(), v.data.end(), data.begin());
        return *this;
	}
    
	const T& operator[](int i) const
	{
		return data[i];
	}

    T& operator[](int i)
    {
        return data[i];
    }

    //scale vec itself
    void scale(const Vec& kv)
    {
        for(int i=0;i!=N;i++)
        {
            data[i]*=kv[i];
        }
    }

	T dot(const Vec& v) const
	{
		T sum{ 0 };
		for (int i = 0; i != N; i++) sum += data[i] * v.data[i];
		return sum;
	}
    Vec cwiseprod(const Vec& v)
    {
        Vec res;
        for(int i=0;i!=N;i++) res.data[i]=data[i]*v.data[i];
        return res;
    }
    
	Vec cross(const Vec& v) const
	{
        static_assert(N==3);
		return {
            data[1] * v.data[2] - data[2] * v.data[1],
            data[2] * v.data[0] - data[0] * v.data[2],
            data[0] * v.data[1] - data[1] * v.data[0]
        };
	}
    
    Vec& operator+=(const Vec& v)
    {
        for(int i=0;i!=N;i++) data[i]+=v[i];
        return *this;
    }

    Vec operator+(const Vec& v) const
    {
        Vec w=*this;
        w+=v;
        return w;
    }

    Vec operator-() const
    {
        Vec v;
        for(int i=0;i!=N;i++) 
        {
            v[i]=-data[i];
        }
        return v;
    }
    friend Vec operator-(const Vec& v1, const Vec& v2)
    {
        Vec v{v1};
        v-=v2;
        return v;
    }

    Vec& operator-=(const Vec& v)
    {
        for(int i=0;i!=N;i++) data[i]-=v[i];
        return *this;
    }

    Vec operator-(const Vec& v)
    {
        Vec w=*this;
        w-=v;
        return w;
    }

    Vec& operator*=(T k)
    {
        for(int i=0;i!=N;i++) data[i]*=k;
        return *this;
    }

    Vec operator*(T k) const
    {
        Vec vr{*this};
        return vr*=k;
    }
    friend Vec operator*(T k, Vec v)
    {
        return v*k;
    }
    Vec& operator/=(T k)
    {
        assert(k!=0);
        T a=1/k;
        return operator*=(a);
    }
    T norm2() const
    {
        T sum=0;
        for(T x:data) sum+=x*x;
        return sum;
    }

    T norm() const {return sqrt(norm2());}

    Vec& normalize()
    {
        T k=1/norm();
        for(T& x:data) x*=k;
        return *this; 
    }
    Vec normalize() const 
    {
        T k=1/norm();
        Vec res;
        for(int i=0;i!=data.size();i++)
        {
            res.data[i]=data[i]*k;
        }
        return res;
    }
    // template<class T,int N>
    friend std::ostream& operator<< <T,N>(std::ostream& ostrm,const Vec& v);

};

}

// using Vec3f=Vec<float,3>;

namespace MyGeo{
template<class T>
struct Vec<T,2>
{
    union 
    {
        std::array<T,2> data;
        struct 
        {
            T x,y;
        };
    };
    Vec(){}
    bool nearZero()
    {
        for(const auto& x:data)
        {
            if(fabs(x)>1e-6) return false;
        }
        return true;
    }
    bool operator==(const Vec& v)
    {
        return data==v.data; 
    }
    bool operator!=(const Vec& v)
    {
        return !(operator==(v));
    }
    Vec(const Vec& v)
    {
        std::copy(v.data.begin(),v.data.end(),data.begin());
    }
    Vec(const Vec<T,1>& v,T x)
    {
        std::copy(v.data.begin(),v.data.end(),data.begin());
        data[1]=x;
    }

//行为不明，需要查
    Vec(std::initializer_list<T> il)
    {
        std::copy(il.begin(),il.end(),data.begin());
    }

	// Vec(const Vec& v) :data{ v.data } {}
	Vec& operator=(const Vec& v)
	{
		std::copy(v.data.begin(), v.data.end(), data.begin());
        return *this;
	}
    
	const T& operator[](int i) const
	{
		return data[i];
	}

    T& operator[](int i)
    {
        return data[i];
    }

    //scale vec itself
    void scale(const Vec& kv)
    {
        for(int i=0;i!=2;i++)
        {
            data[i]*=kv[i];
        }
    }

	T dot(const Vec& v) const
	{
		T sum{ 0 };
		for (int i = 0; i != 2; i++) sum += data[i] * v.data[i];
		return sum;
	}

    Vec cwiseprod(const Vec& v)
    {
        Vec res;
        for(int i=0;i!=2;i++) res.data[i]=data[i]*v.data[i];
        return res;
    }
	// Vec cross(const Vec& v) const
	// {
    //     static_assert(N==3);
	// 	return {
    //         data[1] * v.data[2] - data[2] * v.data[1],
    //         data[2] * v.data[0] - data[0] * v.data[2],
    //         data[0] * v.data[1] - data[1] * v.data[0]
    //     };
	// }
    
    Vec& operator+=(const Vec& v)
    {
        for(int i=0;i!=2;i++) data[i]+=v[i];
        return *this;
    }

    Vec operator+(const Vec& v) const
    {
        Vec w=*this;
        w+=v;
        return w;
    }

    Vec operator-() const
    {
        Vec v;
        for(int i=0;i!=2;i++) 
        {
            v[i]=-data[i];
        }
        return v;
    }
    friend Vec operator-(const Vec& v1, const Vec& v2)
    {
        Vec v{v1};
        v-=v2;
        return v;
    }

    Vec& operator-=(const Vec& v)
    {
        for(int i=0;i!=2;i++) data[i]-=v[i];
        return *this;
    }

    Vec operator-(const Vec& v)
    {
        Vec w=*this;
        w-=v;
        return w;
    }

    Vec& operator*=(T k)
    {
        for(int i=0;i!=2;i++) data[i]*=k;
        return *this;
    }

    Vec operator*(T k) const
    {
        Vec vr{*this};
        return vr*=k;
    }
    friend Vec operator*(T k, Vec v)
    {
        return v*k;
    }
    Vec& operator/=(T k)
    {
        assert(k!=0);
        T a=1/k;
        return operator*=(a);
    }
    T norm2() const
    {
        T sum=0;
        for(T x:data) sum+=x*x;
        return sum;
    }

    T norm() const
    {return sqrt(norm2());}

    Vec& normalize()
    {
        T k=1/norm();
        for(T& x:data) x*=k;
        return *this; 
    }
    Vec normalize() const 
    {
        T k=1/norm();
        Vec res;
        for(int i=0;i!=data.size();i++)
        {
            res.data[i]=data[i]*k;
        }
        return res;
    }
    // template<class T,int N>
    friend std::ostream& operator<< <T,2>(std::ostream& ostrm,const Vec& v);

};
template<class T>
struct Vec<T,3>
{
    union 
    {
        std::array<T,3> data;
        struct 
        {
            T x,y,z;
        };
        Vec<T,2> head;

    };
    Vec(){}
    bool nearZero()
    {
        for(const auto& x:data)
        {
            if(fabs(x)>1e-6) return false;
        }
        return true;
    }
    bool operator==(const Vec& v)
    {
        return data==v.data; 
    }
    bool operator!=(const Vec& v)
    {
        return !(operator==(v));
    }
    Vec(const Vec& v)
    {
        std::copy(v.data.begin(),v.data.end(),data.begin());
    }
    Vec(const Vec<T,2>& v,T x)
    {
        std::copy(v.data.begin(),v.data.end(),data.begin());
        data[2]=x;
    }

//行为不明，需要查
    Vec(std::initializer_list<T> il)
    {
        std::copy(il.begin(),il.end(),data.begin());
    }

	// Vec(const Vec& v) :data{ v.data } {}
	Vec& operator=(const Vec& v)
	{
		std::copy(v.data.begin(), v.data.end(), data.begin());
        return *this;
	}
    
	const T& operator[](int i) const
	{
		return data[i];
	}

    T& operator[](int i)
    {
        return data[i];
    }

    //scale vec itself
    void scale(const Vec& kv)
    {
        for(int i=0;i!=3;i++)
        {
            data[i]*=kv[i];
        }
    }

	T dot(const Vec& v) const 
	{
		T sum{ 0 };
		for (int i = 0; i != 3; i++) sum += data[i] * v.data[i];
		return sum;
	}
    Vec cwiseprod(const Vec& v)
    {
        Vec res;
        for(int i=0;i!=3;i++) res.data[i]=data[i]*v.data[i];
        return res;
    }
	Vec cross(const Vec& v) const
	{
		return {
            data[1] * v.data[2] - data[2] * v.data[1],
            data[2] * v.data[0] - data[0] * v.data[2],
            data[0] * v.data[1] - data[1] * v.data[0]
        };
	}
    
    Vec& operator+=(const Vec& v)
    {
        for(int i=0;i!=3;i++) data[i]+=v[i];
        return *this;
    }

    Vec operator+(const Vec& v) const
    {
        Vec w=*this;
        w+=v;
        return w;
    }

    Vec operator-() const
    {
        Vec v;
        for(int i=0;i!=3;i++) 
        {
            v[i]=-data[i];
        }
        return v;
    }
    friend Vec operator-(const Vec& v1, const Vec& v2)
    {
        Vec v{v1};
        v-=v2;
        return v;
    }

    Vec& operator-=(const Vec& v)
    {
        for(int i=0;i!=3;i++) data[i]-=v[i];
        return *this;
    }

    Vec operator-(const Vec& v)
    {
        Vec w=*this;
        w-=v;
        return w;
    }

    Vec& operator*=(T k)
    {
        for(int i=0;i!=3;i++) data[i]*=k;
        return *this;
    }

    Vec operator*(T k) const
    {
        Vec vr{*this};
        return vr*=k;
    }
    friend Vec operator*(T k, Vec v)
    {
        return v*k;
    }
    Vec& operator/=(T k)
    {
        assert(k!=0);
        T a=1/k;
        return operator*=(a);
    }
    T norm2() const
    {
        T sum=0;
        for(T x:data) sum+=x*x;
        return sum;
    }

    T norm() const {return sqrt(norm2());}

    Vec& normalize()
    {
        T k=1/norm();
        for(T& x:data) x*=k;
        return *this; 
    }
    Vec normalize() const 
    {
        T k=1/norm();
        Vec res;
        for(int i=0;i!=data.size();i++)
        {
            res.data[i]=data[i]*k;
        }
        return res;
    }
    // template<class T,int N>
    friend std::ostream& operator<< <T,3>(std::ostream& ostrm,const Vec& v);

};
}