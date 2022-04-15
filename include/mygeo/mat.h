#pragma once
#include"vec.h"

namespace MyGeo{
template<class T,int M, int N>
struct Mat;

using Mat3f=Mat<float,3,3>;
using Mat4f=Mat<float,4,4>;

template<class T,int M, int N>
std::ostream& operator<<(std::ostream& ostr,const Mat<T,M,N>& m)
{
    for(int i=0;i!=N;i++)
    {
        ostr<<m.col[i]<<std::endl;
    }
    return ostr;
}

template<class T,int N>
Mat<T,N,N> Eye()
{
    Mat<T,N,N> m;
    for(int i=0;i!=N;i++) 
    for(int j=0;j!=N;j++)
    {
        if(j==i) m.col[i][i]=1;
        else m.col[i][j]=0;
    }
    return m;
}

template<class T,int M, int N>
struct Mat
{
    std::array<Vec<T,M>,N> col;

    Mat& operator+=(const Mat& m)
    {
        for(int i=0;i!=N;i++)
        {
            col[i]+=m.col[i];
        } 
        return *this;
    }
    Mat operator+(const Mat& m)
    {
        Mat mr=*this;
        mr+=m;
        return mr;
    }
    Mat operator-()
    {
        Mat mr;
        for(int i=0;i!=N;i++) mr.col[i]=-col[i];
        return mr;
    }
    Mat operator-(const Mat& m)
    {
        Mat mr=m;
        return *this+(-mr);
        
        // return -m;
    }

    Mat& operator-=(const Mat& m)
    {
        *this+=-*this;
        return *this;
    }
    Mat& operator*=(T k)
    {
        for(auto& vc:col) vc*=k;
        return *this;
    }
    Mat operator*(T k)
    {
        Mat mr=*this;
        mr*=k;
        return mr;        
    }

    Mat<T,N,M> transpose() const 
    {
        Mat<T,N,M> m;
        for(int i=0;i!=M;i++)
        for(int j=0;j!=N;j++)
        {
            m.col[j][i]=col[i][j];
        }
        return m;
    }

    Vec<T,M> operator*(const Vec<T,N>& v) const
    {
        // std::cout<<"v: "<<v<<std::endl;
        Vec<T,M> vr;//{0,0,0};
        vr.data.fill(0);
        for(int i=0;i!=N;i++) vr+=col[i]*v[i];
        // std::cout<<&(vr.data[1])<<std::endl;
        return vr;
    }
    template<int L>
    Mat<T,M,L> operator*(const Mat<T,N,L>& m) const 
    {
        Mat<T,M,L> mr;
        for(int i=0;i!=L;i++) mr.col[i]=*this*m.col[i];
        // std::cout<<&(mr.col[1][1])<<std::endl;
        return mr;
    }
    

    friend std::ostream& operator<<<T,M,N>(std::ostream& ostr,const Mat&);

};

}


