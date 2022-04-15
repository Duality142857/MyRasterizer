#pragma once
#include<mygeo/vec.h>
#include<mygeo/geo.h>
#include<mygeo/mat.h>
#include<algorithm>
#include<iostream>
#include<vector>
#include<functional>
#include<optional>
#include<opencv2/opencv.hpp>
#include"../sdlapp.h"
#include<thread>
#include<mutex>
static std::mutex pixelMtx;

#define USE_SDL

namespace MyRasterizer{
struct Vertex
{
    MyGeo::Vec4f pos;
    // MyGeo::Vec3f color;
    MyGeo::Vec2f textureCoord;
    MyGeo::Vec4f normal;
};
struct ShaderMemories
{
    void* vInMemory=nullptr;
    void* vOutMemory=nullptr;
    void* fInMemory=nullptr;
    void* textureMemory=nullptr;
    // void* fOutMemory;
};
struct FragmentData
{
    MyGeo::Vec4f position;
    // MyGeo::Vec3f color;
    MyGeo::Vec4f normal;
    MyGeo::Vec2f texCoord;
    MyGeo::Vec2i screenCoord;
    float depth;
};

struct Texture
{
    cv::Mat imageData;
    int width,height;
    Texture(){}
    Texture(const std::string& filename)
    {
        imageData=cv::imread(filename);
        cv::cvtColor(imageData,imageData,cv::COLOR_RGB2BGR);
        width=imageData.cols;
        height=imageData.rows;
        
    }
    MyGeo::Vec3c getColor(float u, float v)
    {
        auto ui=u*width;
        auto vi=(1-v)*height;
        auto color=imageData.at<cv::Vec3b>(vi,ui);
        return {color[0],color[1],color[2]};
    }
    void attachImage(const std::string& filename)
    {
        imageData=cv::imread(filename);
        cv::cvtColor(imageData,imageData,cv::COLOR_RGB2BGR);
        width=imageData.cols;
        height=imageData.rows;
        std::cout<<width<<'-'<<height<<std::endl;
    }
};

static bool getSign(float x)
{
    if(x>=0) return true;
    else return false;
}

    static int count1=0;
    static int count2=0;
struct Canvas
{
    float width,height;
    std::optional<Texture> texture;
    std::optional<Texture> hmap;
    WindowContext* sdlContext;
    std::vector<MyGeo::Vec3c> framebuffer;
    std::vector<float> zbuffer;
    Canvas(float w,float h):width{w},height{h}
    {
#ifdef USE_SDL
        sdlContext=new WindowContext(width,height,"Canvas");
        framebuffer.resize(w*h);
        std::fill(framebuffer.begin(),framebuffer.end(),MyGeo::Vec3c{0,0,0});
#else
        framebuffer.resize(w*h);
        std::fill(framebuffer.begin(),framebuffer.end(),MyGeo::Vec3c{0,0,0});
#endif
        zbuffer.resize(w*h);
        std::fill(zbuffer.begin(),zbuffer.end(),-10000);
        
    }
    void setTexture(Texture t)
    {
        texture=t;
    }
    void setHeightTexture(Texture t)
    {
        hmap=t;
    }

    void clear()
    {
#ifndef USE_SDL
        std::fill(framebuffer.begin(),framebuffer.end(),MyGeo::Vec3c{0,0,0});
#endif
        std::fill(framebuffer.begin(),framebuffer.end(),MyGeo::Vec3c{0,0,0});
        std::fill(zbuffer.begin(),zbuffer.end(),-10000);
    }
    inline void setPixel(const MyGeo::Vec2i& point, const MyGeo::Vec3c& color)
    {
        if (point.x < 0 || point.x >= width ||
            point.y < 0 || point.y >= height) return;
#ifdef USE_SDL
        // sdlContext->drawPixel(point.x,point.y,color);
        int ind=point.y*width+point.x;
        framebuffer[ind]=color;
#else 
        int ind=point.y*width+point.x;
        framebuffer[ind]=color;
#endif
    }
    void drawSDL()
    {
        for(int y=0;y!=height;y++)
        for(int x=0;x!=width;x++)
        {
            auto ind=y*width+x;
            if(framebuffer[ind]!=MyGeo::Vec3c{0,0,0})sdlContext->drawPixel(x,y,framebuffer[ind]);
        }
    }

    void drawLine(MyGeo::Vec2i src,MyGeo::Vec2i dst)
    {
        MyGeo::Vec3c lineColor{255,255,255};
        auto dr=dst-src;
        int dx_abs=std::abs(dr.x);
        int dy_abs=std::abs(dr.y);
        int tx=2*dx_abs;
        int ty=2*dy_abs;
        int a=2*dy_abs-dx_abs;//a0=-|dx|,a1=2|dy|-|dx|
        int b=2*dx_abs-dy_abs;
        //横向多于纵向
        if(dx_abs>=dy_abs)
        {
            // log("横向");
            if(dr.x<0) std::swap(src,dst);
            dr=dst-src;
            int x=src.x;
            // std::cout<<src<<std::endl;
            int y=src.y;
            int inc=1;
            if(dr.y<0) inc=-1;
            while(x<=dst.x)
            {
                setPixel({x,y},lineColor);
                x++;
                //a大于0说明y方向进展超过阈值,选择斜着走，并且更新a需要截断
                if(a>=0)
                {
                    y+=inc;
                    a+=ty-tx;
                }
                else a+=ty;  //选择横着走，更新a不需要截断。
            }
        }
        else //纵向多于横向，
        {
            // log("纵向");
            if(dr.y<0) std::swap(src,dst);
            dr=dst-src;
            int x=src.x;
            int y=src.y;
            int inc=1;
            if(dr.x<0) inc=-1;
            while(y<=dst.y)
            {
                setPixel({x,y},lineColor);
                y++;
                if(b>=0)
                {
                    x+=inc;
                    b+=tx-ty;
                }
                else b+=tx;
            }
        }
    }
    void drawTrianglesFilled(
        uint32_t numTriangles,
        MyGeo::Mat4f& modelMat,
        MyGeo::Camera& camera,
        std::function<void(MyGeo::Mat4f& modelMat, 
            MyGeo::Camera& camera,
            ShaderMemories& sm,
            size_t ind)> 
            vertexShader,
        std::function<MyGeo::Vec3c(
            ShaderMemories& sm, 
            // std::vector<MyGeo::Vec3c>& framebuffer,
            std::vector<float>& zbuffer, 
            Texture& texture,
            Texture& hmap,
            FragmentData& fd
            )> 
            fragmentShader,
        ShaderMemories sm
        // int numThreads=8
        )
    {
        //vertex shader
        for(int i=0;i!=3*numTriangles;i++)
        {
            vertexShader(modelMat,camera,sm,i);
        }
        
        auto voutVertices=(MyRasterizer::Vertex*)sm.vOutMemory;
        auto vinVertices=(MyRasterizer::Vertex*)sm.vInMemory;

        // rasterization
        std::vector<std::thread> threads;
        int numThreads=6;
        int numTrianglesPerBlock=numTriangles/numThreads;
        // int numTrianglesPerBlock_Last=numTriangles%numTrianglesPerBlock;
        
        for(int i=0;i!=numThreads;i++)
        {
            int startInd=i*numTrianglesPerBlock;
            int endInd=std::min(startInd+numTrianglesPerBlock,(int)numTriangles); 
            // threads.push_back(std::thread(threadTest));
            // std::cout<<"start end "<<startInd<<" "<<endInd<<std::endl;
            threads.push_back(std::thread(
                Canvas::drawThread,
                startInd,
                endInd,
                std::ref(framebuffer),
                std::ref(zbuffer),
                std::ref(texture),
                std::ref(hmap),
                // sdlContext,
                fragmentShader,
                // std::ref(fragmentShader),
                std::ref(sm),
                width,height
                ));
        }
        
        for(int i=0;i!=numThreads;i++) threads[i].join();
        
        
        // std::cout<<"startInd "<<0<<" endInd "<<numTriangles<<std::endl;
        // std::cout<<"numTriangle "<<numTriangles<<std::endl;
//         for(int i=0;i!=numTriangles;i++)
//         {
//             float xmax=std::min(std::max({voutVertices[3*i].pos.x,voutVertices[3*i+1].pos.x,voutVertices[3*i+2].pos.x})+1,width);
//             float xmin=std::max(std::min({voutVertices[3*i].pos.x,voutVertices[3*i+1].pos.x,voutVertices[3*i+2].pos.x}),0.0f);
//             float ymax=std::min(std::max({voutVertices[3*i].pos.y,voutVertices[3*i+1].pos.y,voutVertices[3*i+2].pos.y})+1,height);
//             float ymin=std::max(std::min({voutVertices[3*i].pos.y,voutVertices[3*i+1].pos.y,voutVertices[3*i+2].pos.y}),0.0f);
//             MyGeo::Vec3f denom=getBariDenominator(voutVertices[3*i].pos,voutVertices[3*i+1].pos,voutVertices[3*i+2].pos);
//             //(x,y)为具体像素坐标了
//             for(float x=(int)(xmin+0.5)+0.5;x<xmax;++x)
//             for(float y=(int)(ymin+0.5)+0.5;y<ymax;++y)
//             {
//                 auto bc=getBariCooridinate({x,y,0,0},voutVertices[3*i].pos,voutVertices[3*i+1].pos,voutVertices[3*i+2].pos,denom);
//                 //todo  插值修正！
//                 if(bc.x>0 && bc.y>0 && bc.z>0)
//                 // if(
//                 //     (bc.x>0 || getSign(voutVertices[3*i+1].pos.x-voutVertices[3*i+2].pos.x)*denom[0]*lineJudgeValue({-1.f,-1.f,0.f},voutVertices[3*i+1].pos,voutVertices[3*i+2].pos)>0)
//                 //     &&
//                 //     (bc.y>0 || getSign(voutVertices[3*i+2].pos.x-voutVertices[3*i].pos.x)*denom[1]*lineJudgeValue({-1.f,-1.f,0.f},voutVertices[3*i+2].pos,voutVertices[3*i].pos)>0)
//                 //     &&
//                 //     (bc.z>0 || getSign(voutVertices[3*i].pos.x-voutVertices[3*i+1].pos.x)*denom[2]*lineJudgeValue({-1.f,-1.f,0.f},voutVertices[3*i].pos,voutVertices[3*i+1].pos)>0)
//                 // )
//                 {
//                     FragmentData fd;
//                     fd.position=vinVertices[3*i].pos*bc[0]+vinVertices[3*i+1].pos*bc[1]+vinVertices[3*i+2].pos*bc[2];
//                     fd.position.w=voutVertices[3*i].pos.w*bc[0]+voutVertices[3*i+1].pos.w*bc[1]+voutVertices[3*i+2].pos.w*bc[2];
//                     fd.normal=voutVertices[3*i].normal*bc[0]+voutVertices[3*i+1].normal*bc[1]+voutVertices[3*i+2].normal*bc[2];

//                     // fd.texCoord=(voutVertices[3*i].textureCoord+voutVertices[3*i].textureCoord+voutVertices[3*i].textureCoord)*0.33333f;
//                     // fd.texCoord={0.5,0.5};
//                     // fd.texCoord=voutVertices[3*i].textureCoord*bc[0]*(1/voutVertices[3*i].pos.w)+voutVertices[3*i+1].textureCoord*bc[1]*(1/voutVertices[3*i+1].pos.w)+voutVertices[3*i+2].textureCoord*bc[2]*(1/voutVertices[3*i+2].pos.w);
//                     // fd.texCoord*=w;
//                     // std::cout<<voutVertices
//                     fd.texCoord=voutVertices[3*i].textureCoord*bc[0]+voutVertices[3*i+1].textureCoord*bc[1]+voutVertices[3*i+2].textureCoord*bc[2];
//                     fd.screenCoord={(int)x,(int)y};
//                     // fd.color=voutVertices[3*i].color*bc[0]+voutVertices[3*i+1].color*bc[1]+voutVertices[3*i+2].color*bc[2];
//                     // std::cout<<fd.texCoord<<std::endl;

// // std::cout<<(int)voutVertices[3*i].color.x<<(int)voutVertices[3*i].color.y<<(int)voutVertices[3*i].color.z<<std::endl;
//                     auto fragColor=fragmentShader(sm,zbuffer,texture.value(),fd);
//                     // std::cout<<"scr1 "<<x<<','<<y<<std::endl;
//                     if(fragColor.x!=0 || fragColor.y!=0 || fragColor.z!=0)
//                     {
                        
//                         // setPixel({(int)x,(int)y},fragColor);
//                         int xi=(int)x;
//                         int yi=(int)y;

//                         if (xi < 0 || xi >= width ||
//                             yi < 0 || yi >= height) return;

//                         int ind=yi*width+xi;
//                         // std::cout<<"ind2 "<<ind<<' ';
//                         framebuffer[ind]=fragColor;

//                     }
//                 }
//             }
//         }

// std::cout<<"count "<<count<<std::endl;
        drawSDL();
    
    }

    static void drawThread(
        int startInd,int endInd,
        std::vector<MyGeo::Vec3c>& framebuffer,
        std::vector<float>& zbuffer,
        std::optional<Texture>& texture,
        std::optional<Texture>& hmap,
        // WindowContext* sdlContext,
        std::function<MyGeo::Vec3c(
        ShaderMemories& sm, 
        std::vector<float>& zbuffer, 
        Texture& texture,
        Texture& hmap,
        FragmentData& fd
        )> fragmentShader,
        ShaderMemories& sm,
        float width,float height
    )
    {
        auto voutVertices=(MyRasterizer::Vertex*)sm.vOutMemory;
        auto vinVertices=(MyRasterizer::Vertex*)sm.vInMemory;

        // std::cout<<"startInd "<<startInd<<" endInd "<<endInd<<std::endl;
        for(int i=startInd;i!=endInd;i++)
        {
            // assert(3*i+2<17568);
            // std::cout<<"i "<<i<<std::endl;
            float xmax=std::min(std::max({voutVertices[3*i].pos.x,voutVertices[3*i+1].pos.x,voutVertices[3*i+2].pos.x})+1,width);
            float xmin=std::max(std::min({voutVertices[3*i].pos.x,voutVertices[3*i+1].pos.x,voutVertices[3*i+2].pos.x}),0.0f);
            float ymax=std::min(std::max({voutVertices[3*i].pos.y,voutVertices[3*i+1].pos.y,voutVertices[3*i+2].pos.y})+1,height);
            float ymin=std::max(std::min({voutVertices[3*i].pos.y,voutVertices[3*i+1].pos.y,voutVertices[3*i+2].pos.y}),0.0f);
            MyGeo::Vec3f denom=getBariDenominator(voutVertices[3*i].pos,voutVertices[3*i+1].pos,voutVertices[3*i+2].pos);
            //(x,y)为具体像素坐标了
            for(float x=(int)(xmin+0.5)+0.5;x<xmax;++x)
            for(float y=(int)(ymin+0.5)+0.5;y<ymax;++y)
            {
                auto bc=getBariCooridinate({x,y,0,0},voutVertices[3*i].pos,voutVertices[3*i+1].pos,voutVertices[3*i+2].pos,denom);
                
                //todo  插值修正！
                if(bc.x>0 && bc.y>0 && bc.z>0)
                // if(
                //     (bc.x>0 || getSign(voutVertices[3*i+1].pos.x-voutVertices[3*i+2].pos.x)*denom[0]*lineJudgeValue({-1.f,-1.f,0.f},voutVertices[3*i+1].pos,voutVertices[3*i+2].pos)>0)
                //     &&
                //     (bc.y>0 || getSign(voutVertices[3*i+2].pos.x-voutVertices[3*i].pos.x)*denom[1]*lineJudgeValue({-1.f,-1.f,0.f},voutVertices[3*i+2].pos,voutVertices[3*i].pos)>0)
                //     &&
                //     (bc.z>0 || getSign(voutVertices[3*i].pos.x-voutVertices[3*i+1].pos.x)*denom[2]*lineJudgeValue({-1.f,-1.f,0.f},voutVertices[3*i].pos,voutVertices[3*i+1].pos)>0)
                // )
                {
                    FragmentData fd;
                    fd.position=vinVertices[3*i].pos*bc[0]+vinVertices[3*i+1].pos*bc[1]+vinVertices[3*i+2].pos*bc[2];
                    fd.position.w=voutVertices[3*i].pos.w*bc[0]+voutVertices[3*i+1].pos.w*bc[1]+voutVertices[3*i+2].pos.w*bc[2];
                    fd.normal=voutVertices[3*i].normal*bc[0]+voutVertices[3*i+1].normal*bc[1]+voutVertices[3*i+2].normal*bc[2];
                    // std::cout<<fd.normal<<std::endl;exit(0);

                    fd.texCoord=voutVertices[3*i].textureCoord*bc[0]+voutVertices[3*i+1].textureCoord*bc[1]+voutVertices[3*i+2].textureCoord*bc[2];
                    fd.screenCoord={(int)x,(int)y};

                    auto fragColor=fragmentShader(sm,zbuffer,texture.value(),hmap.value(),fd);
                    if(fragColor.x!=0 || fragColor.y!=0 || fragColor.z!=0)
                    {
                        int xi=(int)x;
                        int yi=(int)y;
                        if (xi < 0 || xi >= width ||
                            yi < 0 || yi >= height) return;
                        int ind=yi*width+xi;
                        framebuffer[ind]=fragColor;
                    }
                }
            }
        }
    
  
    }


    void drawIndexedTrianglesFilled(
        uint32_t* indices, 
        // VertexDataType vdt,
        uint32_t numTriangles,
        const MyGeo::Mat4f& modelMat,
        const MyGeo::Camera& camera,
        // size_t datasize,
        // size_t dataunitsize,
        std::function<void(const MyGeo::Mat4f& modelMat, const MyGeo::Camera& camera,void* vsInMemory,void* textureMemory,void* vsOutMemory)> vertexShader,
        std::function<void(void* fsInMemory,std::vector<MyGeo::Vec3f>& framebuffer)> fragmentShader,
        void* vertexMemory=nullptr, 
        void* textureMemory=nullptr
        )
    {
        //input assemble
        //vertex shader
        for(int i=0;i!=numTriangles;i++)
        {
            vertexShader(modelMat,camera,vertexMemory,textureMemory,nullptr);
        }
        //fragment shader
        // for()
    }

    //f_ij(x,y) judge side of a point with respect to a line
    static inline float lineJudgeValue(const MyGeo::Vec4f& p,const MyGeo::Vec4f& src,const MyGeo::Vec4f& dst)
    {
        return (src.y-dst.y)*p.x+(dst.x-src.x)*p.y+src.x*dst.y-src.y*dst.x;
    }
    
    static inline MyGeo::Vec3f getBariDenominator(MyGeo::Vec4f& v0,MyGeo::Vec4f& v1,MyGeo::Vec4f& v2)
    {
        float a=lineJudgeValue(v0,v1,v2);
        float b=lineJudgeValue(v1,v2,v0);
        float c=lineJudgeValue(v2,v0,v1);
        return {a,b,c};
    }
    
    static inline MyGeo::Vec3f getBariCooridinate(const MyGeo::Vec4f& p,MyGeo::Vec4f& v0,MyGeo::Vec4f& v1,MyGeo::Vec4f& v2,const MyGeo::Vec3f& denom)
    {
        float a=lineJudgeValue(p,v1,v2)/denom[0];
        float b=lineJudgeValue(p,v2,v0)/denom[1];
        // float c=lineJudgeValue(p,v0,v1)/denom[2];
        float c=1-a-b;
        return {a,b,c};
    }
};



}
