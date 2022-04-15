#include"src/canvas.h"
// #include"OBJ_Loader.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<map>
#include"src/testdata.h"
#include<math.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include<tiny_obj_loader.h>

static void loadModel(std::string& modelPath,std::vector<MyRasterizer::Vertex>& vertices)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn,err;    
    if(!tinyobj::LoadObj(&attrib,&shapes,&materials,&warn,&err,modelPath.c_str()))
    {
        throw std::runtime_error(warn+err);
    }
    std::cout<<"shapes "<<shapes.size()<<std::endl;
    for(const auto& shape:shapes)
    {
        for(const auto& index:shape.mesh.indices)
        {
            MyRasterizer::Vertex v;
            v.pos={
                attrib.vertices[3*index.vertex_index],
                attrib.vertices[3*index.vertex_index+1],
                attrib.vertices[3*index.vertex_index+2],
                1
            };
            v.normal={
                attrib.normals[3*index.normal_index],
                attrib.normals[3*index.normal_index+1],
                attrib.normals[3*index.normal_index+2],
                0
            };
            v.textureCoord={
                attrib.texcoords[2*index.texcoord_index],
                attrib.texcoords[2*index.texcoord_index+1]
            };
            vertices.push_back(v);
        }
    }
}



static std::map<std::string,std::string> modelpath={
    {"cube","../models/cube/cube.obj"},
    {"spot","../models/spot/spot_triangulated_good.obj"},
    {"crate","../models/Crate/Crate1.obj"},
    {"rock","../models/rock/rock.obj"},
    {"skull","../models/skull/skull.obj"},
    };
static std::map<std::string,std::string> texpath={
    {"cube","../models/cube/wall.tif"},
    {"spot","../models/spot/spot_texture.png"},
    {"crate","../models/Crate/crate_1.jpg"},
    {"rock","../models/rock/rock.png"},
    {"skull","../models/skull/Skull.jpg"},
    };

namespace MyRasterizer{
// "../models/spot/spot_triangulated_good.obj"
// "../models/spot/spot_texture.png"
static MyGeo::Camera defaultCamera({0,0,5},{0,0,0},{0,-1,0});//{0,0,-50} -4
static MyGeo::Mat4f defaultModelMat;
static MyRasterizer::Texture defaultTexture;
static MyRasterizer::Texture heightTexture;
static std::string modelDir="spot";
static std::string defaultObjPath=modelpath[modelDir];
static std::string defaultTexturePath=texpath[modelDir];
static std::string heighTexturePath="../models/spot/hmap.jpg";
static std::vector<MyRasterizer::Vertex> vertices;
static std::vector<MyRasterizer::Vertex> vertices_out;
static std::vector<MyRasterizer::Vertex> fragment_in;
static MyRasterizer::ShaderMemories sm;

static float Width=1024;
static float Height=1024;
static MyGeo::Vec3f lightDir={0,0,1};


struct PointLight
{
    MyGeo::Vec3f position;
    MyGeo::Vec3f intensity;
};





static void vertexShader(
    MyGeo::Mat4f& modelMat, 
    MyGeo::Camera& camera,
    MyRasterizer::ShaderMemories& sm,
    size_t ind)
{
    MyGeo::Mat4f transformMat=camera.projMat*camera.viewMat*modelMat;
    auto vertexData=(MyRasterizer::Vertex*)sm.vInMemory;
    auto vertexOutData=(MyRasterizer::Vertex*)sm.vOutMemory;
    vertexOutData[ind].pos=transformMat*vertexData[ind].pos;
    auto& pos=vertexOutData[ind].pos;
// std::cout<<vertexData[ind].pos<<std::endl;
    float w_reci=1.0f/pos.w;

    pos.x=(pos.x*w_reci+1)*Width*0.5f;//(-1,1)->(0,Width)
    pos.y=(1-pos.y*w_reci)*Height*0.5f;
    // pos.w=1.0f;
    // pos.w=pos.z;//存储depth等效
    // std::cout<<"depth "<<pos.w<<' ';
    pos.z*=w_reci;
    // std::cout<<pos<<std::endl;
    vertexOutData[ind].textureCoord=vertexData[ind].textureCoord;
    // std::cout<<vertexOutData[ind].textureCoord<<std::endl;
    //假设只有translation和rotation，注意normal w 分量为0
    vertexOutData[ind].normal=transformMat*vertexData[ind].normal;
    // vertexOutData[ind].color=vertexData[ind].color;
}

static MyGeo::Vec3f colorC2F(MyGeo::Vec3c srcColor)
{
    MyGeo::Vec3f dstColor={(float)srcColor.x,(float)srcColor.y,(float)srcColor.z};
    // return dstColor.normalize();
    float k=1.0f/255;
    for(int i=0;i!=3;i++)
    {
        dstColor[i]=(float)srcColor[i]*k;
    }
    return dstColor;
}

static MyGeo::Vec3c colorF2C(MyGeo::Vec3f srcColor)
{
    MyGeo::Vec3c dstColor;
    for(int i=0;i!=3;i++)
    {
        dstColor[i]=srcColor[i]*255;
    }
    return dstColor;
}

// static const float ka=0.1f;
// static const float kd=0.5f;
// static const float ks=0.4f;

static MyGeo::Vec3c BlinnPhongFragmentShader(
    MyRasterizer::ShaderMemories& sm,
    // std::vector<MyGeo::Vec3c>& framebuffer,
    std::vector<float>& zbuffer,
    MyRasterizer::Texture& texture, 
    // MyRasterizer::Texture& hmap,
    MyRasterizer::FragmentData& fd)
{

    static PointLight light1{{20,20,20},{500,500,500}};
    static PointLight light2{{-20,20,0},{500,500,500}};
    static PointLight light3{{0,0,40},{500,500,500}};
    static std::array<PointLight,3> lights={light1,light3};
    MyGeo::Vec3f amb_light_intensity{10, 10, 10};
    static MyGeo::Vec3f ka = {0.005, 0.005, 0.005};
    static MyGeo::Vec3f ks = {0.7937, 0.7937, 0.7937};
    int ind=fd.screenCoord.x+fd.screenCoord.y*Width;

    auto ca=ka.cwiseprod(amb_light_intensity);
    if(fd.position.w>zbuffer[ind])
    {
        zbuffer[ind]=fd.position.w;
        auto color=texture.getColor(fd.texCoord.x,fd.texCoord.y);    
        // auto height=hmap.getColor(fd.texCoord.x,fd.texCoord.y);
        MyGeo::Vec3f kd = colorC2F(color); // diffuse color
        auto normal=fd.normal.head.normalize();
        MyGeo::Vec3f cd{0,0,0},cs{0,0,0};

        for(auto& l:lights)
        {
            auto d2r=1.0f/(l.position-fd.position.head).norm2();// 1/(d*d)
            cd+=kd.cwiseprod(l.intensity)*d2r*std::max(0.f,normal.dot((l.position-fd.position.head).normalize()));

            //reflect vector
            auto r=MyGeo::reflect((l.position-fd.position.head).normalize(),normal).normalize();
            //spectral color
            cs+=ks.cwiseprod(l.intensity)*d2r*std::pow(std::max(0.f,normal.dot(r)),150.0);
        }
        return colorF2C(ca+cd+cs);
    }
    return MyGeo::Vec3c{0,0,0};

}


static MyGeo::Vec3c BlinnPhongFragmentShader_Bump(
    MyRasterizer::ShaderMemories& sm,
    std::vector<float>& zbuffer,
    MyRasterizer::Texture& texture, 
    MyRasterizer::Texture& hmap,
    MyRasterizer::FragmentData& fd)
{
    static PointLight light1{{20,20,20},{500,500,500}};
    static PointLight light2{{-20,20,0},{500,500,500}};
    static PointLight light3{{0,0,40},{500,500,500}};
    static std::array<PointLight,3> lights={light1,light3};
    MyGeo::Vec3f amb_light_intensity{10, 10, 10};
    static MyGeo::Vec3f ka = {0.005, 0.005, 0.005};
    static MyGeo::Vec3f ks = {0.7937, 0.7937, 0.7937};
    int ind=fd.screenCoord.x+fd.screenCoord.y*Width;
    auto ca=ka.cwiseprod(amb_light_intensity);
    if(fd.position.w>zbuffer[ind])
    {
        zbuffer[ind]=fd.position.w;
        auto color=texture.getColor(fd.texCoord.x,fd.texCoord.y);   
        MyGeo::Vec3f kd = colorC2F(color); // diffuse color
        auto normal=fd.normal.head.normalize();
        MyGeo::Vec3f cd{0,0,0},cs{0,0,0};

        float kh=0.2f,kn=0.1f;
        float g=std::sqrt(normal.x*normal.x+normal.z*normal.z);
        float recig=1.0f/g;
        MyGeo::Vec3f t={normal.x*normal.y*recig,g,normal.z*normal.y*recig};
        MyGeo::Vec3f b=normal.cross(t);
        MyGeo::Mat3f tbn{t,b,normal};
        auto h1=colorC2F(hmap.getColor(fd.texCoord.x+1.0f/hmap.width,fd.texCoord.y)).norm();
        auto h2=colorC2F(hmap.getColor(fd.texCoord.x,fd.texCoord.y+1.0f/hmap.height)).norm();
        auto h0=colorC2F(hmap.getColor(fd.texCoord.x,fd.texCoord.y)).norm();
        auto dU=3.f*(h1-h0);
        auto dV=3.f*(h2-h0);
        MyGeo::Vec3f ln{-dU,-dV,1.0f};
        MyGeo::Vec3f surfacePosition=fd.position.head+normal*(kn*h0);
        MyGeo::Vec3f newNormal=(tbn*ln).normalize();
        for(auto& l:lights)
        {
            auto d2r=1.0f/(l.position-surfacePosition).norm2();// 1/(d*d)
            cd+=kd.cwiseprod(l.intensity)*d2r*std::max(0.f,newNormal.dot((l.position-surfacePosition).normalize()));
            //reflect vector
            auto r=MyGeo::reflect((l.position-surfacePosition).normalize(),newNormal).normalize();
            //spectral color
            cs+=ks.cwiseprod(l.intensity)*d2r*std::pow(std::max(0.f,newNormal.dot(r)),150.0);
        }
        return colorF2C(ca+cd+cs);
    }
    return MyGeo::Vec3c{0,0,0};

}


struct MyRasterizerApp
{
    objl::Loader loader;
    MyRasterizer::Canvas* canvas;
    // float Height=400,Width=400;
    std::string windowName;
    
    // MyRasterizer::ShaderMemories sm;

    void genTestVertices()
    {
        // int triangleNum=0;
        genBox(vertices);
        vertices_out.resize(vertices.size());
        
    }

    virtual void genVertices()
    {
        loadModel(defaultObjPath,vertices);
        // bool loadout=loader.LoadFile(defaultObjPath);
        defaultTexture.attachImage(defaultTexturePath);
        heightTexture.attachImage(heighTexturePath);

        // fragment_in.resize(Width*Height);
        // // int triangleNum=0;
        // int numMesh=0;
        // int nv=0;
        // for(auto mesh:loader.LoadedMeshes)
        // {
        //     ++numMesh;
        //     for(int i=0;i<mesh.Vertices.size();++i)
        //     {
        //         ++nv;
        //         auto& v=mesh.Vertices[i];
        //         vertices.push_back(MyRasterizer::Vertex{
        //             {v.Position.X,v.Position.Y,v.Position.Z,1.0f},{v.TextureCoordinate.X,v.TextureCoordinate.Y},{v.Normal.X,v.Normal.Y,v.Normal.Z,0.0f}
        //             });
        //     }
        // }
        vertices_out.resize(vertices.size());
        // std::cout<<"num of triangles "<<ver<<std::endl;
        std::cout<<"num of vertices "<<vertices.size()<<std::endl;
        // std::cout<<"num of meshes "<<numMesh<<std::endl;
    }

    virtual void init()
    {
        defaultCamera.setNearFar(-0.1,-100);
        defaultCamera.setFov(45,1.0);
        defaultCamera.updateMat();

        // genTestVertices();
        genVertices();
        
        canvas=new MyRasterizer::Canvas(Width,Height);
        canvas->setTexture(defaultTexture);
        canvas->setHeightTexture(heightTexture);
        
        sm.vInMemory=(void*)vertices.data();
        sm.vOutMemory=(void*)vertices_out.data();
        sm.fInMemory=(void*)fragment_in.data(); 

    }

    virtual void run()
    {
        // auto color=MyGeo::Vec3c{255,255,255};
        // std::cout<<"color "<<colorC2F(color)<<std::endl;
        // return;
        init();
        std::cout<<"start mainloop"<<std::endl;
        mainLoop();
    }

    virtual void mainLoop()
    {
        int key=0;
        float angleY=0.0f;//30
        float angleX=0.0f;//270
        float angleZ=0.0f;
        float x=0,y=0,z=0;//y=-10

        for(int i=0;i!=1000;i++)
        {
            angleY+=2;
            angleX+=2;
            angleZ+=2;
            x=std::sin(i*0.001);
            y=std::cos(i*0.001);
            canvas->clear();
            defaultModelMat=MyGeo::translateMatrix({x,y,z})*MyGeo::rotationMatrix({0,1,0},angleY)*MyGeo::rotationMatrix({1,0,0},angleX)*MyGeo::rotationMatrix({0,0,1},angleZ);//*MyGeo::scaleMatrix({2,2,2});
            canvas->sdlContext->rendererClear();
            canvas->drawTrianglesFilled(vertices.size()/3,defaultModelMat,defaultCamera,vertexShader,BlinnPhongFragmentShader_Bump,sm);
            // std::cout<<"finish drawing"<<std::endl;
            canvas->sdlContext->renderPresent();
            
            // canvas->sdlContext->sleep(300);
        }
        return;

        // canvas->clear();
        // defaultModelMat=MyGeo::translateMatrix({x,y,z})*MyGeo::rotationMatrix({0,1,0},angleY)*MyGeo::rotationMatrix({1,0,0},angleX)*MyGeo::rotationMatrix({0,0,1},angleZ);//*MyGeo::scaleMatrix({2,2,2});
        // canvas->sdlContext->rendererClear();
        // canvas->drawTrianglesFilled(vertices.size()/3,defaultModelMat,defaultCamera,vertexShader,BlinnPhongFragmentShader,sm);
        // std::cout<<"finish drawing"<<std::endl;
        // canvas->sdlContext->renderPresent();
        // canvas->sdlContext->sleep(5000);
        // return;


        while(key!=27)
        {
            canvas->clear();
            defaultModelMat=MyGeo::translateMatrix({x,y,z})*MyGeo::rotationMatrix({0,1,0},angleY)*MyGeo::rotationMatrix({1,0,0},angleX)*MyGeo::rotationMatrix({0,0,1},angleZ);//*MyGeo::scaleMatrix({2,2,2});
            canvas->drawTrianglesFilled(vertices.size()/3,defaultModelMat,defaultCamera,vertexShader,BlinnPhongFragmentShader_Bump,sm);
            cv::Mat image(Height,Width,CV_8UC3,canvas->framebuffer.data());
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            cv::imwrite("a.png",image);
            break;
            cv::imshow(windowName,image);
            key=cv::waitKey(10);
            if(key=='q') angleY+=10;
            else if(key=='e') angleY-=10;
            else if(key=='r') angleX+=10;
            else if(key=='t') angleX-=10;
            else if(key=='f') angleZ+=10;
            else if(key=='g') angleZ-=10;
            else if(key=='a') x-=0.5;
            else if(key=='d') x+=0.5;
            else if(key=='w') y+=0.5;
            else if(key=='s') y-=0.5;
            else if(key=='z') z+=0.5;
            else if(key=='x') z-=0.5;            
        }
    }
};

}

