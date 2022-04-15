#include"src/canvas.h"
#include"OBJ_Loader.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include"rasterizerApp.h"
constexpr int Width=400;
constexpr int Height=400;

void test()
{
    // MyGeo::Vec4f v0{1,0,0,0},v1{-1,0,0,0},v2{0,1,0,0};
    // auto denom=MyRasterizer::Canvas::getBariDenominator(v0,v1,v2);
    // auto bc=MyRasterizer::Canvas::getBariCooridinate({0,0,0,0},v0,v1,v2,denom);
    // std::cout<<bc<<std::endl;return;
    MyRasterizer::MyRasterizerApp app;
    app.run();
}
#define APP
#ifdef APP
int main()
{
    test();return 0;
}
#elif
int main()
{

    float angle=140.0;
    int key=0;

    MyGeo::Camera camera({0,0,10},{0,0,0},{0,-1,0});
    camera.setNearFar(-0.1,-50);
    camera.setFov(45,1.0);
    camera.updateMat();
    float x=0,y=0,z=0;
    MyGeo::Mat4f modelMat=MyGeo::translateMatrix({x,y,z})*MyGeo::rotationMatrix({0,1,0},angle)*MyGeo::scaleMatrix({2,2,2});

    objl::Loader loader;
    std::string obj_path="../models/Crate/";
    //spot_texture.png
    
    MyRasterizer::Texture texture(obj_path+"crate_1.jpg");

    bool loadout=loader.LoadFile(obj_path+"Crate1.obj");
    std::vector<MyRasterizer::Vertex> vertices;
    std::vector<MyRasterizer::Vertex> vertices_out;
    std::vector<MyRasterizer::Vertex> fragment_in;


    fragment_in.resize(Width*Height);
    int triangleNum=0;
    for(auto mesh:loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            ++triangleNum;
            for(int j=0;j<3;j++)
            {
                auto& v=mesh.Vertices[i+j];
                vertices.push_back(MyRasterizer::Vertex{
                    {v.Position.X,v.Position.Y,v.Position.Z,1.0f},{v.TextureCoordinate.X,v.TextureCoordinate.Y},{v.Normal.X,v.Normal.Y,v.Normal.Z,0.0f}
                    });
            }
        }
    }
    vertices_out.resize(vertices.size());
    std::cout<<"num of triangles "<<triangleNum<<std::endl;

    MyRasterizer::Canvas canvas(Width,Height);
    canvas.setTexture(texture);
    MyRasterizer::ShaderMemories sm;
    sm.vInMemory=(void*)vertices.data();
    sm.vOutMemory=(void*)vertices_out.data();
    sm.fInMemory=(void*)fragment_in.data(); 


//     canvas.drawTrianglesFilled(triangleNum,modelMat,camera,vertexShader,fragmentShader,sm);
//     cv::Mat image(Height,Width,CV_8UC3,canvas.framebuffer.data());
// // CV_32FC3
//     // image.convertTo(image,CV_8UC3,1.0f);
//     cv::imwrite("a.png",image);

    while(key!=27)
    {
        canvas.clear();
        MyGeo::Mat4f modelMat=MyGeo::translateMatrix({x,y,z})*MyGeo::rotationMatrix({0,1,0},angle)*MyGeo::scaleMatrix({2,2,2});
        canvas.drawTrianglesFilled(triangleNum,modelMat,camera,vertexShader,fragmentShader,sm);
        cv::Mat image(Height,Width,CV_8UC3,canvas.framebuffer.data());
        cv::imshow("myimage.png",image);
        key=cv::waitKey(10);

        if(key=='q') angle+=10;
        else if(key=='e') angle-=10;
        else if(key=='a') x-=0.2;
        else if(key=='d') x+=0.2;
        else if(key=='w') y+=0.2;
        else if(key=='s') y-=0.2;
        else if(key=='z') z+=0.2;
        else if(key=='x') z-=0.2;
    }
}
#endif





