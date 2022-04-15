#pragma once
#include<SDL2/SDL.h>
#include<iostream>
#include<array>
#include<mygeo/vec.h>
// static const int WIDTH=100;
// static const int HEIGHT=100;


namespace MyRasterizer{
    
struct WindowContext
{
    SDL_Window* window;
    SDL_Renderer* renderer;
    int width;
    int height;
    // SDL_Surface* surface;
    // SDL_Texture* framebuffer;    
    WindowContext(int width,int height,const std::string& name):width{width},height{height}
    {
        if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO)<0)
        {
            std::cout<<"sdl init failed "<<SDL_GetError()<<std::endl;
            return ;
        }
        window=SDL_CreateWindow(name.c_str(),SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN );
        if(window==nullptr)
        {
            throw std::runtime_error("window create failed "+std::string(SDL_GetError()));
        }

        renderer=SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED);
    }

    void test()
    {
        rendererClear();
        for(int i=0;i!=width;i++)
        for(int j=0;j!=height;j++)
            drawPixel(i,j,{0,255,255,255});
        renderPresent();
        sleep(2000);
    }
    void rendererClear()
    {
        SDL_SetRenderDrawColor(renderer,0,0,0,255);
        SDL_RenderClear(renderer);
    }

    void drawPixel(int x,int y, const MyGeo::Vec3c& color={255,255,255,255})
    {
        // std::cout<<"draw "<<x<<','<<y<<std::endl;
        SDL_SetRenderDrawColor(renderer,color.x,color.y,color.z,255);
        SDL_RenderDrawPoint(renderer,x,y);
    }
    
    void renderPresent()
    {
        SDL_RenderPresent(renderer);
    }
    void sleep(uint32_t miliseconds)
    {
        SDL_Delay(miliseconds);
    }


    ~WindowContext()
    {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
};


}



// void renderTest()
// {
//     SDL_Window* window;
//     SDL_Renderer* renderer;
//     SDL_Surface* surface;
//     SDL_Texture* framebuffer;
// //   SDL_Texture* BlueShapes;
//     // SDL_Rect srcRect;
//     // SDL_Rect dstRect;
//     // srcRect.x=0;
//     // srcRect.y=0;
//     // srcRect.w=
//     if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO)<0)
//     {
//         std::cout<<"sdl init failed "<<SDL_GetError()<<std::endl;
//         return ;
//     }
//     window=SDL_CreateWindow("hello",SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN );
//     if(window==nullptr)
//     {
//         throw std::runtime_error("window create failed "+std::string(SDL_GetError()));
//     }

//     renderer=SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED);

//     framebuffer=SDL_CreateTexture(renderer,SDL_PIXELFORMAT_RGB888,SDL_TEXTUREACCESS_STREAMING,WIDTH,HEIGHT);
//     SDL_SetRenderDrawColor(renderer,255,0,255,1);

//     std::array<MyGeo::Vec3c,WIDTH*HEIGHT> framememory;
//     // std::cout<<"color size "<<sizeof(uint32_t)<<std::endl;
//     for(auto& pixel:framememory)
//     {
//         pixel={0xff,0xff,0xff};
//     }
//     // renderer.
//     for(int i=0;i!=WIDTH;i++)
//     for(int j=0;j!=HEIGHT;j++)
//     SDL_RenderDrawPoint(renderer,i,j);

//     // SDL_UpdateTexture(framebuffer,nullptr,framememory.data(),WIDTH*sizeof(MyGeo::Vec3c));

//     SDL_RenderClear(renderer);
//     // SDL_RenderCopy(renderer,framebuffer,nullptr,nullptr);
//     SDL_RenderPresent(renderer);
//     SDL_Delay(2000);
//     SDL_DestroyWindow(window);
//     SDL_Quit();
//     // surface=SDL_GetWindowSurface(window);
//     // SDL_FreeSurface(surface);
    


// }


// int main()
// {
//     // renderTest();
//     WindowContext wc(800,600,"hello");
    
//     wc.test();
//     return 0;
//     SDL_Window* window=nullptr;
//     SDL_Surface* surface=nullptr;
//     if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO)<0)
//     {
//         std::cout<<"sdl init failed "<<SDL_GetError()<<std::endl;
//         return 1;
//     }
//     window=SDL_CreateWindow("hello",SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN );
//     if(window==nullptr)
//     {
//         throw std::runtime_error("window create failed "+std::string(SDL_GetError()));
//     }
//     surface=SDL_GetWindowSurface(window);
//     SDL_FillRect(surface,nullptr,SDL_MapRGB(surface->format,0xFF,0xFF,0xFF));
//     SDL_UpdateWindowSurface(window);
//     SDL_Delay(2000);

//     SDL_DestroyWindow(window);
//     SDL_Quit();
// }



