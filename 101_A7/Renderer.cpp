//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <vector>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

Vector3f eye_pos(278, 273, -800);

float Global_Progress = 0;

Ray generateRay(int i, int j, const Scene& scene){
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    // generate primary ray direction
    float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                imageAspectRatio * scale;
    float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
    Vector3f dir = normalize(Vector3f(-x, y, 1));
    
    return Ray(eye_pos, dir);
}


void process_chunk(int start, int end, int spp, std::vector<Vector3f>& framebuffer,const Scene* scene) {
    for (int m = start; m < end; ++m) {
        int i = m % scene->width;
        int j = m / scene->width;
        Ray eye_ray = generateRay(i,j,*scene);  // Assuming generateRay is a function to create a ray
        Intersection eye_intersect = scene->intersect(eye_ray);
        framebuffer[m] = Vector3f(0);  // Initialize color accumulation
        for (int k = 0; k < spp; ++k) {
            framebuffer[m] += scene->castRay(eye_ray, eye_intersect, 0) / spp;
        }
        if(eye_intersect.happened && eye_intersect.m->hasEmission()){
            framebuffer[m] += eye_intersect.emit; // add the emission color
        }
        // UpdateProgress((++Global_Progress)/(scene->width*scene->height));
    }
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    int m = 0;

    // change the spp value to change sample ammount
    int spp = 8;
    std::cout << "SPP: " << spp << "\n";
    const int num_threads = std::thread::hardware_concurrency();
    std::cout << "Number of threads: " << num_threads << "\n";
    std::vector<std::thread> threads;

    int chunk_size = (scene.width * scene.height) / num_threads;
    
    for (int t = 0; t < num_threads; ++t) {
        int start = t * chunk_size;
        int end = (t == num_threads - 1) ? (scene.width * scene.height) : (start + chunk_size);
        threads.emplace_back(process_chunk, start, end, spp, std::ref(framebuffer), &scene);
    }

    for (auto& thread : threads) {
        thread.join();
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
