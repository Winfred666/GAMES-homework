//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, const Intersection &point, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    // depth: the depth of recursion
    // return the color of ray
    // if (depth > this->maxDepth) { // do not use maxDepth here, use RussianRoulette
    //     return Vector3f(0);
    // }
    // here do not consider directly hit the emit light, consider outside
    if (!point.happened) {
        return Vector3f(0);
    }
    
    // 1. sample a point from light source
    float pdf_light = 0; // this is the pdf of the light source
    Intersection light_sample;
    sampleLight(light_sample, pdf_light);
    Vector3f L_dir(0);

    //now from light_sample to point, check if the light_sample is blocked
    Vector3f wi = light_sample.coords - point.coords; // the direction from the intersection point to the light source
    Ray light_ray(light_sample.coords, -normalize(wi));
    Intersection light_intersect = intersect(light_ray);

    // if pdf_light is very small(a small solid angle, look like a point), then the light source is very far away, we can ignore it
    if(light_intersect.happened && light_intersect.obj == point.obj
         && pdf_light > EPSILON){ //the light sample is not blocked
        float light_dis = wi.norm();
        wi = normalize(wi);
        
        // calculate L_i * f_r * cos_theta cos_theta' / |x'-p|^2 / pdf_light
        Vector3f f_r = point.m->eval(wi, ray.direction, point.normal); // wo is negative of the outgoing direction
        L_dir = light_sample.emit * f_r * dotProduct(point.normal, wi) * 
        dotProduct(light_sample.normal, -wi) / (light_dis*light_dis) / pdf_light;
    }
    
    // 2. sample a direction from other reflectors
    Vector3f L_indir(0);
    // Test Russian Roulette
    float p = get_random_float();

    if(p < RussianRoulette){
        Vector3f wi = point.m->sample(ray.direction, point.normal); // sample a direction
        wi = wi.normalized();
        float pdf = point.m->pdf(ray.direction, wi, point.normal);
        // if pdf is very small, then the direction is very far away, we can ignore it
        if(pdf < EPSILON){
            return L_dir;
        }

        Ray reflect_ray(point.coords, wi); // create a new ray from point to the sampled direction
        Intersection reflect_intersect = intersect(reflect_ray); // get the intersection point of the new ray
        
        if(reflect_intersect.happened && !reflect_intersect.m->hasEmission()){ // hit a non-emissive object
            
            L_indir = castRay(reflect_ray, reflect_intersect, depth+1) *
             point.m->eval(wi, ray.direction, point.normal) * dotProduct(point.normal,wi)
             / pdf / RussianRoulette;
             // because of texture here, the pdf is not uniform like 1/2PI
        }
    }
    return L_dir + L_indir;
}