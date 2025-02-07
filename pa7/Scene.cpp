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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f L_dir, L_indir;
    Intersection inter = intersect(ray);

    // Don't intersect.
    if (!inter.happened)
        return Vector3f(0.0f);

    // Ray intersects light directly from pixel.
    if (inter.m->hasEmission()) 
        return inter.m->getEmission();

    // Sample light. 
    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light);
    inter_light.distance = (inter_light.coords - inter.coords).norm();

    Vector3f w_inter2light = normalize(inter_light.coords - inter.coords);
    Intersection inter_block = intersect(Ray(inter.coords, w_inter2light));
    if (fabs(inter_block.distance - inter_light.distance) < 0.001) {
        L_dir = inter.m->eval(ray.direction, w_inter2light, inter.normal)
                * inter_light.emit 
                * dotProduct(inter.normal, w_inter2light)
                * dotProduct(inter_light.normal, -w_inter2light)
                / pow(inter_light.distance, 2)
                / pdf_light;
    }


    // Sample non-emitting objects
    if (get_random_float() < RussianRoulette) {
        Vector3f w_inter2obj = inter.m->sample(ray.direction, inter.normal);
        Vector3f bd_ray_orig = dotProduct(inter.normal, w_inter2obj) < 0
                            ? inter.coords - inter.normal * EPSILON 
                            : inter.coords + inter.normal * EPSILON;
        Ray bd_ray = Ray(bd_ray_orig, w_inter2obj);

        Intersection inter_bd_ray = intersect(bd_ray);
        if (inter_bd_ray.happened && !inter_bd_ray.m->hasEmission()) {
            float pdf_obj = inter.m->pdf(ray.direction, w_inter2obj, inter.normal);
            L_indir = inter.m->eval(ray.direction, w_inter2obj, inter.normal)
                    * castRay(Ray(bd_ray_orig, w_inter2obj), depth + 1)
                    * dotProduct(inter.normal, w_inter2obj)
                    / pdf_obj
                    / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}
