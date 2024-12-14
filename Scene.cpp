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

    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir;
    Vector3f L_indir;

    Intersection inter = intersect(ray);

    if(inter.happened){
        Vector3f hit_point = inter.coords;
        Vector3f normal = inter.normal;
        Material* m = inter.m;

        Intersection inter_light;
        float pdf_light;
        sampleLight(inter_light, pdf_light);

        Vector3f light_point_x = inter_light.coords;
        Vector3f ws = normalize(light_point_x - hit_point);
        Vector3f NN = inter_light.normal;
        Vector3f emit = inter_light.emit;

        Vector3f reflectionRayOrig = (dotProduct(ws, normal) < 0) ?
                                             hit_point - normal * EPSILON :
                                             hit_point + normal * EPSILON;
        Ray test(reflectionRayOrig, ws);
        Intersection inter_test = intersect(test);
        
        if(inter_test.happened && inter_test.m->hasEmission()){
            L_dir = emit * m->eval(ray.direction, ws, normal) * dotProduct(ws, normal) * dotProduct(-ws, NN) / ((light_point_x - hit_point).norm() * (light_point_x - hit_point).norm() * pdf_light);
        }

        if(get_random_float() < RussianRoulette){
            Vector3f outDir = m->sample(ray.direction, normal);
            Ray out_ray(reflectionRayOrig, outDir);
            Intersection out_inter = intersect(out_ray);
            if(out_inter.happened && !out_inter.m->hasEmission()){
                L_indir = castRay(out_ray, depth + 1) * m->eval(ray.direction, outDir, normal) * dotProduct(outDir, normal) /(m->pdf(-ray.direction, outDir, normal) * RussianRoulette);
            }
        }

        Vector3f hitColor =  m->getEmission() + L_dir + L_indir;
        hitColor.x = (clamp(0, 1, hitColor.x));
        hitColor.y = (clamp(0, 1, hitColor.y));
        hitColor.z = (clamp(0, 1, hitColor.z));
        return hitColor;
    }
    else{
        return L_dir;
    }



    
    



    

}