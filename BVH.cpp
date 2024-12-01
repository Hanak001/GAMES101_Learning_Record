#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;
    if(splitMethod == SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else if(splitMethod == SplitMethod::SAH)
        root = recursiveBuild_SAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild_SAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild_SAH(std::vector{objects[0]});
        node->right = recursiveBuild_SAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        
        float boundsarea = bounds.SurfaceArea();
        const int bucket_num = 10;
        Bucket bucket[bucket_num];
        for(int i = 0; i < objects.size(); i++){
            int b;
            switch(dim){
                case 0:
                    b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).x * bucket_num;
                    break;
                case 1:
                    b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).y * bucket_num;
                    break;
                case 2:
                    b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).z * bucket_num;
                    break;
            }
            if(b == bucket_num){
                b = bucket_num - 1;
            }
            bucket[b].primitive_num++;
            bucket[b].bound = Union(bucket[b].bound, objects[i]->getBounds());
        }
        float cost[bucket_num - 1];
        float mincost = std::numeric_limits<float>::max();
        int min_idx = -1;
        for(int i = 0; i < bucket_num -1; i++){
            Bounds3 bounds1, bounds2;
            int cnt1 = 0, cnt2 = 0;
            for(int j = 0; j < i + 1; j++){
                cnt1 += bucket[j].primitive_num;
                bounds1 = Union(bounds1, bucket[j].bound);
            }
            for(int j = i + 1; j < bucket_num; j++){
                cnt2 += bucket[j].primitive_num;
                bounds2 = Union(bounds2, bucket[j].bound);
            }
            cost[i] = 0.125f + ((bounds1.SurfaceArea() * cnt1 + bounds2.SurfaceArea() * cnt2) / bounds.SurfaceArea());
            if(cost[i] < mincost){
                mincost = cost[i];
                min_idx = i;
            }
        }

        auto leftshapes = std::vector<Object*>();
        auto rightshapes = std::vector<Object*>(); 
        for(int i = 0; i < objects.size(); i++){
            int b;
            switch(dim){
                case 0:
                    b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).x * bucket_num;
                    break;
                case 1:
                    b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).y * bucket_num;
                    break;
                case 2:
                    b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).z * bucket_num;
                    break;
            }
            if(b == bucket_num){
                b = bucket_num - 1;
            }
            if(b <= min_idx) leftshapes.push_back(objects[i]);
            else rightshapes.push_back(objects[i]);
        }


        node->left = recursiveBuild_SAH(leftshapes);
        node->right = recursiveBuild_SAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if(!node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0})) return Intersection();

    if(!node->left&&!node->right){
        return node->object->getIntersection(ray);
    }

    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    return hit1.distance < hit2.distance? hit1:hit2;


}

