#include <algorithm>
#include <cassert>
#include "BVH.hpp"


class Triangle;

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}


std::vector<Object *>::iterator sortByCentroid(std::vector<Object*> & objects)
{
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
        centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();
    switch (dim) {
    case 0: // sort using centroid x
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
        });
        break;
    case 1: // sort using centroid y
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
    return objects.begin() + objects.size() / 2;
}

std::vector<Object *>::iterator sortBySurfaceArea(std::vector<Object*> & objects, float totalArea) {
    // using SAH , we can make the tree more balanced
    // which makes boxes smaller so that the intersection test is faster
    //firstly still use the centroid to sort the objects
    sortByCentroid(objects);
    float bestSAH = std::numeric_limits<float>::infinity();
    int bestSplit = 0;
    for (int i=0; i<objects.size(); i++){
        Bounds3 leftB, rightB;
        for(int j=0; j<i; j++)
            leftB = Union(leftB, objects[j]->getBounds());
        for(int j=i; j<objects.size(); j++)
            rightB = Union(rightB, objects[j]->getBounds());
        float leftP = leftB.SurfaceArea()/totalArea;
        float rightP = rightB.SurfaceArea()/totalArea;
        float SAH = 1.0f + leftP * (i+1) + rightP * (objects.size()-i-1);
        if (SAH < bestSAH){ // find the best split
            bestSAH = SAH;
            bestSplit = i;
        }
    }
    return objects.begin() + bestSplit;
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
        node->object = objects[0]; // this is actually a triangle
        node->left = nullptr;
        node->right = nullptr; // leaf node
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        // auto middling = sortByCentroid(objects);
        auto middling = sortBySurfaceArea(objects, bounds.SurfaceArea());
        auto beginning = objects.begin();
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

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect; // if no object in the scene
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    inter.happened = false;
    if(!node) return inter;
    std::array<int, 3> dirIsNeg = { ray.direction.x < 0 ? 1 : 0, 
                                ray.direction.y < 0 ? 1 : 0, 
                                ray.direction.z < 0 ? 1 : 0 };
    if(!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return inter; // no intersection
    
    // if it is leaf node, check all objects in it
    if(node->left == nullptr && node->right == nullptr){
        // in fact only triangle is in the leaf node
        return (node->object)->getIntersection(ray);
    }
    
    // if it is not leaf node, check both left and right child node
    Intersection left = getIntersection(node->left, ray); // it will recursely get the object with the nearest intersection
    Intersection right = getIntersection(node->right, ray);
    // if both are hitted , return the nearest one
    if(left.happened && right.happened){
        if(left.distance < right.distance)
            return left;
        else
            return right;
    }
    // if only one is hitted, return the hitted one
    if(left.happened)
        return left;
    return right;
}