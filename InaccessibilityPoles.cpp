//
// Created by Jonas Tollenaere on 11/04/2025.
//

#include "InaccessibilityPoles.h"
#include "meshcore/acceleration/CachingBoundsTreeFactory.h"
#include "meshcore/acceleration/AABBOctree.h"
#include "meshcore/acceleration/AABBVolumeHierarchy.h"
#include <queue>
#include <glm/gtx/component_wise.hpp>

float computeSignedDistanceToMeshAndPolesFlatBVH(const std::shared_ptr<BoundingVolumeHierarchy>& tree, const glm::vec3& point, const std::vector<Sphere>& poles, double minimumDistanceRequired){

    auto minimumSignedDistance = std::numeric_limits<float>::max();
    for (const auto &pole: poles){
        auto signedPoleDistance = glm::length(point - pole.getCenter()) - pole.getRadius();

        if(signedPoleDistance < minimumSignedDistance){
            minimumSignedDistance = signedPoleDistance;
            if(minimumSignedDistance < minimumDistanceRequired){
                return -std::numeric_limits<float>::infinity(); // We are not interested in this point, it is too close to the pole
            }
        }
    }


    auto inside = tree->containsPoint(point);
    if (minimumDistanceRequired > 0 && !inside) {
        return -std::numeric_limits<float>::infinity();
    }

    auto meshDistance = std::sqrt(tree->getShortestDistanceSquared(point));
    meshDistance *= inside ? 1:-1;
    if(meshDistance < minimumSignedDistance){
        minimumSignedDistance = meshDistance;
    }

    return minimumSignedDistance;
}

Sphere InaccessibilityPoles::findNextPoleOfInaccessibility2(const std::shared_ptr<ModelSpaceMesh> &mesh, const std::vector<Sphere> &previousPoles, const std::shared_ptr<BoundingVolumeHierarchy> &tree) {
    /// Inspired by https://github.com/mapbox/polylabel

    // 1. Create a square AABB around the mesh
    auto& modelAABB = mesh->getBounds();
    auto squareAABB = AABB(modelAABB.getMinimum(), modelAABB.getMinimum() + glm::compMax(modelAABB.getMaximum() - modelAABB.getMinimum()));

    // 2. Place the first region on a stack (contrary to the polylabel priority queue)
    auto firstPoint = squareAABB.getCenter();
    auto firstDistance = computeSignedDistanceToMeshAndPolesFlatBVH(tree, firstPoint, previousPoles, -std::numeric_limits<float>::max());
    AABB stack[1024];
    int stackIndex = 0;
    stack[stackIndex++] = squareAABB;

    auto bestDistance = firstDistance;
    auto bestPoint = firstPoint;

    // 3. Refine the approximation of the farthest point
    while (stackIndex >= 0) {

        auto aabb = stack[stackIndex--];

        auto minimumChildDistance = bestDistance - glm::length(aabb.getHalf()); // If lower, we're not interested
        auto childDistance = computeSignedDistanceToMeshAndPolesFlatBVH(tree, aabb.getCenter(), previousPoles, minimumChildDistance);

        if (childDistance > bestDistance) {
            bestPoint = aabb.getCenter();
            bestDistance = childDistance;
        }

        auto maxPotentialChildDistance = childDistance + glm::length(aabb.getHalf());

        if(maxPotentialChildDistance < bestDistance + 1e-3f){
            continue; // Don't add this AABB the queue, the best distance can't be improved in its region
        }

        // 4. Subdivide the AABB and add the children to the queue
        const auto& min = aabb.getMinimum();
        const auto& cen = aabb.getCenter();
        const auto& max = aabb.getMaximum();

        if (glm::length(aabb.getHalf()) > 1e-3f * glm::length(modelAABB.getHalf())) {
            stack[stackIndex++] = AABB(min, cen);
            stack[stackIndex++] = AABB(glm::dvec3(cen.x, min.y, min.z), glm::dvec3(max.x, cen.y, cen.z));
            stack[stackIndex++] = AABB(glm::dvec3(min.x, cen.y, min.z), glm::dvec3(cen.x, max.y, cen.z));
            stack[stackIndex++] = AABB(glm::dvec3(cen.x, cen.y, min.z), glm::dvec3(max.x, max.y, cen.z));
            stack[stackIndex++] = AABB(glm::dvec3(min.x, min.y, cen.z), glm::dvec3(cen.x, cen.y, max.z));
            stack[stackIndex++] = AABB(glm::dvec3(cen.x, min.y, cen.z), glm::dvec3(max.x, cen.y, max.z));
            stack[stackIndex++] = AABB(glm::dvec3(min.x, cen.y, cen.z), glm::dvec3(cen.x, max.y, max.z));
            stack[stackIndex++] = AABB(cen, max);
        }
    }

    // 5. Add the farthest point to the list of poles
    return {bestPoint, bestDistance};
}

Sphere InaccessibilityPoles::findNextPoleOfInaccessibility(const std::shared_ptr<ModelSpaceMesh> &mesh, const std::vector<Sphere> &previousPoles, const std::shared_ptr<BoundingVolumeHierarchy> &tree) {
    /// Inspired by https://github.com/mapbox/polylabel

    // 1. Create a square AABB around the mesh
    auto& modelAABB = mesh->getBounds();
    auto squareAABB = AABB(modelAABB.getMinimum(), modelAABB.getMinimum() + glm::compMax(modelAABB.getMaximum() - modelAABB.getMinimum()));

    // 2. Create a priority queue and add the first
    auto comparator = [](const std::pair<double, AABB>& a, const std::pair<double, AABB>& b) {
        return a.first < b.first; // Compare based on the double value
    };
    std::priority_queue<std::pair<double, AABB>, std::vector<std::pair<double, AABB>>, decltype(comparator)> queue(comparator);

    auto firstPoint = squareAABB.getCenter();
    auto firstDistance = computeSignedDistanceToMeshAndPolesFlatBVH(tree, firstPoint, previousPoles, -std::numeric_limits<float>::max());
    auto firstMaxPotentialDistance = firstDistance + glm::length(squareAABB.getHalf());

    queue.emplace(firstMaxPotentialDistance, squareAABB);
    auto bestDistance = firstDistance;
    auto bestPoint = firstPoint;

    // 3. Refine the approximation of the farthest point
    static auto queries = 0;
    while (!queue.empty()) {
        auto [maxPotentialDistance, aabb] = queue.top();
        queue.pop();

        if (maxPotentialDistance < bestDistance) {
            break; // Points within this AABB cannot improve the best distance, and neither will the rest of the priority queue
        }

        // 4. Subdivide the AABB and add the children to the queue
        const auto& min = aabb.getMinimum();
        const auto& cen = aabb.getCenter();
        const auto& max = aabb.getMaximum();

        auto addToQueue = [&](const AABB& child) {

            if(glm::length(child.getHalf()) < 1e-3f * glm::length(modelAABB.getHalf())){
                return;
            }

            auto minimumChildDistance = bestDistance - glm::length(child.getHalf()); // If lower, we're not interested

            auto childDistance = computeSignedDistanceToMeshAndPolesFlatBVH(tree, child.getCenter(), previousPoles, minimumChildDistance);

            queries++;

            if (childDistance > bestDistance) {
                bestPoint = child.getCenter();
                bestDistance = childDistance;
            }

            auto maxPotentialChildDistance = childDistance + glm::length(child.getHalf());

            if(maxPotentialChildDistance < std::max(0.0f, bestDistance + 1e-3f)){
                return; // Don't add this AABB the queue, the best distance can't be improved in its region
            }
            assert(maxPotentialChildDistance>0.0f);
            queue.emplace(maxPotentialChildDistance, child);
        };

        addToQueue(AABB(min, cen));
        addToQueue(AABB(glm::dvec3(cen.x, min.y, min.z), glm::dvec3(max.x, cen.y, cen.z)));
        addToQueue(AABB(glm::dvec3(min.x, cen.y, min.z), glm::dvec3(cen.x, max.y, cen.z)));
        addToQueue(AABB(glm::dvec3(cen.x, cen.y, min.z), glm::dvec3(max.x, max.y, cen.z)));
        addToQueue(AABB(glm::dvec3(min.x, min.y, cen.z), glm::dvec3(cen.x, cen.y, max.z)));
        addToQueue(AABB(glm::dvec3(cen.x, min.y, cen.z), glm::dvec3(max.x, cen.y, max.z)));
        addToQueue(AABB(glm::dvec3(min.x, cen.y, cen.z), glm::dvec3(cen.x, max.y, max.z)));
        addToQueue(AABB(cen, max));
    }

    // 5. Add the farthest point to the list of poles
    return {bestPoint, bestDistance};
}

std::vector<Sphere> InaccessibilityPoles::computePolesOfInaccessibility(const std::shared_ptr<ModelSpaceMesh>& mesh, size_t numberOfPoles){

    static std::unordered_map<std::pair<size_t,std::shared_ptr<ModelSpaceMesh>>, std::vector<Sphere>> cache;

    // Check if the result is already cached
    if (cache.find({numberOfPoles, mesh}) != cache.end()) {
        return cache[{numberOfPoles, mesh}];
    }

    auto tree = CachingBoundsTreeFactory<BoundingVolumeHierarchy>::getBoundsTree(mesh);

    // If not cached, iteratively
    std::vector<Sphere> poles;
    for (int i = 0; i < numberOfPoles; ++i){
        auto nextPole = findNextPoleOfInaccessibility(mesh, poles, tree);
        poles.emplace_back(nextPole);
    }

    // Cache the result
    cache[{numberOfPoles, mesh}] = poles;

    return poles;
}