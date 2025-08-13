//
// Created by Jonas Tollenaere on 16/07/2025.
//

#include "../include/DiscreteRotationStripPackingTask.h"

#include <fstream>

#include <meshcore/acceleration/CachingBoundsTreeFactory.h>
#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/geometric/Intersection.h>
#include <meshcore/optimization/StripPackingProblem.h>

#include "InaccessibilityPoles.h"

bool DiscreteRotationStripPackingTask::is_contained(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndex) {
    return solution->getProblem()->getContainer().containsAABB(solution->getItemAABB(itemIndex));
}

bool DiscreteRotationStripPackingTask::collide(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndexA, size_t itemIndexB) {

    // First quick reject if bounding boxes do not intersect
    auto boxA = solution->getItemAABB(itemIndexA); // Store the boxes for different computations, avoid recomputing them
    auto boxB = solution->getItemAABB(itemIndexB);
    if (!Intersection::intersect(boxA, boxB)) {
        return false; // Bounding boxes do not intersect
    }

    // Quick intersection guarantee when poles of inaccessibility intersect
    Transformation transformationAtoB = solution->getItemTransformation(itemIndexB).getInverse() * solution->getItemTransformation(itemIndexA);
    for (const auto & pA : solution->getPolesOfInaccessibility(itemIndexA)) {
        auto poleA = pA.getTransformed(transformationAtoB);
        for (const auto & poleB : solution->getPolesOfInaccessibility(itemIndexB)) {
            if (Intersection::intersect(poleA, poleB)) {
                return true;
            }
        }
    }

    // Actual intersection test (default pipeline, reworked)
    return Intersection::intersect(*solution->getSimplifiedItem(itemIndexA), *solution->getSimplifiedItem(itemIndexB));
}

float DiscreteRotationStripPackingTask::overlap_proxy(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndexA, size_t itemIndexB) {
    auto result = 0.0f;
    for (const auto & pA : solution->getPolesOfInaccessibility(itemIndexA)) {
        auto poleA = pA.getTransformed(solution->getItemTransformation(itemIndexA));
        for (const auto & pB : solution->getPolesOfInaccessibility(itemIndexB)) {
            auto poleB = pB.getTransformed(solution->getItemTransformation(itemIndexB));
            const auto penetrationDepth = poleA.getRadius() + poleB.getRadius() - glm::length(poleA.getCenter() - poleB.getCenter());
            if (penetrationDepth > 0) result += penetrationDepth * std::min(poleA.getRadius(), poleB.getRadius());
        }
    }
    return result;
}

float DiscreteRotationStripPackingTask::overlap_proxy_decay(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution, size_t itemIndexA, size_t itemIndexB) {
    auto eps = 1e-2f * std::max(
                   glm::length(solution->getSimplifiedItem(itemIndexA)->getModelSpaceMesh()->getBounds().getHalf()),
                   glm::length(solution->getSimplifiedItem(itemIndexB)->getModelSpaceMesh()->getBounds().getHalf())
               );


    const Transformation transformationAtoB = solution->getItemTransformation(itemIndexB).getInverse() * solution->getItemTransformation(itemIndexA);

    auto result = 0.0f;
    assert(!solution->getPolesOfInaccessibility(itemIndexA).empty());
    assert(!solution->getPolesOfInaccessibility(itemIndexB).empty());
    for (const auto & pA : solution->getPolesOfInaccessibility(itemIndexA)) {
        auto poleA = pA.getTransformed(transformationAtoB);
        for (const auto & poleB : solution->getPolesOfInaccessibility(itemIndexB)) {
            auto delta = poleA.getRadius() + poleB.getRadius() - glm::length(poleA.getCenter() - poleB.getCenter());
            auto deltaPrime = delta > eps ? delta : eps * eps/(2*eps - delta) ;
            result += deltaPrime * std::min(poleA.getRadius(), poleB.getRadius());
        }
    }
    assert(result > 0.0f);
    assert(result >= overlap_proxy(solution, itemIndexA, itemIndexB)); // Decay should not reduce the overlap
    return result;
}

float DiscreteRotationStripPackingTask::quantify_collision(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution, size_t itemIndexA, size_t itemIndexB) {
    auto alpha = overlap_proxy_decay(solution, itemIndexA, itemIndexB);
    auto lambdaA = std::pow(solution->getSimplifiedItem(itemIndexA)->getModelSpaceMesh()->getConvexHull()->getVolume(), 1.0f/3.0f);
    auto lambdaB = std::pow(solution->getSimplifiedItem(itemIndexB)->getModelSpaceMesh()->getConvexHull()->getVolume(), 1.0f/3.0f);
    auto lambdaAB = std::sqrt(lambdaA * lambdaB);
    auto result = lambdaAB * std::pow(alpha, 1.0f/3.0f); // Mapping "volume of overlap" to 1D metric
    assert(result > 0.0f);
    return result;
}

std::optional<AABB> DiscreteRotationStripPackingTask::getValidTranslationRange(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution, float containerHeight, size_t itemIndex) {
    const auto& item = solution->getItem(itemIndex);

    auto container = solution->getProblem()->getContainer();

    auto containerMinimum = container.getMinimum();
    auto containerMaximum = container.getMaximum();
    containerMaximum.z = containerHeight;

    auto aabb = solution->getItemAABB(itemIndex);

    auto min = containerMinimum - aabb.getMinimum();
    auto max = containerMaximum - aabb.getMaximum();

    if (!glm::all(glm::lessThanEqual(min, max + 1e-6f))) {
        return std::nullopt;
    }

    // AABB is computed with current translation, compensate for this;
    min += item->getModelTransformation().getPosition();
    max += item->getModelTransformation().getPosition();

    return AABB{min, glm::max(min,max)};
}

std::vector<bool> DiscreteRotationStripPackingTask::getCollidingItems(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution) {
    const size_t numberOfItems = solution->getNumberOfItems();
    std::vector collidingItems(numberOfItems, false);
    for (size_t i = 0; i < numberOfItems; ++i) {
        for (size_t j = i + 1; j < numberOfItems; ++j) {
            if (collide(solution, i, j)) {
                collidingItems[i] = true;
                collidingItems[j] = true;
            }
        }
    }
    return collidingItems;
}

float DiscreteRotationStripPackingTask::evaluate_item_sample(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution, size_t itemIndex,
    const std::map<std::pair<size_t, size_t>, float> &collisionWeights) {

    auto result = 0.0f;

    for (size_t otherItemIndex=0; otherItemIndex<solution->getNumberOfItems(); ++otherItemIndex) {

        if (otherItemIndex == itemIndex) continue; // Self collision should not be considered

        if (collide(solution, itemIndex, otherItemIndex)) {
            assert(collisionWeights.find({std::min(itemIndex, otherItemIndex),
                std::max(itemIndex, otherItemIndex)}) != collisionWeights.end());
            auto collisionWeight = collisionWeights.find({std::min(itemIndex, otherItemIndex), std::max(itemIndex, otherItemIndex)})->second;
            result += quantify_collision(solution, itemIndex, otherItemIndex) * collisionWeight;
        }
    }
    return result;
}

float DiscreteRotationStripPackingTask::sample_angle(const Random &random, size_t steps) {
    size_t step = random.nextUnsignedInteger(0, steps - 1);
    float angle = 2.0f * glm::pi<float>() * static_cast<float>(step) / static_cast<float>(steps);
    return angle;
}

Quaternion DiscreteRotationStripPackingTask::sample_rotation(const Random &random) {
    const auto angleIndex = random.nextInteger(0, 2);
    const auto randomAngle = sample_angle(random, nRotationAngles);
    auto YPR = glm::vec3(0.0f, 0.0f, 0.0f);
    YPR[angleIndex] = randomAngle;
    return Quaternion(YPR.x, YPR.y, YPR.z);
}

glm::vec3 DiscreteRotationStripPackingTask::sample_position(const AABB &range, const Random &random) {
    glm::vec3 position;
    position.x = (range.getMinimum().x < range.getMaximum().x) ? random.nextFloat(range.getMinimum().x, range.getMaximum().x) : range.getMinimum().x;
    position.y = (range.getMinimum().y < range.getMaximum().y) ? random.nextFloat(range.getMinimum().y, range.getMaximum().y) : range.getMinimum().y;
    position.z = (range.getMinimum().z < range.getMaximum().z) ? random.nextFloat(range.getMinimum().z, range.getMaximum().z) : range.getMinimum().z;
    return position;
}

void DiscreteRotationStripPackingTask::search_position(std::shared_ptr<EnhancedStripPackingSolution> &solution,
                                                       float containerHeight, size_t itemIndex, const Random &random,
                                                       const std::map<std::pair<size_t, size_t>, float> &collisionWeights) {

    auto& item = solution->getItem(itemIndex);
    const auto initialPosition = item->getModelTransformation().getPosition();
    const auto initialRotation = item->getModelTransformation().getRotation();

    // Store sampled evaluations, sorted by placement quality
    std::map<float, std::pair<Quaternion, glm::vec3>> sampledPlacements;

    auto originalEvaluation = evaluate_item_sample(solution, itemIndex, collisionWeights);
    sampledPlacements[originalEvaluation] = {initialRotation, initialPosition};

    // Uniform samples in container
    for (auto sampleIndex = 0; sampleIndex < 100; ++sampleIndex) {

        Quaternion sampledRotation = sample_rotation(random);

        // Set the item's rotation to the sampled rotation
        Transformation newTransformation = item->getModelTransformation();
        newTransformation.setRotation(sampledRotation);
        solution->setItemTransformation(itemIndex, newTransformation);

        auto validTranslationRange = getValidTranslationRange(solution, containerHeight, itemIndex);

        if (!validTranslationRange.has_value()) {
            continue; // We won't be able to find a valid position in the container for this orientation
        }

        glm::vec3 sampledPosition = sample_position(validTranslationRange.value(), random);

        // Set the item's position to the sampled position
        newTransformation.setPosition(sampledPosition);
        solution->setItemTransformation(itemIndex, newTransformation);

        assert(is_contained(solution, itemIndex));

        // Evaluate the placement quality
        sampledPlacements[evaluate_item_sample(solution, itemIndex, collisionWeights)] = {sampledRotation, sampledPosition};
    }

    // Rotated samples near the initial position
    // Seems to work really well, but need to compensate for the shift of the item center
    // After all; our items reference in model space is the minimum corner of its AABB, not the center
    for (auto sampleIndex = 0; sampleIndex < 25; ++sampleIndex) {

        Quaternion sampledRotation = sample_rotation(random);

        auto modelSpaceCenter = solution->getSimplifiedItem(itemIndex)->getModelSpaceMesh()->getCenter();
        auto initialCenterPosition = initialRotation.rotateVertex(modelSpaceCenter);
        auto newCenterPosition = sampledRotation.rotateVertex(modelSpaceCenter);
        auto centerShift = newCenterPosition - initialCenterPosition; // How much the center of the item is shifted in going to new rotation

        // Set the item's rotation to the sampled rotation
        Transformation newTransformation = item->getModelTransformation();
        newTransformation.setRotation(sampledRotation);
        solution->setItemTransformation(itemIndex, newTransformation);

        auto validTranslationRange = getValidTranslationRange(solution, containerHeight, itemIndex);

        if (!validTranslationRange.has_value()) {
            continue; // We won't be able to find a valid position in the container for this orientation
        }

        const auto containerHalf = solution->getProblem()->getContainer().getHalf();
        const auto& stepSize = 0.01f * (containerHalf.x + containerHalf.y);
        glm::vec3 randomShift(random.nextFloat(-stepSize, stepSize),
                              random.nextFloat(-stepSize, stepSize),
                              random.nextFloat(-stepSize, stepSize));

        randomShift -= centerShift; // Compensate for the shift of the item center

        // Clamp to initial+shift to valid positions;
        auto newPosition = validTranslationRange.value().getClosestPoint(initialPosition + randomShift);
        assert(validTranslationRange.value().containsPoint(newPosition));

        // Set the item's position to the sampled position
        newTransformation.setPosition(newPosition);
        solution->setItemTransformation(itemIndex, newTransformation);

        assert(is_contained(solution, itemIndex));

        // Evaluate the placement quality
        sampledPlacements[evaluate_item_sample(solution, itemIndex, collisionWeights)] = {sampledRotation, newPosition};
    }

    // Restore the original rotation
    Transformation originalTransformation = item->getModelTransformation();
    originalTransformation.setRotation(initialRotation);
    originalTransformation.setPosition(initialPosition);
    solution->setItemTransformation(itemIndex, originalTransformation);

    // Samples in neighborhood of item's current position
    {
        auto validTranslationRange = getValidTranslationRange(solution, containerHeight, itemIndex);
        assert(validTranslationRange.has_value());
        for (auto sampleIndex = 0; sampleIndex < 25; ++sampleIndex) {
            const auto containerHalf = solution->getProblem()->getContainer().getHalf();
            const auto& stepSize = 0.1f * (containerHalf.x + containerHalf.y);
            glm::vec3 randomShift(random.nextFloat(-stepSize, stepSize),
                                  random.nextFloat(-stepSize, stepSize),
                                  random.nextFloat(-stepSize, stepSize));

            randomShift *= glm::length(solution->getProblem()->getContainer().getHalf());

            // Clamp to initial+shift to valid positions;
            auto newPosition = validTranslationRange.value().getClosestPoint(initialPosition + randomShift);
            assert(validTranslationRange.value().containsPoint(newPosition));

            // Set the item's position to the sampled position
            Transformation newTransformation = item->getModelTransformation();
            newTransformation.setPosition(newPosition);
            solution->setItemTransformation(itemIndex, newTransformation);

            assert(is_contained(solution, itemIndex));

            // Evaluate the placement quality
            sampledPlacements[evaluate_item_sample(solution, itemIndex, collisionWeights)] = {initialRotation, newPosition};
        }
    }

    // Refine best positions
    constexpr auto POSITIONS_TO_REFINE = 3;
    glm::vec3 bestPositions[POSITIONS_TO_REFINE];
    Quaternion bestRotations[POSITIONS_TO_REFINE];
    float bestEvaluations[POSITIONS_TO_REFINE];
    // Place the first POSITIONS_TO_REFINE best positions in the array
    auto it = sampledPlacements.begin();
    for (auto i = 0; i < POSITIONS_TO_REFINE && it != sampledPlacements.end(); ++i, ++it) {
        auto& placement = it->second;
        bestPositions[i] = placement.second;
        bestEvaluations[i] = it->first;
        bestRotations[i] = placement.first;
    }

    // Pattern search
    for(auto i = 0; i < POSITIONS_TO_REFINE; ++i) {
        auto currentPosition = bestPositions[i];
        auto currentRotation = bestRotations[i];
        auto currentEvaluation = bestEvaluations[i];
        auto stepSize = 0.01f;
        while(stepSize >= 1e-4f && currentEvaluation > 0.0f) {
            bool improved = false;

            glm::vec3 bestNeighborPosition = currentPosition;
            float bestNeighborEvaluation = currentEvaluation;

            for (auto step : {glm::vec3(stepSize, 0, 0), glm::vec3(-stepSize, 0, 0),
                     glm::vec3(0, stepSize, 0), glm::vec3(0, -stepSize, 0),
                     glm::vec3(0, 0, stepSize), glm::vec3(0, 0, -stepSize)}) {

                step *= glm::length(solution->getProblem()->getContainer().getHalf());

                Transformation newTransformation = item->getModelTransformation();
                newTransformation.setRotation(currentRotation);
                newTransformation.setPosition(currentPosition);
                solution->setItemTransformation(itemIndex, newTransformation);

                auto validTranslationRange = getValidTranslationRange(solution, containerHeight, itemIndex);
                assert(validTranslationRange.has_value());

                // Clamp the new position to the valid translation range
                auto newPosition = validTranslationRange.value().getClosestPoint(currentPosition + step);
                assert(validTranslationRange.value().containsPoint(newPosition));

                newTransformation.setPosition(newPosition);
                solution->setItemTransformation(itemIndex, newTransformation);

                auto newEvaluation = evaluate_item_sample(solution, itemIndex, collisionWeights);
                if (newEvaluation < bestNeighborEvaluation) {

                    bestNeighborPosition = newPosition;
                    bestNeighborEvaluation = newEvaluation;

                    if (currentEvaluation <= 0.0f) {
                        // If we found a position with negative evaluation, we can stop
                        Transformation finalTransformation = item->getModelTransformation();
                        finalTransformation.setPosition(newPosition);
                        solution->setItemTransformation(itemIndex, finalTransformation);
                        return;
                    }

                    improved = true;

                    bool hill_climb = true;
                    if (hill_climb) break;
                }

            }
            if (!improved) {
                stepSize *= 0.5f; // Reduce step size
            }
            else {

                // Update the current position and evaluation
                currentPosition = bestNeighborPosition;
                currentEvaluation = bestNeighborEvaluation;;
            }
        }

        assert(is_contained(solution, itemIndex));

        sampledPlacements[currentEvaluation] = {currentRotation, currentPosition};
    }

    // Apply the best sample
    auto bestSample = sampledPlacements.begin()->second;
    assert(sampledPlacements.begin()->first <= originalEvaluation);
    Transformation newTransformation = item->getModelTransformation();
    newTransformation.setPosition(bestSample.second);
    newTransformation.setRotation(bestSample.first);
    solution->setItemTransformation(itemIndex, newTransformation);

    assert(is_contained(solution, itemIndex));
}

void DiscreteRotationStripPackingTask::update_collision_metrics(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution, CollisionMetrics &metrics) {

    // Find intersecting item pairs
    metrics.collision = false;
    metrics.worstCollisionQuantity = 0.0f;
    metrics.totalCollisionQuantity = 0.0f;
    for (auto i = 0; i < solution->getNumberOfItems(); ++i) {
        for (auto j = i + 1; j < solution->getNumberOfItems(); ++j) {

            /*if (!metrics.collidingItems[i] && !metrics.collidingItems[j]) {
                    // If two items previously did not intersect, they will not have been moved
                    assert(!collide(*solution.getItems()[i], *solution.getItems()[j]));
                    metrics.collidingItemPairs[{i, j}] = false;
                    continue;
                }*/

            const bool isColliding = collide(solution, i, j);
            metrics.collidingItemPairs[{i, j}] = isColliding;
            metrics.collision |= isColliding;

            if (isColliding) {
                auto quantity = quantify_collision(solution, i, j);
                metrics.collisionQuantities[{i, j}] = quantity;
                metrics.totalCollisionQuantity += quantity;
                metrics.worstCollisionQuantity = std::max(metrics.worstCollisionQuantity, quantity);
            }
            else {
                metrics.collisionQuantities[{i, j}] = 0.0f;
            }
        }
    }

    // Update the list of colliding items
    std::fill(metrics.collidingItems.begin(), metrics.collidingItems.end(), false);
    if (metrics.collision) {
        for (const auto & [pair, collide] : metrics.collidingItemPairs) {
            if (collide) {
                metrics.collidingItems[pair.first] = true;
                metrics.collidingItems[pair.second] = true;
            }
        }
    }
}

void DiscreteRotationStripPackingTask::move_colliding_items(std::shared_ptr<EnhancedStripPackingSolution> &solution,
                                                            float currentHeight, CollisionMetrics metrics, const std::map<std::pair<size_t, size_t>, float> &collisionWeights,
                                                            const Random &random) {

    // Iterate over the items in random order
    std::vector<size_t> itemIndices(solution->getNumberOfItems());
    std::iota(itemIndices.begin(), itemIndices.end(), 0);
    std::shuffle(itemIndices.begin(), itemIndices.end(), boost::random::mt19937(random.nextInteger()));
    for (const auto& itemIndex : itemIndices) {

        // Continue if not colliding
        if (!metrics.collidingItems[itemIndex]) continue;

        // Search for a better position for the item
        search_position(solution, currentHeight, itemIndex, random, collisionWeights);

        assert(is_contained(solution, itemIndex));
    }
}

DiscreteRotationStripPackingTask::CollisionMetrics DiscreteRotationStripPackingTask::move_colliding_items_multi(
    std::shared_ptr<EnhancedStripPackingSolution> &solution, float currentHeight, const CollisionMetrics &metrics,
    const std::map<std::pair<size_t, size_t>, float> &collisionWeights, const Random &random) {

    // Iterate over items in multiple random orders and take the best result
    constexpr size_t N_WORKERS = 4;
    int seeds[N_WORKERS];
    std::shared_ptr<EnhancedStripPackingSolution> workerSolutions[N_WORKERS];
    CollisionMetrics workerMetrics[N_WORKERS];
    float newTotalCollisionQuantities[N_WORKERS];
    for (size_t i = 0; i < N_WORKERS; ++i) {
        seeds[i] = random.nextInteger();
        workerSolutions[i] = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone());
        workerMetrics[i] = metrics;
    }

    for (size_t i = 0; i < N_WORKERS; ++i) {
        //tbb::parallel_for<size_t>(0, N_WORKERS, [&workerSolutions, &seeds, &workerMetrics, &newTotalCollisionQuantities, &currentHeight, &collisionWeights](const size_t i) {

        auto& workerSolution = workerSolutions[i];

        const Random random(seeds[i]);

        std::vector<size_t> itemIndices(workerSolution->getNumberOfItems());
        std::iota(itemIndices.begin(), itemIndices.end(), 0);
        std::shuffle(itemIndices.begin(), itemIndices.end(), boost::random::mt19937(random.nextInteger()));
        for (const auto& itemIndex : itemIndices) {

            // Skip item if not colliding
            if (!workerMetrics[i].collidingItems[itemIndex]) continue;

            // Search for a better position for this item
            search_position(workerSolution, currentHeight, itemIndex, random, collisionWeights);
        }

        update_collision_metrics(workerSolution, workerMetrics[i]);
        newTotalCollisionQuantities[i] = workerMetrics[i].totalCollisionQuantity;
        //});
    }

    // Find the index with the minimum total collision quantity
    size_t bestIndex = 0;
    for (size_t i = 1; i < N_WORKERS; ++i) {
        if (newTotalCollisionQuantities[i] < newTotalCollisionQuantities[bestIndex]) {
            bestIndex = i;
        }
    }

    solution = workerSolutions[bestIndex];
    return workerMetrics[bestIndex];
}

void DiscreteRotationStripPackingTask::iterate_weights(std::map<std::pair<size_t, size_t>, float> &collisionWeights,
                                                       const EnhancedStripPackingSolution &solution, std::map<std::pair<size_t, size_t>, float> &collisionQuantities,
                                                       const std::map<std::pair<size_t, size_t>, bool> &collidingItemPairs, const float e_max) {

    // Update collision weights based on the current collisions
    for (size_t i = 0; i < solution.getItems().size(); ++i) {
        for (size_t j = i + 1; j < solution.getItems().size(); ++j) {

            float factor;
            assert(collidingItemPairs.find({i, j}) != collidingItemPairs.end());
            if (collidingItemPairs.find({i, j})->second) {
                auto quantity = collisionQuantities[{i, j}];
                factor = 1.05f + (1.5f - 1.05f) * quantity/e_max; // a factor in between 1.05 and 1.5
            }
            else {
                factor = 0.95f;
            }

            auto originalWeight = collisionWeights[{i, j}];
            auto newWeight = std::max(1.0f, factor * originalWeight);

            if (newWeight > 1e8f) {
                std::cout << "Collision weight for items " << i << " and " << j << " exceeded threshold: " << newWeight << std::endl;
                newWeight = 1e8f; // Cap the weight to prevent overflow
            }

            collisionWeights[{i, j}] = newWeight;
        }
    }
}

std::map<std::pair<size_t, size_t>, float> DiscreteRotationStripPackingTask::init_weights(
    const EnhancedStripPackingSolution &solution) {
    std::map<std::pair<size_t, size_t>, float> collisionWeights;
    for (size_t i = 0; i < solution.getItems().size(); ++i) {
        for (size_t j = i + 1; j < solution.getItems().size(); ++j) {
            collisionWeights[{i, j}] = 1.0f; // Initial weight
        }
    }
    return collisionWeights;
}

bool DiscreteRotationStripPackingTask::separate(std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t maxAttempts, size_t maxIterationsWithoutImprovement, float currentHeight, Random &random) {

    auto n = 0; // Strike counter

    // Initialize collision metrics and weights
    CollisionMetrics metrics(solution);
    update_collision_metrics(solution, metrics);
    auto collisionWeights = init_weights(*solution);

    // Keep track of the solution with the lowest collision quantity
    auto bestSolution = solution;
    auto bestCollisionQuantity = metrics.totalCollisionQuantity;

    while (n < maxAttempts && !stopCalled && elapsedMilliseconds < allowedRunTimeMilliseconds) {

        auto currentSolution = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(bestSolution->clone());
        std::fill(metrics.collidingItems.begin(), metrics.collidingItems.end(), true);
        update_collision_metrics(currentSolution, metrics);
        auto initialCollisionQuantity = metrics.totalCollisionQuantity;

        collisionWeights = init_weights(*solution);

        auto i = 0;
        while (i<maxIterationsWithoutImprovement && !stopCalled) {

            if (true) {
                move_colliding_items(currentSolution, currentHeight, metrics, collisionWeights, random);
                update_collision_metrics(currentSolution, metrics);
            }
            else {
                metrics = move_colliding_items_multi(currentSolution, currentHeight, metrics, collisionWeights, random);
            }

            //notifyObserversSolution(currentSolution);

            iterate_weights(collisionWeights, *currentSolution, metrics.collisionQuantities, metrics.collidingItemPairs, metrics.worstCollisionQuantity);
            i++;

            if (!metrics.collision) {
                std::cout << "Found feasible solution with height " << currentHeight << " (" << currentSolution->isFeasible() << ")" << std::endl;
                std::cout << std::endl;
                solution = currentSolution;
                notifyObserversSolution(solution);
                /*
                    assert(solution->isFeasible());
                    if (!solution->isFeasible()) {

                        for (int itemIndex = 0; itemIndex < solution->getItems().size(); ++itemIndex) {
                            auto validRange = getValidTranslationRange(*solution, currentHeight, itemIndex);
                            auto inValidRange = validRange.containsPoint(solution->getItems()[itemIndex]->getModelTransformation().getPosition());
                            std::cout << itemIndex << ": " << inValidRange << std::endl;
                        }

                        bool result = solution->isFeasible();
                        std::cout << "Infeasible solution in iteration " << std::to_string(i) << std::endl;
                    }*/
                return true;
            }

            if (metrics.totalCollisionQuantity < bestCollisionQuantity) {
                std::cout << "Improved total collision quantity to " << metrics.totalCollisionQuantity << std::endl;
                //notifyObserversSolution(bestSolution);
                bestSolution = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(currentSolution->clone());
                bestCollisionQuantity = metrics.totalCollisionQuantity;
                i = 0;
            }
        }

        std::cout << std::to_string(maxIterationsWithoutImprovement) << " iterations without improvement, restarting with best solution" << std::endl;

        if (bestCollisionQuantity < initialCollisionQuantity) {
            n = 0; // Reset the strike counter if we improved
        }
        else {
            std::cout << "No improvement in collision metric, adding a strike." << std::endl;
            n++; // Add a strike
        }

        elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startMilliseconds;
    }

    std::cout << "Failed to improve collision metric after " << maxAttempts << " attempts, returning overlapping solution." << std::endl;
    solution = bestSolution;
    return false;
}

std::shared_ptr<EnhancedStripPackingSolution> DiscreteRotationStripPackingTask::explore(
    std::shared_ptr<EnhancedStripPackingSolution> &solution, float initialHeight, float minimumHeight, const std::vector<Quaternion>& minimumHeightRotations) {

    Random random(seed);
    auto bestSolution = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone());
    startMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    auto currentHeight = initialHeight;

    std::vector<std::shared_ptr<EnhancedStripPackingSolution>> solutionsPool;

    while (elapsedMilliseconds < allowedRunTimeMilliseconds && !stopCalled) {
        bool separated = separate(solution, 3, 100, currentHeight, random);
        if (separated) {
            notifyObserversSolution(solution);
            notifyObserversStatus("Achieved height " + std::to_string(currentHeight));
            std::cout << "Found feasible solution with height " << currentHeight << " (" << solution->isFeasible() << ") after " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startMilliseconds << "ms" << std::endl;
            std::cout << std::endl;
            bestSolution = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone());

            if (currentHeight == minimumHeight) {
                return bestSolution; // We achieved the lower bound height (largest item), no need to continue
            }

            // shrink strip
            constexpr auto REDUCTION_FACTOR = 0.995f;
            currentHeight *= REDUCTION_FACTOR;
            currentHeight = std::max(currentHeight, minimumHeight);
            for (auto itemIndex = 0; itemIndex < solution->getItems().size(); ++itemIndex) {
                auto& item = solution->getItems()[itemIndex];
                auto transformation = item->getModelTransformation();

                auto validTranslationRange = getValidTranslationRange(solution, currentHeight, itemIndex);
                if (!validTranslationRange.has_value()) {

                    // No valid transformation is found in the container with decreased height when the item has this rotation
                    // Set the rotation at which the item has minimum height
                    transformation.setRotation(minimumHeightRotations[itemIndex]);
                    solution->setItemTransformation(itemIndex, transformation);
                    validTranslationRange = getValidTranslationRange(solution, currentHeight, itemIndex);
                    assert(validTranslationRange.has_value());

                    // Place the rotated item at a random position within the valid translation range
                    auto newPosition = sample_position(validTranslationRange.value(), random);
                    transformation.setPosition(newPosition);
                    solution->setItemTransformation(itemIndex, transformation);
                }
                else {

                    // Decrease the z-position of the item with REDUCTION_FACTOR as well, keep it clamped within valid translation ranges
                    transformation.setPositionZ(glm::clamp(transformation.getPosition().z * REDUCTION_FACTOR, validTranslationRange->getMinimum().z, validTranslationRange->getMaximum().z));
                    solution->setItemTransformation(itemIndex, transformation);
                }
            }

            solutionsPool.clear();
        }
        else {
            std::cout << "Disrupting solution." << std::endl;
            solutionsPool.push_back(std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone()));
            auto solutionToDisrupt = solutionsPool[random.nextInteger(0, solutionsPool.size() - 1)];

            // Don't modify the solutions in the pool!
            solutionToDisrupt = std::static_pointer_cast<EnhancedStripPackingSolution>(solutionToDisrupt->clone());

            auto itemIndexA = random.nextInteger(0, solutionToDisrupt->getItems().size() - 1);
            auto itemIndexB = random.nextInteger(0, solutionToDisrupt->getItems().size() - 2);
            if (itemIndexB == itemIndexA) itemIndexB++;

            auto newPosA = solutionToDisrupt->getItems()[itemIndexB]->getModelTransformation().getPosition();
            auto newPosB = solutionToDisrupt->getItems()[itemIndexA]->getModelTransformation().getPosition();
            newPosA = getValidTranslationRange(solutionToDisrupt, currentHeight, itemIndexA).value().getClosestPoint(newPosA);
            newPosB = getValidTranslationRange(solutionToDisrupt, currentHeight, itemIndexB).value().getClosestPoint(newPosB);

            auto newTransformationA = solutionToDisrupt->getItems()[itemIndexA]->getModelTransformation();
            newTransformationA.setPosition(newPosA);
            solutionToDisrupt->setItemTransformation(itemIndexA, newTransformationA);

            auto newTransformationB = solutionToDisrupt->getItems()[itemIndexB]->getModelTransformation();
            newTransformationB.setPosition(newPosB);
            solutionToDisrupt->setItemTransformation(itemIndexB, newTransformationB);

            solution = solutionToDisrupt;
        }

        elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startMilliseconds;
    }

    return bestSolution;
}

void DiscreteRotationStripPackingTask::run() {

    this->notifyObserversStatus("Initialising");

    // Create and notify the solution
    notifyObserversStatus("Constructing solution");
    auto startms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto solution = std::make_shared<EnhancedStripPackingSolution>(problem);
    auto endms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout << "Creating solution took " << (endms-startms) << "ms" << std::endl;

    // Create a trivial solution
    auto trivialHeight = 0.0f;
    auto minimumHeight = 0.0f;
    std::vector<Quaternion> minimumHeightRotations;
    {
        notifyObserversStatus("Creating trivial solution");
        auto currentHeight = 0.0f;
        for (size_t itemIndex = 0; itemIndex < solution->getNumberOfItems(); ++itemIndex) {
            auto& item = solution->getItem(itemIndex);
            const AABB& itemBounds = item->getModelSpaceMesh()->getBounds();

            Transformation transformation;
            transformation.setPositionX(-itemBounds.getMinimum().x);
            transformation.setPositionY(-itemBounds.getMinimum().y);
            transformation.setPositionZ(currentHeight);
            solution->setItemTransformation(itemIndex, transformation);
            auto itemSize = itemBounds.getMaximum() - itemBounds.getMinimum();
            currentHeight += itemSize.z;

            // Derive minimum item height
            // We have to account for all possible rotations
            auto minimumItemHeight = itemSize.z;
            auto minimumItemHeightRotation = item->getModelTransformation().getRotation();
            auto containerSize = problem->getContainer().getMaximum() - problem->getContainer().getMinimum();
            assert(containerSize.x >= itemSize.x && containerSize.y >= itemSize.y);

            for (size_t angleIndex = 0; angleIndex < 2; ++angleIndex) {
                for (size_t step = 0; step < nRotationAngles; ++step) {
                    // Sample a rotation angle
                    // We sample the angle in a discrete manner, so we can use it to rotate the item
                    // around the Z-axis, Y-axis or X-axis
                    // The angle is sampled uniformly from [0, 2pi]
                    float angle = 2.0f * glm::pi<float>() * static_cast<float>(step) / static_cast<float>(nRotationAngles);
                    auto YPR = glm::vec3(0.0f, 0.0f, 0.0f);
                    YPR[angleIndex] = angle;

                    Quaternion rotation(YPR.x, YPR.y, YPR.z);
                    auto rotatedItemAABB = AABBFactory::createAABB(item->getModelSpaceMesh()->getVertices(), Transformation(rotation));
                    auto rotatedItemSize = rotatedItemAABB.getMaximum()-rotatedItemAABB.getMinimum();

                    // Test if this improves the minimum height and if the item fits in the container under this rotation
                    if (rotatedItemSize.z < minimumItemHeight && containerSize.x >= rotatedItemSize.x && containerSize.y >= rotatedItemSize.y) {
                        minimumItemHeight = rotatedItemSize.z;
                        minimumItemHeightRotation = rotation;
                    }
                }
            }

            // We ensure to never go below this height
            minimumHeight = std::max(minimumHeight, minimumItemHeight);

            // We keep track of the rotations used to achieve these minimal heights as well, so we can guarantee finding placements later on
            minimumHeightRotations.push_back(minimumItemHeightRotation);
        }
        notifyObserversSolution(solution);
        trivialHeight = currentHeight;
    }
    auto currentHeight = 0.5f * trivialHeight;
    currentHeight = std::max(currentHeight, minimumHeight);
    auto bestSolution = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone());

    // Create a random overlapping solution, half the trivial height
    Random random(0);
    notifyObserversStatus("Creating random overlapping solution");
    for (int itemIndex = 0; itemIndex < solution->getItems().size(); ++itemIndex) {
        auto& item = solution->getItems()[itemIndex];
        AABB validTranslationRange = getValidTranslationRange(solution, currentHeight, itemIndex).value();
        glm::vec3 randomPosition = sample_position(validTranslationRange, random);
        auto newTransformation = item->getModelTransformation();
        newTransformation.setPosition(randomPosition);
        solution->setItemTransformation(itemIndex, newTransformation);
    }
    notifyObserversSolution(solution);

    notifyObserversStatus("Starting exploration");
    bestSolution = explore(solution, currentHeight, minimumHeight, minimumHeightRotations);

    notifyObserversSolution(bestSolution);
    result = bestSolution;

    // Export the best solution
    auto solutionJSON = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(bestSolution)->toJson();
    std::ofstream jsonOutputFile(SOLUTION_DIR + problem->getName() + "_Discrete_" + std::to_string(seed) + "_" + std::to_string(bestSolution->computeTotalHeight()) + ".json");
    jsonOutputFile << solutionJSON.dump(4);
    jsonOutputFile.close();
}

void DiscreteRotationStripPackingTask::setSeed(int seed) {
    this->seed = seed;
}

void DiscreteRotationStripPackingTask::setAllowedRunTimeMilliseconds(size_t milliseconds) {
    this->allowedRunTimeMilliseconds = milliseconds;
}

std::shared_ptr<StripPackingSolution> DiscreteRotationStripPackingTask::getResult() const {
    return result;
}