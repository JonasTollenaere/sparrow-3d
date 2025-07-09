//
// Created by Jonas Tollenaere on 05/07/2025.
//

#include "FixedRotationStripPackingTask.h"

#include <fstream>

#include <meshcore/acceleration/CachingBoundsTreeFactory.h>
#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/geometric/Intersection.h>
#include <meshcore/optimization/StripPackingProblem.h>

#include "InaccessibilityPoles.h"

bool FixedRotationStripPackingTask::collide(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
                                            size_t itemIndexA, size_t itemIndexB) {

    const auto& itemA = solution->getSimplifiedItem(itemIndexA);
    const auto& itemB = solution->getSimplifiedItem(itemIndexB);

    // First quick reject if bounding boxes do not intersect
    const auto& translationAtoB = - itemB->getModelTransformation().getPosition() + itemA->getModelTransformation().getPosition();
    auto boxA = solution->getItem(itemIndexA)->getModelSpaceMesh()->getBounds().getTranslated(translationAtoB);
    auto& boxB = solution->getItem(itemIndexB)->getModelSpaceMesh()->getBounds();
    if (!Intersection::intersect(boxA, boxB)) {
        return false; // Bounding boxes do not intersect
    }

    // Quick intersection guarantee when poles of inaccessibility intersect
    for (const auto & pA : solution->getPolesOfInaccessibility(itemIndexA)) {
        auto poleA = Sphere(pA.getCenter() + translationAtoB, pA.getRadius());
        for (const auto & poleB : solution->getPolesOfInaccessibility(itemIndexB)) {
            if (Intersection::intersect(poleA, poleB)) {
                return true;
            }
        }
    }

    // Actual intersection test
    return Intersection::intersect(*itemA, *itemB);
}

float FixedRotationStripPackingTask::overlap_proxy(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndexA, size_t itemIndexB) {
    auto result = 0.0f;
    const glm::vec3 translationAtoB = - solution->getItem(itemIndexB)->getModelTransformation().getPosition() + solution->getItem(itemIndexA)->getModelTransformation().getPosition();
    for (const auto & pA : solution->getPolesOfInaccessibility(itemIndexA)) {
        Sphere poleA(pA.getCenter() + translationAtoB, pA.getRadius());
        for (const auto & poleB : solution->getPolesOfInaccessibility(itemIndexB)) {
            const auto penetrationDepth = poleA.getRadius() + poleB.getRadius() - glm::length(poleA.getCenter() - poleB.getCenter());
            if (penetrationDepth > 0) result += penetrationDepth * std::min(poleA.getRadius(), poleB.getRadius());
        }
    }
    return result;
}

float FixedRotationStripPackingTask::overlap_proxy_decay(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndexA, size_t itemIndexB) {
    auto eps = 1e-2f * std::max(glm::length(solution->getSimplifiedItem(itemIndexA)->getModelSpaceMesh()->getBounds().getHalf()),
                                glm::length(solution->getSimplifiedItem(itemIndexB)->getModelSpaceMesh()->getBounds().getHalf()));

    auto result = 0.0f;
    assert(!solution->getPolesOfInaccessibility(itemIndexA).empty());
    assert(!solution->getPolesOfInaccessibility(itemIndexB).empty());
    const glm::vec3 translationAtoB = - solution->getItem(itemIndexB)->getModelTransformation().getPosition() + solution->getItem(itemIndexA)->getModelTransformation().getPosition();
    for (const auto & pA : solution->getPolesOfInaccessibility(itemIndexA)) {
        Sphere poleA(pA.getCenter()+translationAtoB, pA.getRadius());
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

float FixedRotationStripPackingTask::quantify_collision(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndexA, size_t itemIndexB) {
    auto alpha = overlap_proxy_decay(solution, itemIndexA,itemIndexB);
    auto lambdaA = std::pow(solution->getSimplifiedItem(itemIndexA)->getModelSpaceMesh()->getConvexHull()->getVolume(), 1.0f/3.0f);
    auto lambdaB = std::pow(solution->getSimplifiedItem(itemIndexB)->getModelSpaceMesh()->getConvexHull()->getVolume(), 1.0f/3.0f);
    auto lambdaAB = std::sqrt(lambdaA * lambdaB);
    auto result = lambdaAB * std::pow(alpha, 1.0f/3.0f); // Mapping "volume of overlap" to 1D metric
    assert(result > 0.0f);
    return result;
}

AABB FixedRotationStripPackingTask::getValidTranslationRange(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution, float containerHeight, size_t itemIndex) {

    const auto& item = solution->getItem(itemIndex);

    auto containerMinimum = solution->getProblem()->getContainer().getMinimum();
    auto containerMaximum = solution->getProblem()->getContainer().getMaximum();
    containerMaximum.z = containerHeight;

    auto minimumTranslation = containerMinimum - item->getModelSpaceMesh()->getBounds().getMinimum();
    auto maximumTranslation = containerMaximum - item->getModelSpaceMesh()->getBounds().getMaximum();
    return {minimumTranslation, maximumTranslation};
}

std::vector<bool> FixedRotationStripPackingTask::getCollidingItems(
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

float FixedRotationStripPackingTask::evaluate_item_sample(const std::shared_ptr<EnhancedStripPackingSolution> &solution,
    size_t itemIndex, const std::map<std::pair<size_t, size_t>, float> &collisionWeights) {

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

glm::vec3 FixedRotationStripPackingTask::sample_position(const AABB &range, const Random &random) {
    glm::vec3 position;
    position.x = (range.getMinimum().x < range.getMaximum().x) ? random.nextFloat(range.getMinimum().x, range.getMaximum().x) : range.getMinimum().x;
    position.y = (range.getMinimum().y < range.getMaximum().y) ? random.nextFloat(range.getMinimum().y, range.getMaximum().y) : range.getMinimum().y;
    position.z = (range.getMinimum().z < range.getMaximum().z) ? random.nextFloat(range.getMinimum().z, range.getMaximum().z) : range.getMinimum().z;
    return position;
}

void FixedRotationStripPackingTask::search_position(std::shared_ptr<EnhancedStripPackingSolution> &solution,
    float containerHeight, size_t itemIndex, const Random &random,
    const std::map<std::pair<size_t, size_t>, float> &collisionWeights) {

    auto& item = solution->getItem(itemIndex);
    const auto initialPosition = item->getModelTransformation().getPosition();
    const auto validTranslationRange = getValidTranslationRange(solution, containerHeight, itemIndex);

    // Store sampled evaluations, sorted by placement quality
    std::map<float, glm::vec3> sampledPositions;

    auto originalEvaluation = evaluate_item_sample(solution, itemIndex, collisionWeights);
    sampledPositions[originalEvaluation] = initialPosition;

    // Uniform samples in container
    for (auto sampleIndex = 0; sampleIndex < 25; ++sampleIndex) {
        glm::vec3 samplePosition = sample_position(validTranslationRange, random);

        // Set the item's position to the sampled position
        Transformation newTransformation = item->getModelTransformation();
        newTransformation.setPosition(samplePosition);
        solution->setItemTransformation(itemIndex, newTransformation);

        // Evaluate the placement quality
        sampledPositions[evaluate_item_sample(solution, itemIndex, collisionWeights)] = samplePosition;
    }

    // Samples in neighborhood of item's current position
    for (auto sampleIndex = 0; sampleIndex < 50; ++sampleIndex) {
        const auto& stepSize = 0.01f;
        glm::vec3 randomShift(random.nextFloat(-stepSize, stepSize),
                              random.nextFloat(-stepSize, stepSize),
                              random.nextFloat(-stepSize, stepSize));

        randomShift *= glm::length(solution->getProblem()->getContainer().getHalf());

        // Clamp to initial+shift to valid positions;
        auto newPosition = validTranslationRange.getClosestPoint(initialPosition + randomShift);
        assert(validTranslationRange.containsPoint(newPosition));

        // Set the item's position to the sampled position
        Transformation newTransformation = item->getModelTransformation();
        newTransformation.setPosition(newPosition);
        solution->setItemTransformation(itemIndex, newTransformation);

        // Evaluate the placement quality
        sampledPositions[evaluate_item_sample(solution, itemIndex, collisionWeights)] = newPosition;
    }

    // Refine best positions
    constexpr auto POSITIONS_TO_REFINE = 3;
    glm::vec3 bestPositions[POSITIONS_TO_REFINE];
    float bestEvaluations[POSITIONS_TO_REFINE];
    // Place the first POSITIONS_TO_REFINE best positions in the array
    auto it = sampledPositions.begin();
    for (auto i = 0; i < POSITIONS_TO_REFINE && it != sampledPositions.end(); ++i, ++it) {
        bestPositions[i] = it->second;
        bestEvaluations[i] = it->first;
    }

    // Pattern search
    for(auto i = 0; i < POSITIONS_TO_REFINE; ++i) {
        auto currentPosition = bestPositions[i];
        auto currentEvaluation = bestEvaluations[i];
        auto stepSize = 0.01f;
        while(stepSize >= 1e-4f && currentEvaluation > 0.0f) {
            bool improved = false;
            for (auto step : {glm::vec3(stepSize, 0, 0), glm::vec3(-stepSize, 0, 0),
                     glm::vec3(0, stepSize, 0), glm::vec3(0, -stepSize, 0),
                     glm::vec3(0, 0, stepSize), glm::vec3(0, 0, -stepSize)}) {

                step *= glm::length(solution->getProblem()->getContainer().getHalf());

                auto newPosition = validTranslationRange.getClosestPoint(currentPosition + step);
                assert(validTranslationRange.containsPoint(newPosition));

                Transformation newTransformation = item->getModelTransformation();
                newTransformation.setPosition(newPosition);
                solution->setItemTransformation(itemIndex, newTransformation);
                auto newEvaluation = evaluate_item_sample(solution, itemIndex, collisionWeights);
                if (newEvaluation < currentEvaluation) {
                    currentPosition = newPosition;
                    currentEvaluation = newEvaluation;

                    if (currentEvaluation <= 0.0f) {
                        // If we found a position with negative evaluation, we can stop
                        Transformation finalTransformation = item->getModelTransformation();
                        finalTransformation.setPosition(newPosition);
                        solution->setItemTransformation(itemIndex, finalTransformation);
                        return;
                    }

                    improved = true;
                }

            }
            if (!improved) {
                stepSize *= 0.5f; // Reduce step size
            }
        }
        sampledPositions[currentEvaluation] = currentPosition;
    }

    // Apply the best sample
    auto bestSample = sampledPositions.begin();
    assert(bestSample->first <= originalEvaluation);
    Transformation newTransformation = item->getModelTransformation();
    newTransformation.setPosition(bestSample->second);
    solution->setItemTransformation(itemIndex, newTransformation);
}

FixedRotationStripPackingTask::CollisionMetrics::CollisionMetrics() = default;

FixedRotationStripPackingTask::CollisionMetrics::CollisionMetrics(
    const std::shared_ptr<EnhancedStripPackingSolution> &solution) {
    collidingItems.resize(solution->getNumberOfItems(), true);
}

void FixedRotationStripPackingTask::update_collision_metrics(
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

            const bool isColliding = collide(solution, i,j);
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

void FixedRotationStripPackingTask::move_colliding_items(std::shared_ptr<EnhancedStripPackingSolution> &solution,
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
    }
}

FixedRotationStripPackingTask::CollisionMetrics FixedRotationStripPackingTask::move_colliding_items_multi(
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

void FixedRotationStripPackingTask::iterate_weights(std::map<std::pair<size_t, size_t>, float> &collisionWeights,
    const EnhancedStripPackingSolution &solution, std::map<std::pair<size_t, size_t>, float> &collisionQuantities,
    const std::map<std::pair<size_t, size_t>, bool> &collidingItemPairs, const float e_max) {

    // Update collision weights based on the current collisions
    for (size_t i = 0; i < solution.getNumberOfItems(); ++i) {
        for (size_t j = i + 1; j < solution.getNumberOfItems(); ++j) {

            float factor;
            assert(collidingItemPairs.find({i, j}) != collidingItemPairs.end());
            if (collidingItemPairs.find({i, j})->second) {
                auto quantity = collisionQuantities[{i, j}];
                factor = 1.05f + (1.5f - 1.05f) * quantity/e_max; // a factor in between 1.05 and 1.2
                assert(factor <= 2.0f && factor >= 1.2f);
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

std::map<std::pair<size_t, size_t>, float> FixedRotationStripPackingTask::init_weights(
    const EnhancedStripPackingSolution &solution) {
    std::map<std::pair<size_t, size_t>, float> collisionWeights;
    for (size_t i = 0; i < solution.getNumberOfItems(); ++i) {
        for (size_t j = i + 1; j < solution.getNumberOfItems(); ++j) {
            collisionWeights[{i, j}] = 1.0f; // Initial weight
        }
    }
    return collisionWeights;
}

FixedRotationStripPackingTask::FixedRotationStripPackingTask(const std::shared_ptr<StripPackingProblem> &problem): problem(problem) {}

FixedRotationStripPackingTask::FixedRotationStripPackingTask(const std::shared_ptr<StripPackingProblem> &problem,
    int seed): problem(problem), seed(seed) {}

bool FixedRotationStripPackingTask::separate(std::shared_ptr<EnhancedStripPackingSolution> &solution,
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

                        for (int itemIndex = 0; itemIndex < solution->getNumberOfItems(); ++itemIndex) {
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

std::shared_ptr<EnhancedStripPackingSolution> FixedRotationStripPackingTask::explore(
    std::shared_ptr<EnhancedStripPackingSolution> &solution, float initialHeight, float minimumHeight) {

    Random random(seed);
    auto bestSolution = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone());
    startMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    auto currentHeight = initialHeight;

    std::vector<std::shared_ptr<EnhancedStripPackingSolution>> solutionsPool;

    while (elapsedMilliseconds < allowedRunTimeMilliseconds && !stopCalled) {
        bool separated = separate(solution, 3, 200, currentHeight, random);
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
            for (auto itemIndex = 0; itemIndex < solution->getNumberOfItems(); ++itemIndex) {
                auto& item = solution->getItem(itemIndex);
                auto transformation = item->getModelTransformation();
                auto validTranslationRange = getValidTranslationRange(solution, currentHeight, itemIndex);
                transformation.setPositionZ(glm::clamp(transformation.getPosition().z * REDUCTION_FACTOR, validTranslationRange.getMinimum().z, validTranslationRange.getMaximum().z));
                solution->setItemTransformation(itemIndex, transformation);
                assert(problem->getContainer().containsAABB(solution->getItemAABB(itemIndex))); // Ensure items are not below the ground
            }

            solutionsPool.clear();
        }
        else {
            std::cout << "Disrupting solution." << std::endl;
            solutionsPool.push_back(std::dynamic_pointer_cast<EnhancedStripPackingSolution>(solution->clone()));
            auto solutionToDisrupt = solutionsPool[random.nextInteger(0, solutionsPool.size() - 1)];

            auto itemIndexA = random.nextInteger(0, solutionToDisrupt->getNumberOfItems() - 1);
            auto itemIndexB = random.nextInteger(0, solutionToDisrupt->getNumberOfItems() - 2);
            if (itemIndexB == itemIndexA) itemIndexB++;

            auto newPosA = solutionToDisrupt->getItem(itemIndexB)->getModelTransformation().getPosition();
            auto newPosB = solutionToDisrupt->getItem(itemIndexA)->getModelTransformation().getPosition();
            newPosA = getValidTranslationRange(solution, currentHeight, itemIndexA).getClosestPoint(newPosA);
            newPosB = getValidTranslationRange(solution, currentHeight, itemIndexB).getClosestPoint(newPosB);

            auto newTransformationA = solutionToDisrupt->getItem(itemIndexA)->getModelTransformation();
            newTransformationA.setPosition(newPosA);
            solutionToDisrupt->setItemTransformation(itemIndexA, newTransformationA);

            auto newTransformationB = solutionToDisrupt->getItem(itemIndexB)->getModelTransformation();
            newTransformationB.setPosition(newPosB);
            solutionToDisrupt->setItemTransformation(itemIndexB, newTransformationB);

            solution = solutionToDisrupt;
        }

        elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startMilliseconds;
    }

    return bestSolution;
}

void FixedRotationStripPackingTask::run() {

    this->notifyObserversStatus("Initialising");

    // Create and notify the solution
    notifyObserversStatus("Constructing solution");
    auto startms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    auto solution = std::make_shared<EnhancedStripPackingSolution>(problem);
    auto endms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout << "Creating solution took " << (endms-startms) << "ms" << std::endl;

    // Create a trivial solution
    auto trivialHeight = 0.0f; // Height of the trivial solution
    auto minimumHeight = 0.0f; // Minimum height of a solution determined by the highest item
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
            auto itemHeight = itemBounds.getMaximum().z - itemBounds.getMinimum().z;
            currentHeight += itemHeight;
            minimumHeight = std::max(minimumHeight, itemHeight);

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
    for (int itemIndex = 0; itemIndex < solution->getNumberOfItems(); ++itemIndex) {
        auto& item = solution->getItem(itemIndex);
        AABB validTranslationRange = getValidTranslationRange(solution, currentHeight, itemIndex);
        glm::vec3 randomPosition = sample_position(validTranslationRange, random);
        auto newTransformation = item->getModelTransformation();
        newTransformation.setPosition(randomPosition);
        solution->setItemTransformation(itemIndex, newTransformation);
    }
    notifyObserversSolution(solution);


    notifyObserversStatus("Starting exploration");
    bestSolution = explore(solution, currentHeight, minimumHeight);

    notifyObserversSolution(bestSolution);

    result = bestSolution;

    // Export the best solution
    auto solutionJSON = std::dynamic_pointer_cast<EnhancedStripPackingSolution>(bestSolution)->toJson();
    std::ofstream jsonOutputFile(SOLUTION_DIR + problem->getName() + "_GLS_" + std::to_string(bestSolution->computeTotalHeight()) + ".json");
    jsonOutputFile << solutionJSON.dump(4);
    jsonOutputFile.close();
}

void FixedRotationStripPackingTask::setSeed(int seed) {
    this->seed = seed;
}

void FixedRotationStripPackingTask::setAllowedRunTimeMilliseconds(size_t milliseconds) {
    this->allowedRunTimeMilliseconds = milliseconds;
}

std::shared_ptr<StripPackingSolution> FixedRotationStripPackingTask::getResult() const {
    return result;
}

