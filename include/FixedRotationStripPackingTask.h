//
// Created by Jonas Tollenaere on 05/07/2025.
//

#ifndef FIXEDROTATIONSTRIPPACKINGTASK_H
#define FIXEDROTATIONSTRIPPACKINGTASK_H

#include <meshcore/tasks/AbstractTask.h>
#include "EnhancedStripPackingSolution.h"
#include <meshcore/utility/Random.h>

class FixedRotationStripPackingTask final : public AbstractTask {

    bool collide(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    float overlap_proxy(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    float overlap_proxy_decay(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    // Should return a metric that quantifies how bas the collision is, i.e., how much the items overlap.
    float quantify_collision(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    AABB getValidTranslationRange(const std::shared_ptr<EnhancedStripPackingSolution>& solution, float containerHeight, size_t itemIndex);

    std::vector<bool> getCollidingItems(const std::shared_ptr<EnhancedStripPackingSolution>& solution);

    float evaluate_item_sample(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndex, const std::map<std::pair<size_t, size_t>, float> & collisionWeights);

    glm::vec3 sample_position(const AABB& range, const Random& random);

    void search_position(std::shared_ptr<EnhancedStripPackingSolution>& solution, float containerHeight, size_t itemIndex, const Random& random, const std::map<std::pair<size_t, size_t>, float> & collisionWeights);

    struct CollisionMetrics {
        bool collision = true;
        float worstCollisionQuantity = 0.0f;
        float totalCollisionQuantity = 0.0f;
        std::map<std::pair<size_t,size_t>, bool> collidingItemPairs;
        std::map<std::pair<size_t,size_t>, float> collisionQuantities;
        std::vector<bool> collidingItems;

        CollisionMetrics();

        explicit CollisionMetrics(const std::shared_ptr<EnhancedStripPackingSolution>& solution);
    };

    void update_collision_metrics(const std::shared_ptr<EnhancedStripPackingSolution>& solution, CollisionMetrics& metrics);

    void move_colliding_items(std::shared_ptr<EnhancedStripPackingSolution>& solution, float currentHeight, CollisionMetrics metrics, const std::map<std::pair<size_t, size_t>, float> & collisionWeights, const Random& random);

    CollisionMetrics move_colliding_items_multi(std::shared_ptr<EnhancedStripPackingSolution>& solution, float currentHeight, const CollisionMetrics& metrics, const std::map<std::pair<size_t, size_t>, float> & collisionWeights, const Random& random);

    void iterate_weights(std::map<std::pair<size_t, size_t>, float>& collisionWeights, const EnhancedStripPackingSolution& solution,std::map<std::pair<size_t,size_t>, float>& collisionQuantities, const std::map<std::pair<size_t,size_t>, bool>& collidingItemPairs, const float e_max);

    std::map<std::pair<size_t, size_t>, float> init_weights(const EnhancedStripPackingSolution& solution);

    const std::shared_ptr<StripPackingProblem> problem;

private:
    int seed = 0;

    float targetHeight = 0.0f;

    size_t allowedRunTimeMilliseconds = 60 * 60 * 1000; // 1 hour
    size_t TargetReachedMilliseconds = 0;
    std::shared_ptr<StripPackingSolution> result;

    long long startMilliseconds = 0;
    long long elapsedMilliseconds = 0;

public:
    explicit FixedRotationStripPackingTask(const std::shared_ptr<StripPackingProblem>& problem);

    FixedRotationStripPackingTask(const std::shared_ptr<StripPackingProblem>& problem, int seed);

    bool separate(std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t maxAttempts, size_t maxIterationsWithoutImprovement, float currentHeight, Random& random);

    std::shared_ptr<EnhancedStripPackingSolution> explore(std::shared_ptr<EnhancedStripPackingSolution>& solution, float initialHeight, float minimumHeight);

    void run() override;

    void setSeed(int seed);

    void setAllowedRunTimeMilliseconds(size_t milliseconds);

    std::shared_ptr<StripPackingSolution> getResult() const;
};

#endif //FIXEDROTATIONSTRIPPACKINGTASK_H
