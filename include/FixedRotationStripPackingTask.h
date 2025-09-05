//
// Created by Jonas Tollenaere on 05/07/2025.
//

#ifndef FIXEDROTATIONSTRIPPACKINGTASK_H
#define FIXEDROTATIONSTRIPPACKINGTASK_H

#include "EnhancedStripPackingSolution.h"
#include "SparrowOptions.h"

#include <meshcore/tasks/AbstractTask.h>
#include <meshcore/utility/Random.h>

class FixedRotationStripPackingTask final : public AbstractTask {

    /**
     * Checks if two items in a solution collide.
     * @param solution The solution containing the items.
     * @param itemIndexA Index of the first item.
     * @param itemIndexB Index of the second item.
     * @return True if the items collide, false otherwise.
     */
    static bool collide(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Computes an approximate metric for the overlap between two items in a solution,
     * based on the overlap of their poles of inaccessibility.
     * @param solution The solution containing the items.
     * @param itemIndexA Index of the first item.
     * @param itemIndexB Index of the second item.
     * @return A float representing the value of the overlap metric.
     */
    static float overlap_proxy(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Computes an improved approximation metric for the overlap between two items in a solution,
     * which also produces non-zero values when their poles of inaccessibility do not intersect.
     * @param solution The solution containing the items.
     * @param itemIndexA Index of the first item.
     * @param itemIndexB Index of the second item.
     * @return A float representing the value of the improved overlap metric.
     */
    static float overlap_proxy_decay(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Returns a metric that quantifies how bas the collision is, i.e., how much the items overlap.
     * @param solution
     * @param itemIndexA
     * @param itemIndexB
     * @return
     */
    static float quantify_collision(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    static AABB getValidTranslationRange(const std::shared_ptr<EnhancedStripPackingSolution>& solution, float containerHeight, size_t itemIndex);

    static std::vector<bool> getCollidingItems(const std::shared_ptr<EnhancedStripPackingSolution>& solution);

    float evaluate_item_sample(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndex, const std::map<std::pair<size_t, size_t>, float> & collisionWeights);

    float evaluate_item_sample_threshold(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndex, const std::map<std::pair<size_t, size_t>, float> & collisionWeights, float threshold);

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

    SparrowOptions options;

    long long startMilliseconds = 0;
    long long elapsedMilliseconds = 0;

    const std::shared_ptr<StripPackingProblem> problem;
    std::shared_ptr<StripPackingSolution> result;

public:
    FixedRotationStripPackingTask(const std::shared_ptr<StripPackingProblem>& problem, const SparrowOptions& options = SparrowOptions());

    bool separate(std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t maxAttempts, size_t maxIterationsWithoutImprovement, float currentHeight, long long allowedRunTimeMilliseconds, const Random& random);

    std::shared_ptr<EnhancedStripPackingSolution> explore(std::shared_ptr<EnhancedStripPackingSolution>& solution, float initialHeight, float minimumHeight,const Random& random);
    std::shared_ptr<EnhancedStripPackingSolution> compress(std::shared_ptr<EnhancedStripPackingSolution>& solution, float initialHeight, float minimumHeight, const Random& random);

    void run() override;

    std::shared_ptr<StripPackingSolution> getResult() const;
};

#endif //FIXEDROTATIONSTRIPPACKINGTASK_H
