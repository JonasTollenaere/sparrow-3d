//
// Created by Jonas Tollenaere on 16/07/2025.
//

#ifndef DISCRETEROTATIONSTRIPPACKINGTASK_H
#define DISCRETEROTATIONSTRIPPACKINGTASK_H

#include "EnhancedStripPackingSolution.h"
#include "SparrowOptions.h"

#include <meshcore/tasks/AbstractTask.h>
#include <meshcore/utility/Random.h>

class DiscreteRotationStripPackingTask final : public AbstractTask {

    /**
     * Checks if the item with the given index is contained in the container.
     * @param solution The solution to be checked.
     * @param itemIndex The index of the item to check.
     * @return true if the item is contained in the container, false otherwise.
     */
    bool is_contained(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndex);

    /**
     * Checks if two items in a solution collide.
     * @param solution The solution containing the items.
     * @param itemIndexA The index of the first item.
     * @param itemIndexB The index of the second item.
     * @return true if the items collide, false otherwise.
     */
    bool collide(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Computes an approximation of the overlap between two items in a solution. The items are assumed to collide.
     * @param solution The solution containing the items.
     * @param itemIndexA The index of the first item.
     * @param itemIndexB The index of the second item.
     * @return
     */
    float overlap_proxy(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Computes a decay version of the overlap approximation between two items in a solution. Even if the poles of inaccessibility do not overlap, their pairwise distance is still considered.
     * @param solution The solution containing the items.
     * @param itemIndexA The index of the first item.
     * @param itemIndexB The index of the second item.
     * @return
     */
    float overlap_proxy_decay(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Should return a metric that quantifies how bad the collision is, i.e., how much the items overlap.
     * @param solution
     * @param itemIndexA
     * @param itemIndexB
     * @return A float value representing the quantity of the collision.
     */
    float quantify_collision(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndexA, size_t itemIndexB);

    /**
     * Returns a valid translation range for the item with the given index in the solution, considering its current rotation.
     * @param solution The solution containing the item.
     * @param containerHeight The height of the container in which the item should be placed.
     * @param itemIndex
     * @return
     */
    std::optional<AABB> getValidTranslationRange(const std::shared_ptr<EnhancedStripPackingSolution>& solution, float containerHeight, size_t itemIndex);

    /**
     *
     * @param solution
     * @return
     */
    std::vector<bool> getCollidingItems(const std::shared_ptr<EnhancedStripPackingSolution>& solution);

    /**
     *
     * @param solution
     * @param itemIndex
     * @param collisionWeights
     * @return
     */
    float evaluate_item_sample(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndex, const std::map<std::pair<size_t, size_t>, float> & collisionWeights);

    float evaluate_item_sample_threshold(const std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t itemIndex, const std::map<std::pair<size_t, size_t>, float> & collisionWeights, float threshold);

    /**
     *
     * @param random
     * @param steps
     * @return
     */
    float sample_angle(const Random& random, size_t steps);

    /**
     *
     * @param random
     * @return
     */
    Quaternion sample_rotation(const Random& random);

    /**
     *
     * @param range
     * @param random
     * @return
     */
    glm::vec3 sample_position(const AABB& range, const Random& random);

    /**
     *
     * @param solution
     * @param containerHeight
     * @param itemIndex
     * @param random
     * @param collisionWeights
     */
    void search_position(std::shared_ptr<EnhancedStripPackingSolution>& solution, float containerHeight, size_t itemIndex, const Random& random, const std::map<std::pair<size_t, size_t>, float> & collisionWeights);

    struct CollisionMetrics {
        bool collision = true;
        float worstCollisionQuantity = 0.0f;
        float totalCollisionQuantity = 0.0f;
        std::map<std::pair<size_t,size_t>, bool> collidingItemPairs;
        std::map<std::pair<size_t,size_t>, float> collisionQuantities;
        std::vector<bool> collidingItems;

        CollisionMetrics() = default;

        explicit CollisionMetrics(const std::shared_ptr<EnhancedStripPackingSolution>& solution) {
            collidingItems.resize(solution->getNumberOfItems(), true);
        }
    };

    void update_collision_metrics(const std::shared_ptr<EnhancedStripPackingSolution>& solution, CollisionMetrics& metrics);

    void move_colliding_items(std::shared_ptr<EnhancedStripPackingSolution>& solution, float currentHeight, CollisionMetrics metrics, const std::map<std::pair<size_t, size_t>, float> & collisionWeights, const Random& random);

    CollisionMetrics move_colliding_items_multi(std::shared_ptr<EnhancedStripPackingSolution>& solution, float currentHeight, const CollisionMetrics& metrics, const std::map<std::pair<size_t, size_t>, float> & collisionWeights, const Random& random);

    void iterate_weights(std::map<std::pair<size_t, size_t>, float>& collisionWeights, const EnhancedStripPackingSolution& solution,std::map<std::pair<size_t,size_t>, float>& collisionQuantities, const std::map<std::pair<size_t,size_t>, bool>& collidingItemPairs, const float e_max);

    std::map<std::pair<size_t, size_t>, float> init_weights(const EnhancedStripPackingSolution& solution);

    SparrowOptions options;

    size_t nRotationAngles; // Number of discrete rotation angles around each axis

    long long startMilliseconds = 0;
    long long elapsedMilliseconds = 0;

    const std::shared_ptr<StripPackingProblem> problem;
    std::shared_ptr<StripPackingSolution> result;

public:
    explicit DiscreteRotationStripPackingTask(const std::shared_ptr<StripPackingProblem>& problem, size_t nRotationAngles, const SparrowOptions& options = SparrowOptions());

    bool separate(std::shared_ptr<EnhancedStripPackingSolution>& solution, size_t maxAttempts, size_t maxIterationsWithoutImprovement, float currentHeight, long long allowedRunTimeMilliseconds, const Random& random);

    std::shared_ptr<EnhancedStripPackingSolution> explore(std::shared_ptr<EnhancedStripPackingSolution>& solution, float initialHeight, float minimumHeight, const std::vector<Quaternion>& minimumHeightRotations, const Random& random);
    std::shared_ptr<EnhancedStripPackingSolution> compress(std::shared_ptr<EnhancedStripPackingSolution>& solution, float initialHeight, float minimumHeight, const std::vector<Quaternion>& minimumHeightRotations, const Random& random);

    void run() override;

    std::shared_ptr<StripPackingSolution> getResult() const;
};

#endif //DISCRETEROTATIONSTRIPPACKINGTASK_H
