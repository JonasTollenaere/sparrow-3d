//
// Created by Jonas Tollenaere on 02/07/2025.
//

#ifndef ENHANCEDSTRIPPACKINGSOLUTION_H
#define ENHANCEDSTRIPPACKINGSOLUTION_H

#include <meshcore/optimization/StripPackingSolution.h>

class EnhancedStripPackingSolution: public StripPackingSolution {
    std::vector<std::shared_ptr<WorldSpaceMesh>> simplifiedItems;
    std::vector<std::vector<Sphere>> itemPolesOfInaccessibility;

public:
    EnhancedStripPackingSolution(const EnhancedStripPackingSolution& other);
    explicit EnhancedStripPackingSolution(StripPackingSolution&& solution);
    explicit EnhancedStripPackingSolution(const std::shared_ptr<StripPackingProblem>& problem): EnhancedStripPackingSolution(StripPackingSolution(problem)) {}

    std::shared_ptr<AbstractSolution> clone() const override;

    [[nodiscard]] size_t getNumberOfItems() const;
    [[nodiscard]] const std::vector<std::shared_ptr<WorldSpaceMesh>>& getSimplifiedItems() const;
    [[nodiscard]] const std::shared_ptr<WorldSpaceMesh>& getSimplifiedItem(size_t itemIndex) const;
    [[nodiscard]] const std::vector<Sphere>& getPolesOfInaccessibility(size_t itemIndex) const;

    void setItemTransformation(size_t itemIndex, const Transformation& transformation) override;
};



#endif //ENHANCEDSTRIPPACKINGSOLUTION_H
