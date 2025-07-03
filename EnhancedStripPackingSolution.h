//
// Created by Jonas Tollenaere on 02/07/2025.
//

#ifndef ENHANCEDSTRIPPACKINGSOLUTION_H
#define ENHANCEDSTRIPPACKINGSOLUTION_H

#include <meshcore/optimization/StripPackingSolution.h>

#include "AlphaWrapper.h"
#include "InaccessibilityPoles.h"

class EnhancedStripPackingSolution: public StripPackingSolution {
    std::vector<std::shared_ptr<WorldSpaceMesh>> simplifiedItems;
    std::vector<std::vector<Sphere>> itemPolesOfInaccessibility;

public:
    explicit EnhancedStripPackingSolution(StripPackingSolution&& solution): StripPackingSolution(solution) {
        simplifiedItems.reserve(solution.getItems().size());
        itemPolesOfInaccessibility.reserve(solution.getItems().size());
        for (const auto & item : solution.getItems()) {

            auto modelSpaceMesh = item->getModelSpaceMesh();
            auto simplifiedMesh = modelSpaceMesh;
            if(modelSpaceMesh->getTriangles().size() > 1000){
                auto start = std::chrono::high_resolution_clock::now();
                auto wrappedMesh = AlphaWrapper::getAlphaWrapping(modelSpaceMesh);
                auto end = std::chrono::high_resolution_clock::now();
                if(wrappedMesh->getTriangles().size() < modelSpaceMesh->getTriangles().size()){
                    simplifiedMesh = wrappedMesh;
                    std::cout << "Alpha wrapping took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms - Reduced size from "
                              << modelSpaceMesh->getTriangles().size() << " triangles to " << wrappedMesh->getTriangles().size() << std::endl;
                }
                else {
                    std::cout << "Alpha wrapping took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms - Did not reduce size" << std::endl;
                }
            }

            auto worldSpaceMesh = std::make_shared<WorldSpaceMesh>(simplifiedMesh);
            worldSpaceMesh->setModelTransformation(item->getModelTransformation());
            simplifiedItems.push_back(worldSpaceMesh);

            itemPolesOfInaccessibility.push_back(InaccessibilityPoles::computePolesOfInaccessibility(simplifiedMesh, 32));
        }
    }

    EnhancedStripPackingSolution(const EnhancedStripPackingSolution& other): StripPackingSolution(other),
    itemPolesOfInaccessibility(other.itemPolesOfInaccessibility) {
        for (size_t i = 0; i < other.simplifiedItems.size(); ++i) {
            simplifiedItems.push_back(other.simplifiedItems[i]->clone());

        }
    }

    size_t getNumberOfItems() const {
        return getItems().size();
    }

    explicit EnhancedStripPackingSolution(const std::shared_ptr<StripPackingProblem>& problem): EnhancedStripPackingSolution(StripPackingSolution(problem)) {}

    [[nodiscard]] const std::vector<std::shared_ptr<WorldSpaceMesh>>& getSimplifiedItems() const {
        return simplifiedItems;
    }

    [[nodiscard]] const std::shared_ptr<WorldSpaceMesh>& getSimplifiedItem(size_t itemIndex) const {
        return simplifiedItems[itemIndex];
    }

    [[nodiscard]] const std::vector<Sphere>& getPolesOfInaccessibility(size_t itemIndex) const {
        return itemPolesOfInaccessibility[itemIndex];
    }

    void setItemTransformation(size_t itemIndex, const Transformation& transformation) override {
        StripPackingSolution::setItemTransformation(itemIndex, transformation);

        // Update the simplified item as well
        simplifiedItems[itemIndex]->setModelTransformation(transformation);
    }

    std::shared_ptr<AbstractSolution> clone() const override {
        return std::make_shared<EnhancedStripPackingSolution>(*this);
    }
};



#endif //ENHANCEDSTRIPPACKINGSOLUTION_H
