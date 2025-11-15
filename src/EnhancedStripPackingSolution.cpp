//
// Created by Jonas Tollenaere on 02/07/2025.
//

#include "EnhancedStripPackingSolution.h"

#include <iostream>

#include "InaccessibilityPoles.h"
#include "AlphaWrapper.h"

EnhancedStripPackingSolution::EnhancedStripPackingSolution(StripPackingSolution &&solution): StripPackingSolution(solution) {
    simplifiedItems.reserve(solution.getItems().size());
    itemPolesOfInaccessibility.reserve(solution.getItems().size());
    for (const auto & item : solution.getItems()) {

        auto modelSpaceMesh = item.getModelSpaceMesh();
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
        worldSpaceMesh->setModelTransformation(item.getModelTransformation());
        simplifiedItems.push_back(worldSpaceMesh);

        itemPolesOfInaccessibility.push_back(InaccessibilityPoles::computePolesOfInaccessibility(simplifiedMesh, 32));
    }
}

EnhancedStripPackingSolution::EnhancedStripPackingSolution(const EnhancedStripPackingSolution &other): StripPackingSolution(other),
    itemPolesOfInaccessibility(other.itemPolesOfInaccessibility) {
    for (const auto & simplifiedItem : other.simplifiedItems) {
        simplifiedItems.push_back(simplifiedItem->clone());

    }
}

std::shared_ptr<AbstractSolution> EnhancedStripPackingSolution::clone() const {
    return std::make_shared<EnhancedStripPackingSolution>(*this);
}

size_t EnhancedStripPackingSolution::getNumberOfItems() const {
    return getItems().size();
}

const std::vector<std::shared_ptr<WorldSpaceMesh>> & EnhancedStripPackingSolution::getSimplifiedItems() const {
    return simplifiedItems;
}

const std::shared_ptr<WorldSpaceMesh> & EnhancedStripPackingSolution::getSimplifiedItem(size_t itemIndex) const {
    return simplifiedItems[itemIndex];
}

const std::vector<Sphere> & EnhancedStripPackingSolution::getPolesOfInaccessibility(size_t itemIndex) const {
    return itemPolesOfInaccessibility[itemIndex];
}

void EnhancedStripPackingSolution::setItemTransformation(size_t itemIndex, const Transformation &transformation) {
    StripPackingSolution::setItemTransformation(itemIndex, transformation);

    // Update the simplified item as well
    simplifiedItems[itemIndex]->setModelTransformation(transformation);
}
