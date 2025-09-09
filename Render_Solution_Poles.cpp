//
// Created by Jonas on 9/09/2025.
//

#include <iostream>
#include <meshcore/utility/FileParser.h>
#include "meshcore/rendering/ApplicationWindow.h"
#include "meshcore/optimization/StripPackingSolution.h"
#include <QApplication>
#include <QMenu>

#include "AlphaWrapper.h"
#include "EnhancedStripPackingSolution.h"
#include "InaccessibilityPoles.h"

void run(RenderWidget* renderWidget);

int main(int argc, char *argv[]) {

    QApplication app(argc, argv);
    ApplicationWindow window;
    window.show();

    std::thread thread(run, window.getRenderWidget());
    int returnCode = QApplication::exec();
    thread.join();
    return returnCode;
}

void run(RenderWidget* renderWidget){

    std::string solutionPath = std::string(MESHCORE_DATA_DIR) + "Stoyan et al. 2005/Solution1.json";
    auto solution = StripPackingSolution::fromJson(solutionPath);

    renderWidget->setSolutionRenderCallback(
        [](RenderWidget* renderWidget, const std::shared_ptr<const AbstractSolution>& solution) {
            auto clone = std::dynamic_pointer_cast<StripPackingSolution>(solution->clone());
            auto sol = std::make_shared<EnhancedStripPackingSolution>(std::move(*clone));

            renderWidget->clearGroup("MinimalContainer");
                float maximumHeight = 0.0f;
                for (size_t itemIndex = 0; itemIndex < sol->getItems().size(); ++itemIndex) {
                    const auto& item = sol->getItem(itemIndex);
                    const auto& itemName = sol->getItemName(itemIndex);

                    // Update the maximum height
                    const auto& itemAABB = sol->getItemAABB(itemIndex);
                    maximumHeight = std::max(maximumHeight,itemAABB.getMaximum().z);

                    renderWidget->renderWorldSpaceMesh("Items", item, StripPackingProblem::getItemColor(itemName));
                    renderWidget->renderWorldSpaceMesh("Ghosts", item, Color(0.9,0.9,0.9,0.3));

                    auto convexHull = std::make_shared<WorldSpaceMesh>(item->getModelSpaceMesh()->getConvexHull());
                    convexHull->setModelTransformation(item->getModelTransformation());
                    renderWidget->renderWorldSpaceMesh("ConvexHulls", convexHull, Color(0.5, 0.5, 0.5, 0.5));

                    // Render the poles of inaccessibility for this item
                    for (size_t poleIndex = 0; poleIndex < sol->getPolesOfInaccessibility(itemIndex).size(); ++poleIndex) {
                        Sphere transformedSphere = sol->getPolesOfInaccessibility(itemIndex)[poleIndex].getTransformed(item->getModelTransformation());
                        renderWidget->renderSphere("Item"+std::to_string(itemIndex), "Pole"+std::to_string(poleIndex), transformedSphere,StripPackingProblem::getItemColor(itemName));
                    }
                }
                auto min = sol->getProblem()->getContainer().getMinimum();
                auto max = sol->getProblem()->getContainer().getMaximum();

                renderWidget->renderBox("MinimalContainer", "AABB", {min, {max.x, max.y, maximumHeight}});
        }
    );

    renderWidget->notifySolution(solution);
}
