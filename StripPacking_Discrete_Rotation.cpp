//
// Created by Jonas Tollenaere on 02/06/2025.
//

#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/optimization/StripPackingProblem.h>

#include "../datasets/StripPackingInstances.h"

#include "DiscreteRotationStripPackingTask.h"

int main(int argc, char *argv[]){

    //DiscreteRotationStripPackingTask task(StripPackingProblem::fromInstancePath(LIU_2015_EXAMPLE_1 , ObjectOrigin::AlignToMinimum), 8);
    DiscreteRotationStripPackingTask task(StripPackingProblem::fromInstancePath(STOYAN_2004_EXAMPLE_2 , ObjectOrigin::AlignToMinimum), 8);

    QApplication app(argc, argv);
    ApplicationWindow window;
    window.show();
    RenderWidget* renderWidget = window.getRenderWidget();
    renderWidget->observeTask(&task);
    task.start();
    int returnCode = QApplication::exec();
    task.stop();
    task.join();

    return returnCode;
}