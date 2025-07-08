//
// Created by Jonas Tollenaere on 02/06/2025.
//

#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/optimization/StripPackingProblem.h>

#include "../datasets/StripPackingInstances.h"

#include "FixedRotationStripPackingTask.h"

int main(int argc, char *argv[]){

    FixedRotationStripPackingTask task(StripPackingProblem::fromInstancePath(LAMAS_FERNANDEZ_2022_CHESS, ObjectOrigin::AlignToMinimum));

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