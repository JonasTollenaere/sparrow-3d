#include <iostream>
#include <meshcore/utility/FileParser.h>
#include "meshcore/rendering/ApplicationWindow.h"
#include <QApplication>
#include <QMenu>

#include "AlphaWrapper.h"
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

    std::cout << "Hello, World!" << std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    //auto mesh = FileParser::loadMeshFile(MESHCORE_DATA_DIR + std::string("Liu et al. 2015/ring.obj"));
    //auto mesh = FileParser::loadMeshFile(MESHCORE_DATA_DIR + std::string("Stoyan et al. 2004/nonconvex/sanitized/polytope1.obj"));
    auto mesh = FileParser::loadMeshFile(MESHCORE_DATA_DIR + std::string("Lamas-Fernandez, C. et al. 2022/Engine/header.obj"));
    //auto mesh = FileParser::loadMeshFile(MESHCORE_DATA_DIR + std::string("Lamas-Fernandez, C. et al. 2022/Chess/classic_bishop.stl"));
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Loading took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    // Shift mesh to its center
    {
        WorldSpaceMesh temp(mesh);
        temp.getModelTransformation().deltaPosition(-mesh->getBounds().getCenter());
        mesh = temp.getTransformedModelSpaceMesh();
    }
    renderWidget->renderWorldSpaceMesh("Original", std::make_shared<WorldSpaceMesh>(mesh), Color::Locust());

    start = std::chrono::high_resolution_clock::now();
    const auto wrappedMesh = AlphaWrapper::getAlphaWrapping(mesh);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Wrapping took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    if (wrappedMesh->getTriangles().size() < mesh->getTriangles().size()) mesh = wrappedMesh;
    renderWidget->renderWorldSpaceMesh("Wrapped", std::make_shared<WorldSpaceMesh>(mesh), Color::White(0.5));

    start = std::chrono::high_resolution_clock::now();
    auto poles = InaccessibilityPoles::computePolesOfInaccessibility(mesh, 25);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Computing poles of inaccessibility took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    for (auto i = 0; i<poles.size(); i++){
        auto& pole = poles[i];
        auto color = Color::Red();
        renderWidget->renderSphere("Poles", "Pole " + std::to_string(i), pole, color);
    }
}
