//
// Created by Jonas Tollenaere on 05/07/2025.
//

#include "FixedRotationStripPackingTask.h"

#include <fstream>

#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/optimization/StripPackingProblem.h>
#include <../datasets/StripPackingInstances.h>

#include "BenchmarkObserver.h"
#include "DiscreteRotationStripPackingTask.h"
#include "InaccessibilityPoles.h"

void run(RenderWidget* renderWidget) {

    std::vector<std::pair<std::string, float>> instances;

    // Liu with fixed rotation instances and best reported heights
    instances.emplace_back(LIU_2015_EXAMPLE_1, 34.13);

    for (int seed = 0; seed < 15; ++seed) {
        for (const auto& [instance, bestReportedHeight] : instances) {

            renderWidget->clear();

            DiscreteRotationStripPackingTask task(StripPackingProblem::fromInstancePath(instance, ObjectOrigin::AlignToMinimum));

            task.setSeed(seed);
            task.setAllowedRunTimeMilliseconds(10 * 60 * 1000);

            StripPackingBenchmarkObserver observer(bestReportedHeight);
            task.registerObserver(&observer);
            renderWidget->observeTask(&task);

            task.start();
            task.join();

            // Export the best solution
            auto result = task.getResult();
            auto solutionJSON = result->toJson();
            std::ofstream jsonOutputFile(std::string(SOLUTION_DIR) + "benchmark/" + result->getProblem()->getName() + "_Discrete_" + std::to_string(result->computeTotalHeight()) + ".json");
            jsonOutputFile << solutionJSON.dump(4);
            jsonOutputFile.close();

            // Check if CSV file exists before opening
            std::string csvFilePath = std::string(SOLUTION_DIR) + "benchmark/benchmark_results_discrete.csv";
            bool csvFileExists = std::ifstream(csvFilePath).good();
            std::ofstream csvFile(csvFilePath, std::ios::app);
            if (!csvFileExists) {
                csvFile << "Instance,Seed,Best height 05 minutes,"
                        << "Best height 10 minutes,Best height 20 minutes,"
                        << "Best height 30 minutes,Best height 60 minutes,"
                        << "Best height,Target height, Target height (ms)\n";
            }
            csvFile << instance << "," << seed << ","
                    << observer.getBestHeight05Minutes() << ","
                    << observer.getBestHeight10Minutes() << ","
                    << observer.getBestHeight20Minutes() << ","
                    << observer.getBestHeight30Minutes() << ","
                    << observer.getBestHeight60Minutes() << ","
                    << observer.getBestHeight() << ","
                    << observer.getTargetHeight() << ","
                    << observer.getTargetHeightMilliseconds() << "\n";

            csvFile.close();
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    ApplicationWindow window;
    window.show();

    std::thread thread(run, window.getRenderWidget());
    int returnCode = QApplication::exec();
    thread.join();
    return returnCode;

}