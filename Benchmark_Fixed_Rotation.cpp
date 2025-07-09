//
// Created by Jonas Tollenaere on 05/07/2025.
//

#include "FixedRotationStripPackingTask.h"

#include <fstream>

#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/optimization/StripPackingProblem.h>
#include <../datasets/StripPackingInstances.h>

#include "BenchmarkObserver.h"
#include "InaccessibilityPoles.h"

void run(RenderWidget* renderWidget) {

    std::vector<std::pair<std::string, float>> instances;

    // Stoyan 2005 instances and best reported heights
    instances.emplace_back(STOYAN_2005_EXAMPLE_1, 17.51);
    instances.emplace_back(STOYAN_2005_EXAMPLE_2, 19.01);
    instances.emplace_back(STOYAN_2005_EXAMPLE_3, 29.76);

    // Egeblad's Merged instances and best reported heights
    instances.emplace_back(EGEBLAD_2009_MERGED_1, 16.99);
    instances.emplace_back(EGEBLAD_2009_MERGED_2, 21.97);
    instances.emplace_back(EGEBLAD_2009_MERGED_3, 24.04);
    instances.emplace_back(EGEBLAD_2009_MERGED_4, 26.47); // Extrafine, 4 hours
    instances.emplace_back(EGEBLAD_2009_MERGED_5, 29.24); // Extrafine, 4 hours

    // Liu with fixed rotation instances and best reported heights
    instances.emplace_back(LIU_2015_EXAMPLE_1, 34.13);

    // Lamas-Fernandez with fixed rotation instances and best reported heights
    instances.emplace_back(LAMAS_FERNANDEZ_2022_CHESS, 131);
    //instances.emplace_back(LAMAS_FERNANDEZ_2022_ENGINE, 0.0); // TODO Not sure

    // Stoyan 2004 instances and best reported heights
    instances.emplace_back(STOYAN_2004_EXAMPLE_1, 56.01);
    instances.emplace_back(STOYAN_2004_EXAMPLE_2, 28.47);
    instances.emplace_back(STOYAN_2004_EXAMPLE_3, 41.30);
    instances.emplace_back(STOYAN_2004_EXAMPLE_4, 54.83);
    instances.emplace_back(STOYAN_2004_EXAMPLE_5, 72.33);

    for (int seed = 0; seed < 15; ++seed) {
        for (const auto& [instance, bestReportedHeight] : instances) {

            renderWidget->clear();

            FixedRotationStripPackingTask task(StripPackingProblem::fromInstancePath(instance, ObjectOrigin::AlignToMinimum));

            task.setSeed(seed);
            task.setAllowedRunTimeMilliseconds(60 * 60 * 1000);

            StripPackingBenchmarkObserver observer(bestReportedHeight);
            task.registerObserver(&observer);
            renderWidget->observeTask(&task);

            task.start();
            task.join();

            // Export the best solution
            auto result = task.getResult();
            auto solutionJSON = result->toJson();
            std::ofstream jsonOutputFile(std::string(SOLUTION_DIR) + "benchmark/" + result->getProblem()->getName() + "_GLS_" + std::to_string(result->computeTotalHeight()) + ".json");
            jsonOutputFile << solutionJSON.dump(4);
            jsonOutputFile.close();

            // Check if CSV file exists before opening
            std::string csvFilePath = std::string(SOLUTION_DIR) + "benchmark/benchmark_results.csv";
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