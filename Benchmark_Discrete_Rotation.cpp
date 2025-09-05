//
// Created by Jonas Tollenaere on 05/07/2025.
//

#include "DiscreteRotationStripPackingTask.h"

#include <fstream>
#include <fmt/core.h>

#include <meshcore/rendering/ApplicationWindow.h>
#include <meshcore/optimization/StripPackingProblem.h>
#include <../datasets/StripPackingInstances.h>

#include "BenchmarkObserver.h"
#include "InaccessibilityPoles.h"

void run(RenderWidget* renderWidget) {

    std::vector<std::tuple<std::string, float, size_t>> instances;

    // Liu instances
    instances.emplace_back(LIU_2015_EXAMPLE_1, 34.13, 8);

    // Stoyan 2004 instances with the rotation angles benchmarked in Liu et al. 2015
    instances.emplace_back(STOYAN_2004_EXAMPLE_5, 73.6, 4);
    instances.emplace_back(STOYAN_2004_EXAMPLE_4, 59, 4);
    instances.emplace_back(STOYAN_2004_EXAMPLE_3, 46, 4);
    instances.emplace_back(STOYAN_2004_EXAMPLE_2, 31, 4);

    instances.emplace_back(STOYAN_2004_EXAMPLE_5, 80.1, 2);
    instances.emplace_back(STOYAN_2004_EXAMPLE_4, 65, 2);
    instances.emplace_back(STOYAN_2004_EXAMPLE_3, 48, 2);
    instances.emplace_back(STOYAN_2004_EXAMPLE_2, 34, 2);

    // Egeblad 2009
    instances.emplace_back(EGEBLAD_2009_MERGED_1,16,4);
    instances.emplace_back(EGEBLAD_2009_MERGED_2,19.81,4);
    instances.emplace_back(EGEBLAD_2009_MERGED_3,25.19,4);
    instances.emplace_back(EGEBLAD_2009_MERGED_4,30.41,4);
    instances.emplace_back(EGEBLAD_2009_MERGED_5,38.31,4);

    // Stoyan 2005 instances, benchmarked by Egeblad et al. 2009
    instances.emplace_back(STOYAN_2005_EXAMPLE_1, 16,4);
    instances.emplace_back(STOYAN_2005_EXAMPLE_2, 16,4);
    instances.emplace_back(STOYAN_2005_EXAMPLE_3, 24.91,4);

    instances.emplace_back(LAMAS_FERNANDEZ_2022_ENGINE, 0.0f, 4);
    instances.emplace_back(LAMAS_FERNANDEZ_2022_CHESS, 0.0f, 4);

    for (const auto& [instance, bestReportedHeight, nRotationAngles] : instances) {
        for (int seed = 0; seed < 15; ++seed) {
            renderWidget->clear();

            SparrowOptions options;
            options.seed = seed;

            DiscreteRotationStripPackingTask task(StripPackingProblem::fromInstancePath(instance, ObjectOrigin::AlignToMinimum), nRotationAngles, options);

            StripPackingBenchmarkObserver observer(bestReportedHeight);
            task.registerObserver(&observer);
            renderWidget->observeTask(&task);

            task.start();
            task.join();

            // Export the best solution
            auto result = task.getResult();
            auto solutionJSON = result->toJson();
            std::ofstream jsonOutputFile(std::string(SOLUTION_DIR) + "discrete/" + result->getProblem()->getName() + "_Discrete_RN" + std::to_string(nRotationAngles) + "_" + std::to_string(seed) + "_" + std::to_string(result->computeTotalHeight()) + ".json");
            jsonOutputFile << solutionJSON.dump(4);
            jsonOutputFile.close();

            // Check if CSV file exists before opening
            std::string csvFilePath = std::string(SOLUTION_DIR) + "discrete/benchmark_results_Discrete.csv";
            bool csvFileExists = std::ifstream(csvFilePath).good();
            std::ofstream csvFile(csvFilePath, std::ios::app);
            if (!csvFileExists) {
                csvFile << "Instance;nRotations;Seed;Best height 05 minutes;"
                        << "Best height 10 minutes;Best height 20 minutes;"
                        << "Best height 30 minutes;Best height 60 minutes;"
                        << "Best height;Target height; Target height (ms)\n";
            }

            auto format_number = [](double value) {
                std::string s = fmt::format("{:.2f}", value);
                std::replace(s.begin(), s.end(), '.', ',');
                return s;
            };
            csvFile << fmt::format("{};{};{};{};{};{};{};{};{};{};{}\n",
                instance,
                nRotationAngles,
                seed,
                format_number(observer.getBestHeight05Minutes()),
                format_number(observer.getBestHeight10Minutes()),
                format_number(observer.getBestHeight20Minutes()),
                format_number(observer.getBestHeight30Minutes()),
                format_number(observer.getBestHeight60Minutes()),
                format_number(observer.getBestHeight()),
                format_number(observer.getTargetHeight()),
                observer.getTargetHeightMilliseconds()
            );
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