//
// Created by Jonas Tollenaere on 04/07/2025.
//

#ifndef BENCHMARKOBSERVER_H
#define BENCHMARKOBSERVER_H

class StripPackingBenchmarkObserver: public AbstractTaskObserver {

    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point stopTime;

    float targetHeight = 0.0f; // Target height for the benchmark, used to determine if the solution is feasible
    long long targetHeightMilliseconds = std::numeric_limits<long long>::max(); // Target height in milliseconds, used to determine if the solution is feasible

    float bestHeight05Minutes = std::numeric_limits<float>::max();
    float bestHeight10Minutes = std::numeric_limits<float>::max();
    float bestHeight20Minutes = std::numeric_limits<float>::max();
    float bestHeight30Minutes = std::numeric_limits<float>::max();
    float bestHeight60Minutes = std::numeric_limits<float>::max();
    float bestHeight = std::numeric_limits<float>::max();

public:

    explicit StripPackingBenchmarkObserver(const float targetHeight = 0.0f) : targetHeight(targetHeight) {}

    virtual ~StripPackingBenchmarkObserver() = default;

    void notifyStarted() override {
        startTime = std::chrono::high_resolution_clock::now();

        // Reset the best solutions in case the task was started multiple times
        bestHeight05Minutes = std::numeric_limits<float>::max();
        bestHeight10Minutes = std::numeric_limits<float>::max();
        bestHeight30Minutes = std::numeric_limits<float>::max();
        bestHeight20Minutes = std::numeric_limits<float>::max();
        bestHeight60Minutes = std::numeric_limits<float>::max();
        bestHeight = std::numeric_limits<float>::max();

    }

    void notifyFinished() override {
        stopTime = std::chrono::high_resolution_clock::now();

        std::cout << "Benchmark finished." << std::endl;
        auto elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime).count();
        std::cout << "Best height within 5 minutes: " << bestHeight05Minutes << std::endl;
        std::cout << "Best height within 10 minutes: " << bestHeight10Minutes << std::endl;
        std::cout << "Best height within 20 minutes: " << bestHeight20Minutes << std::endl;
        std::cout << "Best height within 30 minutes: " << bestHeight30Minutes << std::endl;
        std::cout << "Best height within 60 minutes: " << bestHeight60Minutes << std::endl;
        std::cout << "Best height overall: " << bestHeight << " in " << elapsedMilliseconds << " ms" << std::endl;
    }

    void notifyProgress(float progress) override {

    }

    void notifyStatus(const std::string &status) override {

    }

    [[nodiscard]] float getBestHeight() const {
        return bestHeight;
    }

    [[nodiscard]] float getBestHeight05Minutes() const {
        return bestHeight05Minutes;
    }

    [[nodiscard]] float getBestHeight10Minutes() const {
        return bestHeight10Minutes;
    }

    [[nodiscard]] float getBestHeight20Minutes() const {
        return bestHeight20Minutes;
    }

    [[nodiscard]] float getBestHeight30Minutes() const {
        return bestHeight30Minutes;
    }

    [[nodiscard]] float getBestHeight60Minutes() const {
        return bestHeight60Minutes;
    }

    [[nodiscard]] long long getTargetHeightMilliseconds() const {
        return targetHeightMilliseconds;
    }

    [[nodiscard]] float getTargetHeight() const {
        return targetHeight;
    }

    void notifySolution(const std::shared_ptr<const AbstractSolution>& sol) override{

        const auto& solution = std::dynamic_pointer_cast<const StripPackingSolution>(sol);
        
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        auto solutionHeight = solution->computeTotalHeight();

        if (solutionHeight < bestHeight) {
            bestHeight = solution->computeTotalHeight();
        }

        if (solutionHeight < targetHeight && elapsedMilliseconds < targetHeightMilliseconds) {
            targetHeightMilliseconds = elapsedMilliseconds;
            std::cout << "Found a solution with target height " << targetHeight << " in " << targetHeightMilliseconds << " ms." << std::endl;
        }

        // Within 5 minutes
        if (elapsedMilliseconds <= 5 * 60 * 1000) {
            if (solution->computeTotalHeight() < bestHeight05Minutes) {
                bestHeight05Minutes = solution->computeTotalHeight();
            }
        }
        
        // Within 10 minutes
        if (elapsedMilliseconds <= 10 * 60 * 1000) {
            if (solution->computeTotalHeight() < bestHeight10Minutes) {
                bestHeight10Minutes = solution->computeTotalHeight();
            }
        }

        // Within 20 minutes
        if (elapsedMilliseconds <= 20 * 60 * 1000) {
            if (solution->computeTotalHeight() < bestHeight20Minutes) {
                bestHeight20Minutes = solution->computeTotalHeight();
            }
        }
        
        // Within 30 minutes
        if (elapsedMilliseconds <= 30 * 60 * 1000) {
            if (solution->computeTotalHeight() < bestHeight30Minutes) {
                bestHeight30Minutes = solution->computeTotalHeight();
            }
        }
        
        // Within 60 minutes
        if (elapsedMilliseconds <= 60 * 60 * 1000) {
            if (solution->computeTotalHeight() < bestHeight60Minutes) {
                bestHeight60Minutes = solution->computeTotalHeight();
            }
        }
        
    }
};

#endif //BENCHMARKOBSERVER_H
