//
// Created by Jonas on 4/09/2025.
//

#ifndef SPARROW_3D_SPARROWOPTIONS_H
#define SPARROW_3D_SPARROWOPTIONS_H

struct SparrowOptions {

    // General
    int seed = 0;

    // Separation procedure
    float weightMultiplierUpper = 1.50f;
    float weightMultiplierLower = 1.05f;
    float weightMultiplierDecay = 0.95f;
    int nWorkers = 4;

    // Exploration phase
    float explorationReductionFactor = 0.995f;
    int explorationMaxSeparationIterations = 100;
    int exploreMaxSeparationAttempts = 3;
    long long exploreTimeMillis = 1000 * 60 * 8; // 8 minutes

    // Compression phase
    float compressionReductionFactorMax = 0.9995f;
    float compressionReductionFactorMin = 0.9999f;
    int compressionMaxSeparationIterations = 50;
    int compressionMaxSeparationAttempts = 5;
    long long compressionTimeMillis = 1000 * 60 * 2; // 2 minutes
};

#endif //SPARROW_3D_SPARROWOPTIONS_H