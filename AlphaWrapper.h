//
// Created by Jonas on 3/03/2023.
//

#ifndef EXTENDEDMESHCORE_ALPHAWRAPPER_H
#define EXTENDEDMESHCORE_ALPHAWRAPPER_H

#include <mutex>
#include <unordered_map>
#include "meshcore/core/ModelSpaceMesh.h"

class AlphaWrapper {

    static std::mutex cacheMapMutex;
    static std::unordered_map<std::shared_ptr<ModelSpaceMesh>, std::shared_ptr<ModelSpaceMesh>> cacheMap;

    static std::shared_ptr<ModelSpaceMesh> createAlphaWrapping(const std::shared_ptr<ModelSpaceMesh>& input, double relative_alpha=1/20.0, double relative_offset = 1/600.0);

public:
    static std::shared_ptr<ModelSpaceMesh> getAlphaWrapping(const std::shared_ptr<ModelSpaceMesh>& input);
};
#endif //EXTENDEDMESHCORE_ALPHAWRAPPER_H
