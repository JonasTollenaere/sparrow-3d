//
// Created by Jonas Tollenaere on 11/04/2025.
//

#ifndef EXTENDEDMESHCORE_INACCESSIBILITYPOLES_H
#define EXTENDEDMESHCORE_INACCESSIBILITYPOLES_H

#include "meshcore/core/Sphere.h"
#include "meshcore/core/ModelSpaceMesh.h"
#include "meshcore/acceleration/BoundingVolumeHierarchy.h"
#include <vector>



class InaccessibilityPoles{
public:
    static Sphere findNextPoleOfInaccessibility(const std::shared_ptr<ModelSpaceMesh> & mesh, const std::vector<Sphere> & poles, const std::shared_ptr<BoundingVolumeHierarchy> &tree);
    static std::vector<Sphere> computePolesOfInaccessibility(const std::shared_ptr<ModelSpaceMesh> &mesh, size_t numberOfPoles);
};

#endif //EXTENDEDMESHCORE_INACCESSIBILITYPOLES_H
