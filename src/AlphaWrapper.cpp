//
// Created by Jonas on 5/03/2023.
//

#include "AlphaWrapper.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Real_timer.h>

namespace AW3 = CGAL::Alpha_wraps_3;
namespace PMP = CGAL::Polygon_mesh_processing;
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Surface = CGAL::Surface_mesh<Point_3>;


std::mutex AlphaWrapper::cacheMapMutex{};
std::unordered_map<std::shared_ptr<ModelSpaceMesh>, std::shared_ptr<ModelSpaceMesh>> AlphaWrapper::cacheMap{};

std::shared_ptr<ModelSpaceMesh> AlphaWrapper::createAlphaWrapping(const std::shared_ptr<ModelSpaceMesh> &input, double relative_alpha, double relative_offset) {

    // 1. Convert this to a CGAL Surface_mesh
    std::vector<Kernel::Point_3> points;
    std::vector<std::vector<std::size_t> > faces;
    for (const auto &vertex : input->getVertices()){
        points.emplace_back(vertex.x, vertex.y, vertex.z);
    }
    for (const auto &triangle : input->getTriangles()){
        std::vector<std::size_t> triangle_indices;
        triangle_indices.emplace_back(triangle.vertexIndex0);
        triangle_indices.emplace_back(triangle.vertexIndex1);
        triangle_indices.emplace_back(triangle.vertexIndex2);
        faces.emplace_back(triangle_indices);
    }
    Surface mesh;
    CGAL::Polygon_mesh_processing::orient_polygon_soup(points, faces);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, faces, mesh);

    // 2. CGAL Alpha Wrapping
    CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(mesh);
    const double diag_length = std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
                                         CGAL::square(bbox.ymax() - bbox.ymin()) +
                                         CGAL::square(bbox.zmax() - bbox.zmin()));
    const double alpha = diag_length * relative_alpha;
    const double offset = diag_length * relative_offset;
    CGAL::Real_timer t;
    t.start();
    Surface wrap;
    CGAL::alpha_wrap_3(mesh, alpha, offset, wrap);
    t.stop();
#if !NDEBUG
    std::cout << "Original: " << num_vertices(mesh) << " vertices, " << num_faces(mesh) << " faces" << std::endl;
    std::cout << "Result: " << num_vertices(wrap) << " vertices, " << num_faces(wrap) << " faces" << std::endl;
    std::cout << "Took " << t.time() << " s." << std::endl;
#endif

    // 3. Convert the result back to a ModelSpaceMesh
    std::vector<Vertex> vertices;
    std::vector<IndexTriangle> triangles;
    for (const auto &vertex: wrap.vertices()){
        const auto &point = wrap.point(vertex);
        vertices.emplace_back(point.x(), point.y(), point.z());
    }
    for (Surface::Face_index face_index : wrap.faces()) {
        std::vector<unsigned int> indices;
        CGAL::Vertex_around_face_circulator<Surface> vcirc(wrap.halfedge(face_index), wrap), done(vcirc);
        do indices.push_back(*vcirc++); while (vcirc != done);
        assert(indices.size() == 3 && "Only triangles are supported."); // If this fails, we could use the mapbox triangulation
        triangles.emplace_back(indices[0], indices[1], indices[2]);
    }

    auto result = std::make_shared<ModelSpaceMesh>(vertices, triangles);
    result->setName("Wrapper of " + input->getName());
    return result;
}

std::shared_ptr<ModelSpaceMesh> AlphaWrapper::getAlphaWrapping(const std::shared_ptr<ModelSpaceMesh> &input) {

    double relative_alpha=1/20.0;
    double relative_offset = 1/600.0;

    cacheMapMutex.lock();
    const auto cacheIterator = cacheMap.find(input);
    cacheMapMutex.unlock();
    if(cacheIterator != cacheMap.end()){
        return cacheIterator->second;
    }

    auto result = createAlphaWrapping(input, relative_alpha, relative_offset);
    cacheMapMutex.lock();
    cacheMap[input] = result;
    cacheMapMutex.unlock();
    return result;
}