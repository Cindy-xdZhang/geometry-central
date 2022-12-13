#pragma once
#include "geometrycentral/surface/direction_fields.h"
#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/exact_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/parameterize.h"
#include "geometrycentral/surface/simple_polygon_mesh.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/surface/vector_heat_method.h"

#include "geometrycentral/surface/boundary_first_flattening.h"

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/structure.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"
#include "polyscope/curve_network.h"
using geometrycentral::Vector2;
using geometrycentral::Vector3;
struct BarycentricVertex {
    double u;
    double v;
    double w;
    size_t faceId;
};
struct BumpMeshParameters {
    Vector2 domain_extent = Vector2::constant(5.f);
    Vector2 bump_center = Vector2::constant(0.f);
    double bump_sigma = 0.4f;
};

struct FlowFieldParameters {
    unsigned int version = 1;
    std::string name = "test02";
    double maxVelocity = 20;
    double tMin = 0.f;
    double tMax = 12.f;
    double extent = 2.f;
    unsigned int numberOfTimeSteps = 32;
    Vector2 vortex_center_start = Vector2({ -2.f, 0 });
    Vector2 vortex_center_end = Vector2({ 2.f, 0 });

    double getTimeStep()
    {
        return (tMax - tMin) / double(numberOfTimeSteps - 1);
    }
};

geometrycentral::Vector3 projectToTangentPlane(geometrycentral::Vector3 v, geometrycentral::Vector3 n);
geometrycentral::Vector2 getPartialDerivativesOfGaussianBump(const Vector2& bumpCenter, double sigma, const Vector2& evaluationPos);
geometrycentral::Vector3 get3DEmbeddingPosition(const geometrycentral::surface::SurfacePoint& barycentricPoint, const geometrycentral::surface::VertexPositionGeometry& geometry);
geometrycentral::Vector3 embeddingPolarCoordinateOnPlane(const Vector2& vortex_core, const double angle, double distance);
inline BarycentricVertex getBarycentricCoordinate(const geometrycentral::Vector2& point, geometrycentral::surface::ManifoldSurfaceMesh& mesh, geometrycentral::surface::VertexPositionGeometry& geometry)
{
    auto getBaricentric = [](const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c, double& u, double& v, double& w) {
        Vector2 v0 = b - a;
        Vector2 v1 = c - a;
        Vector2 v2 = p - a;
        double d00 = geometrycentral::dot(v0, v0);
        double d01 = geometrycentral::dot(v0, v1);
        double d11 = geometrycentral::dot(v1, v1);
        double d20 = geometrycentral::dot(v2, v0);
        double d21 = geometrycentral::dot(v2, v1);
        double denom = d00 * d11 - d01 * d01;
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
        u = 1.0f - v - w;
    };

    double u, v, w;
    bool found = false;
    size_t faceId;
    for (geometrycentral::surface::Face& f : mesh.faces()) {
        Vector3 v1_3d = geometry.vertexPositions[(f.halfedge().vertex().getIndex())];
        Vector3 v2_3d = geometry.vertexPositions[(f.halfedge().next().vertex().getIndex())];
        Vector3 v3_3d = geometry.vertexPositions[(f.halfedge().next().next().vertex().getIndex())];
        Vector2 v1;
        Vector2 v2;
        Vector2 v3;

        v1.x = v1_3d.x;
        v1.y = v1_3d.y;
        v2.x = v2_3d.x;
        v2.y = v2_3d.y;
        v3.x = v3_3d.x;
        v3.y = v3_3d.y;
        // get barycentric coordinate of vortex core.
        getBaricentric(point, v1, v2, v3, u, v, w);

        if ((u >= 0) && (v >= 0) && (w >= 0)) {
            found = true;
            faceId = f.getIndex();
            break;
        }
    }

    if (!found) {
        std::cout << "barycentric coordinates of vortex core could not be determined" << std::endl;
        return {};
    }

    // Pick a point as the source set
    /*std::vector<geometrycentral::surface::SurfacePoint> sourcePoints;
    geometrycentral::surface::Face f = g_mesh_bump->face(faceId);
    Vector3 fBary;
    fBary.x = u;
    fBary.y = v;
    fBary.z = w;*/

    return { u, v, w, faceId };
}
inline geometrycentral::surface::SurfacePoint getSurfacePoint(const Vector2& point, geometrycentral::surface::ManifoldSurfaceMesh& mesh, geometrycentral::surface::VertexPositionGeometry& geometry)
{
    BarycentricVertex bary = getBarycentricCoordinate(point, mesh, geometry);
    const geometrycentral::surface::Face face = mesh.face(bary.faceId);
    Vector3 fBary;
    fBary.x = bary.u;
    fBary.y = bary.v;
    fBary.z = bary.w;
    return { face, fBary };
}

std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> makeGaussianBumpMesh(const Vector2& extent, const Vector2& bumpCenter, double sigma, const int g_numOfVertRows, const int g_numOfVertCols);
std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> makePlaneMesh(const Vector2& extent, const int g_numOfVertRows, const int g_numOfVertCols);