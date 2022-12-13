#include <my_utils.h>
using namespace geometrycentral;
geometrycentral::Vector3 projectToTangentPlane(geometrycentral::Vector3 v, geometrycentral::Vector3 n)
{
    const double normalComponent = dot(v, n);
    return (v - normalComponent * n);
}

Vector2 getPartialDerivativesOfGaussianBump(const Vector2& bumpCenter, double sigma, const Vector2& evaluationPos)
{
    double dX = (evaluationPos.x - bumpCenter.x);
    double dY = (evaluationPos.y - bumpCenter.y);
    constexpr double m_2pi = 2 * M_PI;
    double x2_y2 = dX * dX + dY * dY;
    double partialX = (-1 / (m_2pi * pow(sigma, 4))) * dX * exp(-x2_y2 / (2 * sigma * sigma));
    double partialY = (-1 / (m_2pi * pow(sigma, 4))) * dY * exp(-x2_y2 / (2 * sigma * sigma));
    return Vector2 { partialX, partialY };
}

Vector3 get3DEmbeddingPosition(const geometrycentral::surface::SurfacePoint& barycentricPoint, const geometrycentral::surface::VertexPositionGeometry& geometry)
{
    assert(barycentricPoint.type == geometrycentral::surface::SurfacePointType::Face);

    auto f = barycentricPoint.face;
    auto uvw = barycentricPoint.faceCoords;

    Vector3 v1_3d = geometry.vertexPositions[(f.halfedge().vertex().getIndex())];
    Vector3 v2_3d = geometry.vertexPositions[(f.halfedge().next().vertex().getIndex())];
    Vector3 v3_3d = geometry.vertexPositions[(f.halfedge().next().next().vertex().getIndex())];

    Vector3 thisPoint = uvw.x * v1_3d + uvw.y * v2_3d + uvw.z * v3_3d;
    return thisPoint;
}
Vector3 embeddingPolarCoordinateOnPlane(const Vector2& vortex_core, const double angle, double distance)
{
    // on the plane you can compute exponential map without  geoExac
    Vector2 traceVec = { distance * std::cos(angle), distance * std::sin(angle) };
    auto tracePoint = vortex_core + traceVec;

    return { tracePoint.x, tracePoint.y, 0.0 };
}
std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> makeGaussianBumpMesh(const Vector2& extent, const Vector2& bumpCenter, double sigma, const int g_numOfVertRows, const int g_numOfVertCols)
{

    std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> simpleMesh = std::make_unique<geometrycentral::surface::SimplePolygonMesh>();
    simpleMesh->clear();

    double xExtent = extent.x;
    double yExtent = extent.y;

    Vector3 startPos;
    startPos.x = xExtent * -0.5f;
    startPos.y = yExtent * -0.5f;
    startPos.z = 0.f;

    Vector3 offsetX = Vector3::zero();
    offsetX.x = xExtent / (g_numOfVertCols - 1);
    Vector3 offsetY = Vector3::zero();
    offsetY.y = yExtent / (g_numOfVertRows - 1);

    Vector3 bumpCenter3D;
    bumpCenter3D.x = bumpCenter.x;
    bumpCenter3D.y = bumpCenter.y;
    bumpCenter3D.z = 0.f;

    for (int j = 0; j < g_numOfVertRows; j++) {
        for (int i = 0; i < g_numOfVertCols; i++) {

            Vector3 position = startPos + offsetX * i + offsetY * j;
            Vector3 d = position - bumpCenter3D;
            double dist_squared = d.norm2();
            double val = 1.0 / (sigma * sigma * (2.0 * geometrycentral::PI)) * exp(-0.5 * dist_squared / (sigma * sigma));
            position.z = val;
            simpleMesh->vertexCoordinates.push_back(position);
        }
    }

    for (int j = 0; j < g_numOfVertRows - 1; j++) {
        for (int i = 0; i < g_numOfVertCols - 1; i++) {
            int linindx = j * g_numOfVertCols + i;
            int indx00 = linindx;
            int indx01 = linindx + 1;
            int indx10 = linindx + g_numOfVertCols;
            int indx11 = linindx + g_numOfVertCols + 1;
            std::vector<size_t> face1;
            face1.push_back(indx00);
            face1.push_back(indx01);
            face1.push_back(indx11);

            std::vector<size_t> face2;
            face2.push_back(indx00);
            face2.push_back(indx11);
            face2.push_back(indx10);
            simpleMesh->polygons.push_back(face1);
            simpleMesh->polygons.push_back(face2);
        }
    }

    // simpleMesh->stripUnusedVertices();

    return std::move(simpleMesh);
}

std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> makePlaneMesh(const Vector2& extent, const int g_numOfVertRows, const int g_numOfVertCols)
{

    std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> simpleMesh = std::make_unique<geometrycentral::surface::SimplePolygonMesh>();
    simpleMesh->clear();

    double xExtent = extent.x;
    double yExtent = extent.y;

    Vector3 startPos;
    startPos.x = xExtent * -0.5f;
    startPos.y = yExtent * -0.5f;
    startPos.z = 0;

    Vector3 offsetX = Vector3::zero();
    offsetX.x = xExtent / g_numOfVertCols;
    Vector3 offsetY = Vector3::zero();
    offsetY.y = yExtent / g_numOfVertRows;

    for (int j = 0; j < g_numOfVertRows; j++) {
        for (int i = 0; i < g_numOfVertCols; i++) {

            Vector3 position = startPos + offsetX * i + offsetY * j;
            simpleMesh->vertexCoordinates.push_back(position);
        }
    }

    for (int j = 0; j < g_numOfVertRows - 1; j++) {
        for (int i = 0; i < g_numOfVertCols - 1; i++) {
            int linindx = j * g_numOfVertCols + i;
            int indx00 = linindx;
            int indx01 = linindx + 1;
            int indx10 = linindx + g_numOfVertCols;
            int indx11 = linindx + g_numOfVertCols + 1;
            std::vector<size_t> face1;
            face1.push_back(indx00);
            face1.push_back(indx01);
            face1.push_back(indx11);

            std::vector<size_t> face2;
            face2.push_back(indx00);
            face2.push_back(indx11);
            face2.push_back(indx10);
            simpleMesh->polygons.push_back(face1);
            simpleMesh->polygons.push_back(face2);
        }
    }

    simpleMesh->stripUnusedVertices();

    return std::move(simpleMesh);
}
