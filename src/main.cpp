#include <my_utils.h>
// using namespace geometrycentral;
// using namespace geometrycentral::surface;

using namespace geometrycentral;
using namespace geometrycentral::surface;
// == Geometry-central data
// std::unique_ptr<ManifoldSurfaceMesh> mesh;
// std::unique_ptr<VertexPositionGeometry> geometry;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh_bump;
polyscope::SurfaceMesh* psMesh_plane;

std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> g_mesh_bump;
std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> g_geometry_bump;
geometrycentral::surface::ManifoldSurfaceMesh* g_mesh_plane;
geometrycentral::surface::VertexPositionGeometry* g_geometry_plane;

std::unique_ptr<geometrycentral::surface::GeodesicAlgorithmExact> g_mmp;
polyscope::PointCloud* g_psCloud;

int g_numOfVertCols = 32;
int g_numOfVertRows = 32;
unsigned int g_numOfTimeStep = 32;
bool g_onBump = false;
double g_time = 0;
bool g_Animate = false;
int g_selectionRow = 0;
int g_selectionCol = 0;
int g_integer_t_debug = 0;
FlowFieldParameters g_flowFieldParameters;
BumpMeshParameters g_bumpMeshParameters;
std::vector<geometrycentral::surface::VertexData<Vector3>> g_pushForwardVectorField;
std::vector<geometrycentral::surface::VertexData<double>> g_geoDistance;

namespace {

double extentMin = 0.0001;
double extentMax = 100.0;
double doubleMin = 0.0001;
double doubleMax = 100.0;

void computeFlowFieldOnBump(double time, geometrycentral::surface::VertexData<Vector3>& vel_bump, geometrycentral::surface::VertexData<Vector3>& vel_plane)
{

    // auto getBaricentric = []
    //     // Compute barycentric coordinates (u, v, w) for
    //     // point p with respect to triangle (a, b, c)
    //     (const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c, double& u, double& v, double& w) {
    //         Vector2 v0 = b - a;
    //         Vector2 v1 = c - a;
    //         Vector2 v2 = p - a;
    //         double d00 = geometrycentral::dot(v0, v0);
    //         double d01 = geometrycentral::dot(v0, v1);
    //         double d11 = geometrycentral::dot(v1, v1);
    //         double d20 = geometrycentral::dot(v2, v0);
    //         double d21 = geometrycentral::dot(v2, v1);
    //         double denom = d00 * d11 - d01 * d01;
    //         v = (d11 * d20 - d01 * d21) / denom;
    //         w = (d00 * d21 - d01 * d20) / denom;
    //         u = 1.0f - v - w;
    //     };

    // double u, v, w;
    // bool found = false;
    // size_t faceId;
    // double oneTimeStepDuration = (g_flowFieldParameters.tMax - g_flowFieldParameters.tMin) / (g_flowFieldParameters.numberOfTimeSteps - 1);
    // double normalizedTimeStep = (time - g_flowFieldParameters.tMin) / oneTimeStepDuration / (g_flowFieldParameters.numberOfTimeSteps - 1);
    // Vector2 vortex_center = g_flowFieldParameters.vortex_center_start + normalizedTimeStep * (g_flowFieldParameters.vortex_center_end - g_flowFieldParameters.vortex_center_start);

    // for (geometrycentral::surface::Face& f : g_mesh_bump->faces()) {
    //     Vector3 v1_3d = g_geometry_bump->vertexPositions[(f.halfedge().vertex().getIndex())];
    //     Vector3 v2_3d = g_geometry_bump->vertexPositions[(f.halfedge().next().vertex().getIndex())];
    //     Vector3 v3_3d = g_geometry_bump->vertexPositions[(f.halfedge().next().next().vertex().getIndex())];
    //     Vector2 v1;
    //     Vector2 v2;
    //     Vector2 v3;

    //    v1.x = v1_3d.x;
    //    v1.y = v1_3d.y;
    //    v2.x = v2_3d.x;
    //    v2.y = v2_3d.y;
    //    v3.x = v3_3d.x;
    //    v3.y = v3_3d.y;
    //    // get barycentric coordinate of vortex core.
    //    getBaricentric(vortex_center, v1, v2, v3, u, v, w);

    //    if ((u >= 0) && (v >= 0) && (w >= 0)) {
    //        found = true;
    //        faceId = f.getIndex();
    //        break;
    //    }
    //}

    // if (!found) {
    //     std::cout << "barycentric coordinates of vortex core could not be determined" << std::endl;
    //     return;
    // }

    //// Pick a point as the source set
    // std::vector<geometrycentral::surface::SurfacePoint> sourcePoints;
    // geometrycentral::surface::Face f = g_mesh_bump->face(faceId);
    // Vector3 fBary;
    // fBary.x = u;
    // fBary.y = v;
    // fBary.z = w;
    // sourcePoints.push_back(geometrycentral::surface::SurfacePoint(f, fBary));

    //// Run MMP from these source points
    // g_mmp->propagate(sourcePoints);

    //// Get the distance function at all mesh vertices
    // geometrycentral::surface::VertexData<double> distToSource = g_mmp->getDistanceFunction();
    // psMesh_bump->addVertexDistanceQuantity("dist", distToSource);
    // psMesh_bump->getQuantity("dist")->setEnabled(true);

    // for (int i = 0; i < g_mesh_bump->nVertices(); i++) {
    //     // for (Vertex& v_bump : g_mesh_bump->vertices()) {
    //     geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
    //     geometrycentral::surface::Vertex& v_plane = g_mesh_plane->vertex(i);

    //    geometrycentral::surface::SurfacePoint queryPoint(v_bump);
    //    double distance = g_mmp->getDistance(queryPoint);

    //    Vector3 vBasisX = g_geometry_bump->vertexTangentBasis[v_bump][0];
    //    Vector3 vBasisY = g_geometry_bump->vertexTangentBasis[v_bump][1];

    //    int index = v_bump.getIndex();
    //    Vector3 pos_3d = g_geometry_bump->vertexPositions[index];
    //    Vector2 pos;
    //    pos.x = pos_3d.x;
    //    pos.y = pos_3d.y;

    //    Vector2 direction = pos - vortex_center;
    //    Vector2 velocityComponents = getVelocityOneVortexFromDirection(direction, distance, time);

    //    // Vector2 velocityComponents = getVelocityOneVortexFromPos(pos, time);

    //    Vector3 vBasisX_plane = g_geometry_plane->vertexTangentBasis[v_plane][0];
    //    Vector3 vBasisY_plane = g_geometry_plane->vertexTangentBasis[v_plane][1];
    //    Vector3 vel3D_plane = vBasisX_plane * velocityComponents.x + vBasisY_plane * velocityComponents.y;
    //    vel_plane[v_plane] = vel3D_plane;

    //    Vector3 vBasisX_bump = g_geometry_bump->vertexTangentBasis[v_bump][0];
    //    Vector3 vBasisY_bump = g_geometry_bump->vertexTangentBasis[v_bump][1];
    //    Vector3 vel3D_bump = vBasisX_bump * velocityComponents.x + vBasisY_bump * velocityComponents.y;
    //    vel_bump[v_bump] = vel3D_bump;
    //}
}

inline double getDuration()
{
    return (g_flowFieldParameters.tMax - g_flowFieldParameters.tMin);
}

inline double getDiscreteTimeStepDuration()
{
    double duration = getDuration();
    return duration / (g_flowFieldParameters.numberOfTimeSteps - 1);
}

inline double getNormalizedTimeStep(double time)
{
    double duration = getDuration();
    double normalizedTimeStep = (time - g_flowFieldParameters.tMin) / duration;
    return normalizedTimeStep;
}

inline Vector2 getVortexCenter(double time)
{
    double normalizedTimeStep = getNormalizedTimeStep(time);
    return g_flowFieldParameters.vortex_center_start + normalizedTimeStep * (g_flowFieldParameters.vortex_center_end - g_flowFieldParameters.vortex_center_start);
}

Eigen::Matrix3d getFaceLocalFrame(geometrycentral::surface::ManifoldSurfaceMesh* mesh,
    geometrycentral::surface::VertexPositionGeometry* geometry, const geometrycentral::surface::Face& f)
{

    // local coordinate of vortex core
    geometry->requireFaceTangentBasis();
    geometry->requireFaceNormals();
    Vector3 core_local_frame_x = geometry->faceTangentBasis[f][0];
    Vector3 core_local_frame_y = geometry->faceTangentBasis[f][1];
    Vector3 core_local_frame_z = geometry->faceNormal(f);

    Eigen::Matrix3d matLocaltoCartesian;
    matLocaltoCartesian.col(0) = Eigen::Vector3d { core_local_frame_x.x, core_local_frame_x.y, core_local_frame_x.z };
    matLocaltoCartesian.col(1) = Eigen::Vector3d { core_local_frame_y.x, core_local_frame_y.y, core_local_frame_y.z };
    matLocaltoCartesian.col(2) = Eigen::Vector3d { core_local_frame_z.x, core_local_frame_z.y, core_local_frame_z.z };
    return matLocaltoCartesian;
}

Eigen::Matrix3d getVertexLocalFrame(geometrycentral::surface::ManifoldSurfaceMesh* mesh,
    geometrycentral::surface::VertexPositionGeometry* geometry, const geometrycentral::surface::Vertex& v)
{

    // local coordinate of vortex core
    geometry->requireVertexTangentBasis();
    geometry->requireVertexNormals();
    Vector3 core_local_frame_x = geometry->vertexTangentBasis[v][0];
    Vector3 core_local_frame_y = geometry->vertexTangentBasis[v][1];
    Vector3 core_local_frame_z = geometry->vertexNormals[v];

    Eigen::Matrix3d matLocaltoCartesian;
    matLocaltoCartesian.col(0) = Eigen::Vector3d { core_local_frame_x.x, core_local_frame_x.y, core_local_frame_x.z };
    matLocaltoCartesian.col(1) = Eigen::Vector3d { core_local_frame_y.x, core_local_frame_y.y, core_local_frame_y.z };
    matLocaltoCartesian.col(2) = Eigen::Vector3d { core_local_frame_z.x, core_local_frame_z.y, core_local_frame_z.z };
    return matLocaltoCartesian;
}

inline Vector3 embeddingPolarCoordinateOnMesh(const geometrycentral::surface::SurfacePoint& vortex_core, geometrycentral::surface::VertexPositionGeometry& geometry, const double angle, double distance)
{

    Vector2 traceVec = distance * normalize(Vector2::fromAngle(angle));

    SurfacePoint pathEndpoint = traceGeodesic(geometry, vortex_core, traceVec).endPoint.inSomeFace();
    Vector3 v_pos3 = get3DEmbeddingPosition(pathEndpoint, geometry);
    return v_pos3;
}

geometrycentral::Vector3 backTrackGeodesicDirection(const Vector3& sourceCore, geometrycentral::surface::GeodesicAlgorithmExact* gmm, const geometrycentral::surface::Vertex& v, const geometrycentral::Vector3& n)
{

    const double geoDistance0 = gmm->getDistance(v);
    std::vector<SurfacePoint> geodesicCurve = gmm->traceBack(v);
    assert(geodesicCurve.size() > 0);
    geodesicCurve.pop_back();

    // map point from bump to geodesic polar coordinate
    Vector3 lastPoint = get3DEmbeddingPosition(geodesicCurve.back().inSomeFace(), *g_geometry_bump);
    // get angle of polar coordinate
    Vector3 approximateTangentVector3d = (lastPoint - sourceCore);
    if (approximateTangentVector3d.norm() < 1e-8) {
        geodesicCurve.pop_back();
        lastPoint = get3DEmbeddingPosition(geodesicCurve.back().inSomeFace(), *g_geometry_bump);
        approximateTangentVector3d = (lastPoint - sourceCore);
    }
    approximateTangentVector3d = projectToTangentPlane(approximateTangentVector3d, n);
    approximateTangentVector3d = normalize(approximateTangentVector3d) * geoDistance0;
    return approximateTangentVector3d;
}

Vector2 getVelocityOneVortexFromDirection(const Vector2& direction, double distance, double time)
{
    Vector2 vortex_center = getVortexCenter(time);
    Vector2 pos = direction.normalize() * distance;

    double xChart1 = pos.x;
    double yChart1 = pos.y;

    double vecU1 = g_flowFieldParameters.maxVelocity * yChart1 * exp((-xChart1 * xChart1 - yChart1 * yChart1) * 10.0 * g_flowFieldParameters.extent);
    double vecV1 = g_flowFieldParameters.maxVelocity * -xChart1 * exp((-xChart1 * xChart1 - yChart1 * yChart1) * 10.0 * g_flowFieldParameters.extent);

    Vector2 components;
    components.x = vecU1;
    components.y = vecV1;

    return components;
};

Vector2 getVelocityOneVortexFromPos(const Vector2& pos, double time)
{
    Vector2 vortex_center = getVortexCenter(time);

    /*int c = (pos.x + (g_bumpMeshParameters.domain_extent.x / 2.f)) / (g_bumpMeshParameters.domain_extent.x) * 127;
    int r = (pos.y + (g_bumpMeshParameters.domain_extent.y / 2.f)) / (g_bumpMeshParameters.domain_extent.y) * 127;
    double x = (((double)c * (1.f / 127.f)) - 0.5) * 2.f;
    double y = (((double)r * (1.f / 127.f)) - 0.5) * 2.f;
    double normalizedtime = getNormalizedTimeStep(time);
    x -= vortex_center.x;
    y -= vortex_center.y;
    double vx = g_flowFieldParameters.maxVelocity * y * exp((-x * x - y * y) * 10.0 * g_flowFieldParameters.extent);
    double vy = g_flowFieldParameters.maxVelocity * -x * exp((-x * x - y * y) * 10.0 * g_flowFieldParameters.extent);*/

    double x = pos.x;
    double y = pos.y;

    x -= vortex_center.x;
    y -= vortex_center.y;

    double vx = g_flowFieldParameters.maxVelocity * y * exp((-x * x - y * y) * 10.0 * g_flowFieldParameters.extent);
    double vy = g_flowFieldParameters.maxVelocity * -x * exp((-x * x - y * y) * 10.0 * g_flowFieldParameters.extent);

    /* Vector2 evaluationPos = (pos - vortex_center);
     double vecU1 = g_flowFieldParameters.maxVelocity * evaluationPos.y * exp((-evaluationPos.x * evaluationPos.x - evaluationPos.y * evaluationPos.y) * 10.0 * g_flowFieldParameters.extent);
     double vecV1 = g_flowFieldParameters.maxVelocity * -evaluationPos.x * exp((-evaluationPos.x * evaluationPos.x - evaluationPos.y * evaluationPos.y) * 10.0 * g_flowFieldParameters.extent);

     Vector2 components;
     components.x = vecU1;
     components.y = vecV1;*/

    Vector2 components;
    components.x = vx;
    components.y = vy;

    return components;
};

void computeFlowFieldOnPlane(double time, geometrycentral::surface::VertexData<Vector3>& vel_plane)
{

    Vector2 vortex_center = getVortexCenter(time);

    for (int i = 0; i < g_mesh_bump->nVertices(); i++) {
        geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
        geometrycentral::surface::Vertex& v_plane = g_mesh_plane->vertex(i);

        Vector3 v_pos3D = g_geometry_plane->inputVertexPositions[i];
        Vector2 v_pos;
        v_pos.x = v_pos3D.x;
        v_pos.y = v_pos3D.y;

        Vector2 velocityComponents = getVelocityOneVortexFromPos(v_pos, time);

        // Vector2 velocityComponents;
        // velocityComponents.x = 0.25;
        // velocityComponents.y = -0.25;

        // Vector3 vBasisX_plane = g_geometry_plane->vertexTangentBasis[v_plane][0];
        // Vector3 vBasisY_plane = g_geometry_plane->vertexTangentBasis[v_plane][1];

        Vector3 vBasisX_plane { 1, 0, 0 };
        Vector3 vBasisY_plane { 0, 1, 0 };
        Vector3 vel3D_plane = vBasisX_plane * velocityComponents.x + vBasisY_plane * velocityComponents.y;
        vel3D_plane.x = vel3D_plane.x;
        vel3D_plane.y = vel3D_plane.y;
        vel3D_plane.z = 0;
        vel_plane[v_plane] = vel3D_plane;
    }
}

void drawRandomGeodesicCurves(geometrycentral::surface::SurfacePoint& vortex_core_Onsurface)
{
    std::vector<Vector3> positions;
    std::vector<std::array<size_t, 2>> edgeInds;
    size_t eid = 0;

    positions.reserve(g_mesh_bump->nVertices() * 128);
    edgeInds.reserve(g_mesh_bump->nVertices() * 20);
    // g_mmp->propagate(sourcePoints);
    for (int i = 0; i < g_mesh_bump->nVertices(); i += 1) {
        geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
        geometrycentral::surface::SurfacePoint queryPoint(v_bump);

        if (i % 137 == 0) {
            std::vector<SurfacePoint> geodesicCurve = g_mmp->traceBack(queryPoint); // crash
            for (const auto& point : geodesicCurve) {
                positions.emplace_back(get3DEmbeddingPosition(point.inSomeFace(), *g_geometry_bump));
                edgeInds.push_back({ eid, eid + 1 });
                eid += 1;
            }
            edgeInds.pop_back();
        }
    }
    psMesh_bump->removeQuantity("path edges");
    auto pathQ = psMesh_bump->addSurfaceGraphQuantity("path edges", positions, edgeInds);
    pathQ->setEnabled(true);
    pathQ->setColor(polyscope::render::RGB_RED);
    pathQ->setRadius(0.001);
}

inline void CreatePolarCoordinateBasisOnPlane(const Vector2& vortex_core, geometrycentral::surface::VertexData<Vector3>& vBasis1, geometrycentral::surface::VertexData<Vector3>& vBasis2)
{
    geometrycentral::surface::VertexData<Vector3> Basis_r(*g_mesh_plane);
    geometrycentral::surface::VertexData<Vector3> Basis_theta(*g_mesh_plane);

    for (geometrycentral::surface::Vertex v : g_mesh_plane->vertices()) {
        Vector2 v_pos = { g_geometry_plane->inputVertexPositions[v].x, g_geometry_plane->inputVertexPositions[v].y };
        auto r = v_pos - vortex_core;
        auto this_basis_r = normalize(r);
        Vector2 theta_basis = (this_basis_r.rotate90()) * r.norm();
        Basis_r[v] = Vector3 { this_basis_r.x, this_basis_r.y, 0.0 };
        Basis_theta[v] = Vector3 { theta_basis.x, theta_basis.y, 0.0 };
    }
    // psMesh_plane->setVertexTangentBasisX(Basis_r);
    psMesh_plane->addVertexVectorQuantity("basis r", Basis_r);
    psMesh_plane->addVertexVectorQuantity("basis theta", Basis_theta);

    vBasis1 = Basis_r;
    vBasis2 = Basis_theta;
}

inline Eigen::Vector2d getUVCoordinate(const Vector2 basis_u, const Vector2 basis_v, const Vector2& faceVec)
{
    Eigen::Matrix<double, 2, 2> A;
    A.col(0) << basis_u.x, basis_u.y;
    A.col(1) << basis_v.x, basis_v.y;
    Eigen::Vector2d rhs;
    rhs << faceVec.x, faceVec.y;
    // Solve
    Eigen::Vector2d result = A.colPivHouseholderQr().solve(rhs);
    return result;
}

inline Eigen::Vector2d getUVCoordinate(const Vector3 basis_u, const Vector3 basis_v, const Vector3& faceVec)
{
    Eigen::Matrix<double, 3, 2> A;
    A.col(0) << basis_u.x, basis_u.y, basis_u.z;
    A.col(1) << basis_v.x, basis_v.y, basis_v.z;
    Eigen::Vector3d rhs;
    rhs << faceVec.x, faceVec.y, faceVec.z;
    // Solve
    Eigen::Vector2d result = A.colPivHouseholderQr().solve(rhs);

    return result;
}

inline void CreatePolarCoordinateBasisOnBump0(const geometrycentral::surface::SurfacePoint& vortex_core_bump,
    const geometrycentral::surface::VertexData<double>& geoDistance, const geometrycentral::surface::VertexData<double>& angle,
    geometrycentral::surface::VertexData<Vector3>& vBasis1, geometrycentral::surface::VertexData<Vector3>& vBasis2)
{

    geometrycentral::surface::VertexData<Vector3> Basis_r(*g_mesh_bump);
    geometrycentral::surface::VertexData<Vector3> Basis_theta(*g_mesh_bump);

    // geometrycentral::surface::VectorHeatMethodSolver vhmSolver(*g_geometry_bump);
    //  logarithmMap is not as precise as traceBack, it returns tangent vector (polar coordinate) in source point local tangent space
    // geometrycentral::surface::VertexData<Vector2> logarithMap = vhmSolver.computeLogMap(vortex_core_bump);
    g_geometry_bump->requireVertexTangentBasis();
    g_geometry_bump->requireVertexNormals();
    const auto vortex_core_embedding = get3DEmbeddingPosition(vortex_core_bump, *g_geometry_bump);

    for (geometrycentral::surface::Vertex vert : g_mesh_bump->vertices()) {
        const int num_vertex = vert.degree();
        Eigen::Matrix<double, -1, 2> A;
        A.resize(num_vertex, 2);
        Eigen::VectorXd R(num_vertex);
        Eigen::VectorXd Theta(num_vertex);
        int i = 0;
        const Vector3 local_frame_x = g_geometry_bump->vertexTangentBasis[vert][0];
        const Vector3 local_frame_y = g_geometry_bump->vertexTangentBasis[vert][1];
        const Vector3 local_frame_z = g_geometry_bump->vertexNormals[vert];
        for (Halfedge he : vert.outgoingHalfedges()) {
            const auto adjacent_v = he.twin().vertex();
            const auto projectedVec = projectToTangentPlane(g_geometry_bump->vertexPositions[adjacent_v] - vortex_core_embedding, local_frame_z);
            Eigen::Vector2d uv_i = getUVCoordinate(local_frame_x, local_frame_y, projectedVec);

            A(i, 0) = uv_i(0);
            A(i, 1) = uv_i(1);
            Theta(i) = angle[adjacent_v];
            R(i) = geoDistance[adjacent_v];
            i++;
        }

        Eigen::Matrix2d drdtheta;
        drdtheta.row(0) = A.colPivHouseholderQr().solve(R);
        drdtheta.row(1) = A.colPivHouseholderQr().solve(Theta);
        Eigen::Matrix2d basis = drdtheta.inverse();

        Basis_r[vert] = normalize(basis(0, 0) * local_frame_x + basis(1, 0) * local_frame_y);
        Basis_theta[vert] = normalize(basis(0, 1) * local_frame_x + basis(1, 1) * local_frame_y);
    }
    // psMesh_plane->setVertexTangentBasisX(Basis_r);
    psMesh_bump->addVertexVectorQuantity("basis r", Basis_r);
    psMesh_bump->addVertexVectorQuantity("basis theta", Basis_theta);
    vBasis1 = Basis_r;
    vBasis2 = Basis_theta;
}
inline void CreatePolarCoordinateBasisOnBump1(const geometrycentral::surface::SurfacePoint& vortex_core_bump,
    const geometrycentral::surface::VertexData<double>& geoDistance, const geometrycentral::surface::VertexData<Vector3>& direction,
    geometrycentral::surface::VertexData<Vector3>& vBasis1, geometrycentral::surface::VertexData<Vector3>& vBasis2)
{

    geometrycentral::surface::VertexData<Vector3> Basis_r(*g_mesh_bump);
    geometrycentral::surface::VertexData<Vector3> Basis_theta(*g_mesh_bump);

    // geometrycentral::surface::VectorHeatMethodSolver vhmSolver(*g_geometry_bump);
    //  logarithmMap is not as precise as traceBack, it returns tangent vector (polar coordinate) in source point local tangent space
    // geometrycentral::surface::VertexData<Vector2> logarithMap = vhmSolver.computeLogMap(vortex_core_bump);
    g_geometry_bump->requireVertexTangentBasis();
    g_geometry_bump->requireVertexNormals();
    const auto vortex_core_embedding = get3DEmbeddingPosition(vortex_core_bump, *g_geometry_bump);

    for (geometrycentral::surface::Vertex vert : g_mesh_bump->vertices()) {
        const Vector3 local_frame_x = g_geometry_bump->vertexTangentBasis[vert][0];
        const Vector3 local_frame_y = g_geometry_bump->vertexTangentBasis[vert][1];
        const Vector3 local_frame_z = g_geometry_bump->vertexNormals[vert];
        Basis_r[vert] = normalize(direction[vert]);
        const Vector2 uv_i = { getUVCoordinate(local_frame_x, local_frame_y, Basis_r[vert])(0), getUVCoordinate(local_frame_x, local_frame_y, Basis_r[vert])(1) };
        const Vector2 uv_i_flip = uv_i.rotate90();
        Basis_theta[vert] = geoDistance[vert] * normalize(uv_i_flip.x * local_frame_x + uv_i_flip.y * local_frame_y);
    }
    // psMesh_plane->setVertexTangentBasisX(Basis_r);
    psMesh_bump->addVertexVectorQuantity("basis r", Basis_r);
    psMesh_bump->addVertexVectorQuantity("basis theta", Basis_theta);
    vBasis1 = Basis_r;
    vBasis2 = Basis_theta;
}

void getOrCreateBumpTangentBasis(geometrycentral::surface::VertexData<Vector3>& vBasisX, geometrycentral::surface::VertexData<Vector3>& vBasisY,
    geometrycentral::surface::VertexData<Vector3>& normalTangentPlane)
{
    static bool has = false;
    static geometrycentral::surface::VertexData<Vector3> svBasisX(*g_mesh_bump);
    static geometrycentral::surface::VertexData<Vector3> svBasisY(*g_mesh_bump);
    static geometrycentral::surface::VertexData<Vector3> snormalTangetPlane(*g_mesh_bump);
    if (has == false) {

        for (geometrycentral::surface::Vertex v : g_mesh_bump->vertices()) {
            Vector3 v_pos3D = g_geometry_bump->inputVertexPositions[v];

            Vector2 evaluationPos { v_pos3D.x, v_pos3D.y };
            Vector2 partials = getPartialDerivativesOfGaussianBump(g_bumpMeshParameters.bump_center, g_bumpMeshParameters.bump_sigma, evaluationPos);
            auto v1 = normalize(Vector3 { 1., 0, partials.x });
            auto v2 = normalize(Vector3 { 0, 1., partials.y });
            svBasisX[v] = v1;
            svBasisY[v] = v2;
            snormalTangetPlane[v] = normalize(cross(v1, v2));
        }

        psMesh_bump->setVertexTangentBasisX(svBasisX);
        /*psMesh_bump->addVertexIntrinsicVectorQuantity("X1", vBasisX);
        psMesh_bump->addVertexIntrinsicVectorQuantity("X2", vBasisY);*/
        psMesh_bump->addVertexVectorQuantity("X1", svBasisX);
        psMesh_bump->addVertexVectorQuantity("X2", svBasisY);
        psMesh_bump->addVertexVectorQuantity("vertex normal", snormalTangetPlane);

        has = true;
    }
    vBasisX = svBasisX;
    vBasisY = svBasisY;
    normalTangentPlane = snormalTangetPlane;
}

void demoGeodesicCurve(/* geometrycentral::surface::VertexData<Vector3>& vel_bump, geometrycentral::surface::VertexData<Vector3>& vel_plane*/)
{
    try {

        geometrycentral::surface::VertexData<Vector3> vBasisX {};
        geometrycentral::surface::VertexData<Vector3> vBasisY {};
        geometrycentral::surface::VertexData<Vector3> normalTangetPlane {};
        getOrCreateBumpTangentBasis(vBasisX, vBasisY, normalTangetPlane);
        geometrycentral::surface::VertexData<Vector3> polarBasis_r_plane {};
        geometrycentral::surface::VertexData<Vector3> polarBasis_theta_plane {};
        geometrycentral::surface::VertexData<Vector3> polarBasis_r_bump {};
        geometrycentral::surface::VertexData<Vector3> polarBasis_theta_bump {};
        geometrycentral::surface::VertexData<Vector3> vectorField(*g_mesh_bump);
        geometrycentral::surface::VertexData<Vector3> direction(*g_mesh_bump);
        geometrycentral::surface::VertexData<double> angles(*g_mesh_bump);
        // In the TP of bump mesh, the vector that project to { 1, 0} is defined as angle zero direction.->basis vBasisX is the zero direction
        constexpr Vector2 planeAnglezeroDir { 1, 0 };

        const auto time = 0 + 21 * g_flowFieldParameters.getTimeStep();
        // const auto time = g_time;
        const auto vortex_core2D = getVortexCenter(time);
        // CreatePolarCoordinateBasisOnPlane(vortex_core2D, polarBasis_r_plane, polarBasis_theta_plane);

        const auto vortex_core_barycentric = getBarycentricCoordinate(vortex_core2D, *g_mesh_bump, *g_geometry_bump);
        const geometrycentral::surface::Face f = g_mesh_bump->face(vortex_core_barycentric.faceId);
        const geometrycentral::surface::Face faceOnPlane = g_mesh_plane->face(vortex_core_barycentric.faceId);
        const Vector3 fBary = { vortex_core_barycentric.u, vortex_core_barycentric.v, vortex_core_barycentric.w };
        const auto vortex_core_Onplane = geometrycentral::surface::SurfacePoint(faceOnPlane, fBary);
        const auto vortex_core_OnBump = geometrycentral::surface::SurfacePoint(f, fBary);
        const auto vortex_core_embedding = get3DEmbeddingPosition(vortex_core_OnBump, *g_geometry_bump);
        const auto matLocaltoCartesian = getFaceLocalFrame(g_mesh_bump.get(), g_geometry_bump.get(), f);
        Eigen::Matrix3d matLocaltoCartesianInv = matLocaltoCartesian.inverse();

        // Run MMP from these source points
        g_mmp->propagate(vortex_core_OnBump);
        geometrycentral::surface::VertexData<double> distToSource = g_mmp->getDistanceFunction();
        // drawRandomGeodesicCurves(vortex_core_Onsurface);
        const int Nv = g_mesh_bump->nVertices();

        const double delta_mesh = 1.25 * g_bumpMeshParameters.domain_extent.x / (g_numOfVertCols - 1);

        //#pragma omp parallel for
        for (int i = 0; i < Nv; i += 1) {
            geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
            const Vector3& n = normalTangetPlane[v_bump];
            const Vector3 destinationPoint = g_geometry_bump->vertexPositions[v_bump];
            auto BackTrackVec = backTrackGeodesicDirection(vortex_core_embedding, g_mmp.get(), v_bump, n);
            const double geoDistance0 = BackTrackVec.norm();
            /*        if (geoDistance0 > 0.38) {
                        continue;
                    }*/
            direction[v_bump] = BackTrackVec;
            const double cosA = (dot(BackTrackVec, vBasisX[v_bump])) / (vBasisX[v_bump].norm() * BackTrackVec.norm());
            double angle = (acos(cosA));
            if (BackTrackVec.y < 0)
                angle = 2 * M_PI - angle;
            angles[v_bump] = angle;

            {
                // debuging code
                //
                // trackBackVec  is in word Cartesian space, trackBackVec =trackBackVec.x *{1,0,0}+trackBackVec.y *{0,1,0}
                // need to transform trackBackVec to face local coordinate
                /*
                Eigen::Vector3d trackBackVec = { geoDistance0 * std::cos(angle), geoDistance0 * std::sin(angle), 0 };
                Eigen::Vector3d trackBackVeclocal = matLocaltoCartesianInv * trackBackVec;


               auto traceRes = traceGeodesic(*g_geometry_bump, vortex_core_OnBump, { trackBackVeclocal[0], trackBackVeclocal[1] }, { true });
                SurfacePoint debug_sfp = traceRes.endPoint.inSomeFace();

                Vector3 debug_inversePoint = get3DEmbeddingPosition(debug_sfp, *g_geometry_bump);
                assert(norm(debug_inversePoint - destinationPoint) < 1e-7);*/
            }

            /*Vector3 v_pos3 = embeddingPolarCoordinateOnMesh(vortex_core_Onplane, *g_geometry_plane, angle, g_mmp->getDistance(v_bump));
            Vector3 v_bumpCorrespondingPoint_Plane = embeddingPolarCoordinateOnPlane(vortex_core2D, angle, geoDistance0);
            Vector2 v_bumpCorrespondingPoint_Plane2D = { v_bumpCorrespondingPoint_Plane.x, v_bumpCorrespondingPoint_Plane.y };
            Vector2 vectorfield = getVelocityOneVortexFromPos(v_bumpCorrespondingPoint_Plane2D, time);

            Vector2 pointPlusDelta_Plane = v_bumpCorrespondingPoint_Plane2D + delta_mesh * vectorfield;
            const auto r_vector = pointPlusDelta_Plane - vortex_core2D;
            Vector3 vf3d = Vector3::zero();

            Eigen::Vector3d trackBackVec = { r_vector.x, r_vector.y, 0 };
            Eigen::Vector3d trackBackVeclocal = matLocaltoCartesianInv * trackBackVec;

            auto traceRes = traceGeodesic(*g_geometry_bump, vortex_core_OnBump, { trackBackVeclocal[0], trackBackVeclocal[1] }, { true });

            Vector3 PointPlusDelta_bump = get3DEmbeddingPosition(traceRes.endPoint.inSomeFace(), *g_geometry_bump);

            auto pushforwardedVector = (PointPlusDelta_bump - destinationPoint) / delta_mesh;
            // vf3d = Vector3projToTp(pushforwardedVector, n);
            vf3d = pushforwardedVector;
            // Vector3 vf3d = vf2d.x * vBasisX[v_bump] + vf2d.y * vBasisY[v_bump];
            vectorField[v_bump] = vf3d;*/
        }
        // CreatePolarCoordinateBasisOnBump0(vortex_core_OnBump, distToSource, angles, polarBasis_r_bump, polarBasis_theta_bump);
        CreatePolarCoordinateBasisOnBump1(vortex_core_OnBump, distToSource, direction, polarBasis_r_bump, polarBasis_theta_bump);

        for (int i = 0; i < Nv; i += 1) {
            geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
            double angle = angles[v_bump];
            double geoDistance0 = distToSource[v_bump];
            Vector3 v_bumpCorrespondingPoint_Plane = embeddingPolarCoordinateOnPlane(vortex_core2D, angle, geoDistance0);
            Vector2 vortexfieldVec = getVelocityOneVortexFromPos(Vector2 { v_bumpCorrespondingPoint_Plane.x, v_bumpCorrespondingPoint_Plane.y }, time);
            // get polar basis on plane
            Vector2 r = Vector2 { v_bumpCorrespondingPoint_Plane.x, v_bumpCorrespondingPoint_Plane.y } - vortex_core2D;
            auto this_basis_r = normalize(r);
            Vector2 this_theta_basis = (this_basis_r.rotate90()) * r.norm();
            auto uv = getUVCoordinate(this_basis_r, this_theta_basis, vortexfieldVec);
            auto pushforwardedVector = uv(0) * polarBasis_r_bump[v_bump] + uv(1) * polarBasis_theta_bump[v_bump];
            vectorField[v_bump] = pushforwardedVector;
        }

        psMesh_bump->addVertexDistanceQuantity("dist", distToSource);

        psMesh_bump->getQuantity("dist")->setEnabled(true);
        psMesh_bump->addVertexScalarQuantity("geo angle", angles);
        psMesh_bump->addVertexVectorQuantity("lastPoint-core", direction);

        psMesh_bump->removeQuantity("V");
        psMesh_bump->addVertexVectorQuantity("V", vectorField);
        psMesh_bump->getQuantity("V")->setEnabled(true);

        psMesh_bump->addVertexVectorQuantity("vertex normal", normalTangetPlane);
        // psMesh_bump->addVertexScalarQuantity("dist", angles);
        // psMesh_bump->getQuantity("dist")->setEnabled(true);

    } catch (const std::exception& exption) {
        std::cerr << exption.what();
    }
}

void updateFlowOnBumpFromCache(double time, const std::vector<geometrycentral::surface::VertexData<Vector3>>& pushForwardVectorField, std::vector<geometrycentral::surface::VertexData<double>>& geoDistance)
{

    const int Tdim = g_flowFieldParameters.numberOfTimeSteps;

    g_integer_t_debug = static_cast<int>(getNormalizedTimeStep(time) * Tdim);
    if (g_integer_t_debug < static_cast<int>(pushForwardVectorField.size())) {
        const auto& vf = pushForwardVectorField[g_integer_t_debug];
        const auto& distToSource = geoDistance[g_integer_t_debug];

        psMesh_bump->removeQuantity("geo dist");
        psMesh_bump->addVertexDistanceQuantity("geo dist", distToSource);
        psMesh_bump->getQuantity("geo dist")->setEnabled(true);

        // psMesh_bump->removeQuantity("Vector field");
        psMesh_bump->addVertexVectorQuantity("Vector field", vf);
        psMesh_bump->getQuantity("Vector field")->setEnabled(true);
    }
}

void preComputeDiffeomorphism(std::vector<geometrycentral::surface::VertexData<Vector3>>& pushForwardVectorField, std::vector<geometrycentral::surface::VertexData<double>>& geoDistance)
{
    try {
        const double delta_mesh = 1.25 * g_bumpMeshParameters.domain_extent.x / (g_numOfVertCols - 1);
        const int Tdim = g_flowFieldParameters.numberOfTimeSteps;
        pushForwardVectorField.clear();
        pushForwardVectorField.resize(Tdim);
        geoDistance.clear();
        geoDistance.resize(Tdim);

        // local coordinate of vortex core
        g_geometry_bump->requireFaceTangentBasis();
        g_geometry_bump->requireFaceNormals();
        geometrycentral::surface::VertexData<Vector3> vBasisX {};
        geometrycentral::surface::VertexData<Vector3> vBasisY {};
        geometrycentral::surface::VertexData<Vector3> normalTangetPlane {};
        getOrCreateBumpTangentBasis(vBasisX, vBasisY, normalTangetPlane);
        g_geometry_bump->requireVertexTangentBasis();
        g_geometry_bump->requireVertexNormals();
        std::vector<geometrycentral::surface::GeodesicAlgorithmExact> g_mmpOfTs;
        for (int t = 0; t < Tdim; t++) {
            g_mmpOfTs.emplace_back(*g_mesh_bump, *g_geometry_bump);
        }

        for (int t = 0; t < Tdim; t++) {
            geometrycentral::surface::VertexData<Vector3> vectorField(*g_mesh_bump);
            double m_time = g_flowFieldParameters.tMin + g_flowFieldParameters.getTimeStep() * t;
            auto vortex_core2D = getVortexCenter(m_time);

            const auto vortex_core_barycentric = getBarycentricCoordinate(vortex_core2D, *g_mesh_bump, *g_geometry_bump);
            const geometrycentral::surface::Face f = g_mesh_bump->face(vortex_core_barycentric.faceId);
            const geometrycentral::surface::Face faceOnPlane = g_mesh_plane->face(vortex_core_barycentric.faceId);
            const Vector3 fBary = { vortex_core_barycentric.u, vortex_core_barycentric.v, vortex_core_barycentric.w };
            const auto vortex_core_Onplane = geometrycentral::surface::SurfacePoint(faceOnPlane, fBary);
            const auto vortex_core_OnBump = geometrycentral::surface::SurfacePoint(f, fBary);
            const auto vortex_core_embedding = get3DEmbeddingPosition(vortex_core_OnBump, *g_geometry_bump);
            auto& g_mmpOfT = g_mmpOfTs[t];
            g_mmpOfT.propagate(vortex_core_OnBump);
            geometrycentral::surface::VertexData<double> distToSource = g_mmpOfT.getDistanceFunction();
            geometrycentral::surface::VertexData<Vector3> polarBasis_r_bump {};
            geometrycentral::surface::VertexData<Vector3> polarBasis_theta_bump {};
            geometrycentral::surface::VertexData<Vector3> direction(*g_mesh_bump);
            geometrycentral::surface::VertexData<double> angles(*g_mesh_bump);

            std::cout << "precompute push forward for time slice " << t << ".\n";
            const int Nv = g_mesh_bump->nVertices();
            for (int i = 0; i < Nv; i += 1) {
                geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
                const Vector3 n = g_geometry_bump->vertexNormals[v_bump];
                const Vector3 destinationPoint = g_geometry_bump->vertexPositions[v_bump];
                const double geoDistance0 = distToSource[v_bump];

                auto BackTrackVec = backTrackGeodesicDirection(vortex_core_embedding, &g_mmpOfT, v_bump, n);
                direction[v_bump] = BackTrackVec;
                const double cosA = (dot(BackTrackVec, vBasisX[v_bump])) / (vBasisX[v_bump].norm() * BackTrackVec.norm());
                double angle = (acos(cosA));
                if (BackTrackVec.y < 0)
                    angle = 2 * M_PI - angle;
                angles[v_bump] = angle;
            }
            CreatePolarCoordinateBasisOnBump1(vortex_core_OnBump, distToSource, direction, polarBasis_r_bump, polarBasis_theta_bump);
            for (int i = 0; i < Nv; i += 1) {
                geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(i);
                double angle = angles[v_bump];
                double geoDistance0 = distToSource[v_bump];
                Vector3 v_bumpCorrespondingPoint_Plane = embeddingPolarCoordinateOnPlane(vortex_core2D, angle, geoDistance0);
                Vector2 vortexfieldVec = getVelocityOneVortexFromPos(Vector2 { v_bumpCorrespondingPoint_Plane.x, v_bumpCorrespondingPoint_Plane.y }, m_time);
                // get polar basis on plane
                Vector2 r = Vector2 { v_bumpCorrespondingPoint_Plane.x, v_bumpCorrespondingPoint_Plane.y } - vortex_core2D;
                auto this_basis_r = normalize(r);
                Vector2 this_theta_basis = (this_basis_r.rotate90()) * r.norm();
                auto uv = getUVCoordinate(this_basis_r, this_theta_basis, vortexfieldVec);
                auto pushforwardedVector = uv(0) * polarBasis_r_bump[v_bump] + uv(1) * polarBasis_theta_bump[v_bump];
                vectorField[v_bump] = pushforwardedVector;
            }

            pushForwardVectorField[t] = std::move(vectorField);
            geoDistance[t] = std::move(distToSource);
        }
    } catch (const std::exception& exception) {
        std::cerr << exception.what();
    }
}

}

void updateFlowField(double time)
{

    geometrycentral::surface::VertexData<Vector3> vel_bump(*g_mesh_bump);
    geometrycentral::surface::VertexData<Vector3> vel_plane(*g_mesh_plane);

    if (g_onBump) {
        computeFlowFieldOnBump(time, vel_bump, vel_plane);
    } else {
        computeFlowFieldOnPlane(time, vel_plane);
    }

    updateFlowOnBumpFromCache(time, g_pushForwardVectorField, g_geoDistance);

    // psMesh_plane->removeQuantity("V");
    psMesh_plane->addVertexIntrinsicVectorQuantity("V", vel_plane);
    psMesh_plane->getQuantity("V")->setEnabled(true);
}

void exportTestCase()
{

    Eigen::VectorXd data_bump(g_numOfVertCols * g_numOfVertRows * g_flowFieldParameters.numberOfTimeSteps * 2);
    Eigen::VectorXd data_plane(g_numOfVertCols * g_numOfVertRows * g_flowFieldParameters.numberOfTimeSteps * 2);

    double dt = g_flowFieldParameters.getTimeStep();

    geometrycentral::surface::VertexData<Vector3> vel_plane(*g_mesh_plane);
    std::cout << "computing flow field for export" << std::endl;
    for (unsigned int t = 0; t < g_flowFieldParameters.numberOfTimeSteps; t++) {
        double exporttime = g_flowFieldParameters.tMin + (double)t * dt;
        std::cout << ".";

        computeFlowFieldOnPlane(exporttime, vel_plane);

        for (unsigned int r = 0; r < g_numOfVertRows; r++) {
            for (unsigned int c = 0; c < g_numOfVertCols; c++) {
                unsigned int lin_vert_index = r * g_numOfVertCols + c;
                Vector3 vec3d_bump = g_pushForwardVectorField[t][lin_vert_index];
                Vector3 vec3d_plane = vel_plane[lin_vert_index];

                unsigned int lin_vector_component_index = (t * g_numOfVertRows * g_numOfVertCols + lin_vert_index) * 2;
                data_bump(lin_vector_component_index) = vec3d_bump.x;
                data_bump(lin_vector_component_index + 1) = vec3d_bump.y;

                data_plane(lin_vector_component_index) = vec3d_plane.x;
                data_plane(lin_vector_component_index + 1) = vec3d_plane.y;
            }
        }
    }
    std::cout << "done." << std::endl;

    auto write_binary = [](const std::string& filename, const Eigen::VectorXd& vec) {
        std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
        if (!out.is_open()) {
            std::cout << "something went wrong opening file " + std::string(filename) + " for writing" << std::endl;
            return;
        }
        unsigned int size = vec.size();
        out.write((char*)vec.data(), size * sizeof(double));
        out.close();
    };

    auto write_header = [](const std::string& filename) {
        std::ofstream out(filename.c_str(), std::ios::out);
        if (!out.is_open()) {
            std::cout << "something went wrong opening file " + std::string(filename) + " for writing" << std::endl;
            return;
        }

        out << "bumpMeshParameters:";
        out << "\n";

        out << "g_numOfVertCols";
        out << "\n";
        out << g_numOfVertCols;
        out << "\n";
        out << "g_numOfVertRows";
        out << "\n";
        out << g_numOfVertRows;
        out << "\n";

        out << "g_bumpMeshParameters.domain_extent.x";
        out << "\n";
        out << g_bumpMeshParameters.domain_extent.x;
        out << "\n";
        out << "g_bumpMeshParameters.domain_extent.y";
        out << "\n";
        out << g_bumpMeshParameters.domain_extent.y;
        out << "\n";
        out << "g_bumpMeshParameters.bump_center.x";
        out << "\n";
        out << g_bumpMeshParameters.bump_center.x;
        out << "\n";
        out << "g_bumpMeshParameters.bump_center.y";
        out << "\n";
        out << g_bumpMeshParameters.bump_center.y;
        out << "\n";
        out << "g_bumpMeshParameters.bump_sigma";
        out << "\n";
        out << g_bumpMeshParameters.bump_sigma;
        out << "\n";
        out << "\n";

        out << "g_flowFieldParameters";
        out << "\n";
        out << "g_flowFieldParameters.tMin";
        out << "\n";
        out << g_flowFieldParameters.tMin;
        out << "\n";
        out << "g_flowFieldParameters.tMax";
        out << "\n";
        out << g_flowFieldParameters.tMax;
        out << "\n";
        out << "g_flowFieldParameters.numberOfTimeSteps";
        out << "\n";
        out << g_flowFieldParameters.numberOfTimeSteps;
        out << "\n";

        out << "g_flowFieldParameters.version";
        out << "\n";
        out << g_flowFieldParameters.version;
        out << "\n";
        out << "g_flowFieldParameters.maxVelocity";
        out << "\n";
        out << g_flowFieldParameters.maxVelocity;
        out << "\n";
        out << "g_flowFieldParameters.extent";
        out << "\n";
        out << g_flowFieldParameters.extent;
        out << "\n";
        out << "g_flowFieldParameters.vortex_center_start.x";
        out << "\n";
        out << g_flowFieldParameters.vortex_center_start.x;
        out << "\n";
        out << "g_flowFieldParameters.vortex_center_start.y";
        out << "\n";
        out << g_flowFieldParameters.vortex_center_start.y;
        out << "\n";
        out << "g_flowFieldParameters.vortex_center_end.x";
        out << "\n";
        out << g_flowFieldParameters.vortex_center_end.x;
        out << "\n";
        out << "g_flowFieldParameters.vortex_center_end.y";
        out << "\n";
        out << g_flowFieldParameters.vortex_center_end.y;
        out << "\n";

        out.close();
    };

    std::string filename_bump = g_flowFieldParameters.name + "_bump_datax.raw";
    std::string filename_plane = g_flowFieldParameters.name + "_plane_datax.raw";

    std::string headerFilename = g_flowFieldParameters.name + "_info.txt";

    write_binary(filename_bump, data_bump);
    write_binary(filename_plane, data_plane);
    write_header(headerFilename);
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback()
{
    ImGui::SliderScalar("flow extent", ImGuiDataType_Double, &(g_flowFieldParameters.extent), &extentMin, &extentMax, "%f", ImGuiSliderFlags_None);
    ImGui::SliderScalar("flow max velocity", ImGuiDataType_Double, &(g_flowFieldParameters.maxVelocity), &doubleMin, &doubleMax, "%f", ImGuiSliderFlags_None);
    ImGui::SliderScalar("flow tmin", ImGuiDataType_Double, &(g_flowFieldParameters.tMin), &doubleMin, &doubleMax, "%f", ImGuiSliderFlags_None);
    ImGui::SliderScalar("flow tmax", ImGuiDataType_Double, &(g_flowFieldParameters.tMax), &doubleMin, &doubleMax, "%f", ImGuiSliderFlags_None);
    ImGui::SliderScalar("time", ImGuiDataType_Double, &g_time, &g_flowFieldParameters.tMin, &g_flowFieldParameters.tMax, "%f", ImGuiSliderFlags_None);
    ImGui::SliderInt("selected t idx", &g_integer_t_debug, 0, g_flowFieldParameters.numberOfTimeSteps - 1);
    ImGui::SliderInt("selected row", &g_selectionRow, 0, g_numOfVertRows - 1);
    ImGui::SliderInt("selected column", &g_selectionCol, 0, g_numOfVertCols - 1);
    ImGui::Checkbox("animate", &g_Animate);
    ImGui::Checkbox("flow on bump", &g_onBump);
    if (ImGui::Button("geodesic demo")) {
        demoGeodesicCurve();
    }
    if (ImGui::Button("preComputeDiffeomorphism")) {
        preComputeDiffeomorphism(g_pushForwardVectorField, g_geoDistance);
    }
    if (ImGui::Button("compute flow field")) {
        updateFlowField(g_time);
    }
    if (ImGui::Button("export data")) {
        exportTestCase();
    }
    if (g_Animate) {
        double dt = g_flowFieldParameters.getTimeStep();
        g_time += dt;
        if (g_time > g_flowFieldParameters.tMax) {
            g_time = g_flowFieldParameters.tMin;
        }
        updateFlowField(g_time);
    }

    unsigned int selectionIndex = g_selectionRow * g_numOfVertCols + g_selectionCol;
    geometrycentral::surface::Vertex& v_bump = g_mesh_bump->vertex(selectionIndex);
    if (selectionIndex != v_bump.getIndex())
        std::cout << "unexpected index " << std::endl;
    Vector3 v_pos = g_geometry_bump->inputVertexPositions[selectionIndex];
    /*   std::vector<glm::vec3> points;
       points.push_back(glm::vec3(v_pos.x, v_pos.y, v_pos.z));

       Vector2 center = getVortexCenter(g_time);
       Vector2 d = center - g_bumpMeshParameters.bump_center;
       double dist_squared = d.norm2();
       double val = 1.0 / (g_bumpMeshParameters.bump_sigma * sqrt(2.0 * geometrycentral::PI)) * exp(-0.5 * dist_squared / (g_bumpMeshParameters.bump_sigma * g_bumpMeshParameters.bump_sigma));

       points.push_back(glm::vec3(center.x, center.y, val));
   */
}

int main(int argc, char** argv)
{
    g_flowFieldParameters.numberOfTimeSteps = g_numOfTimeStep;
    // Initialize polyscope
    polyscope::init();
    polyscope::view::upDir = polyscope::UpDir::ZUp;
    // Set the callback function
    polyscope::state::userCallback = myCallback;

    // build bump mesh
    std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> simpleMeshBump = makeGaussianBumpMesh(g_bumpMeshParameters.domain_extent, g_bumpMeshParameters.bump_center, g_bumpMeshParameters.bump_sigma, g_numOfVertRows, g_numOfVertCols);
    auto MeshBump = geometrycentral::surface::makeManifoldSurfaceMeshAndGeometry(simpleMeshBump->polygons, simpleMeshBump->vertexCoordinates);
    g_mesh_bump = std::move(std::get<0>(MeshBump));
    g_geometry_bump = std::move(std::get<1>(MeshBump));
    psMesh_bump = polyscope::registerSurfaceMesh("bump", g_geometry_bump->inputVertexPositions, g_mesh_bump->getFaceVertexList(), polyscopePermutations(*g_mesh_bump));

    g_mmp = std::make_unique<geometrycentral::surface::GeodesicAlgorithmExact>(*g_mesh_bump, *g_geometry_bump);

    // build plane mesh
    std::unique_ptr<geometrycentral::surface::SimplePolygonMesh> simpleMeshPlane = makePlaneMesh(g_bumpMeshParameters.domain_extent, g_numOfVertRows, g_numOfVertCols);
    auto planeMesh = geometrycentral::surface::makeManifoldSurfaceMeshAndGeometry(simpleMeshPlane->polygons, simpleMeshPlane->vertexCoordinates);

    g_mesh_plane = std::get<0>(planeMesh).get();
    g_geometry_plane = std::get<1>(planeMesh).get();

    psMesh_plane = polyscope::registerSurfaceMesh("plane", g_geometry_plane->inputVertexPositions, g_mesh_plane->getFaceVertexList(), polyscopePermutations(*std::get<0>(planeMesh)));
    psMesh_plane->translate(glm::vec3(0, 0, 5));
    // Set vertex tangent spaces
    g_geometry_plane->requireVertexTangentBasis();

    // add basis vectors
    geometrycentral::surface::VertexData<Vector3> vBasisX_plane(*g_mesh_plane);
    geometrycentral::surface::VertexData<Vector3> vBasisY_plane(*g_mesh_plane);

    for (geometrycentral::surface::Vertex v : g_mesh_plane->vertices()) {
        // vBasisX_plane[v] = g_geometry_plane->vertexTangentBasis[v][0];
        // vBasisY_plane[v] = g_geometry_plane->vertexTangentBasis[v][1];

        vBasisX_plane[v] = Vector3 { 1, 0, 0 };
        vBasisY_plane[v] = Vector3 { 0, 1, 0 };
    }

    psMesh_plane->setVertexTangentBasisX(vBasisX_plane);
    psMesh_plane->addVertexIntrinsicVectorQuantity("X1", vBasisX_plane);
    psMesh_plane->addVertexIntrinsicVectorQuantity("X2", vBasisY_plane);
    Vertex v = g_mesh_plane->vertex(0);
    Vector2 traceVec = 3 * Vector2::fromAngle(M_PI / 6);
    SurfacePoint pathEndpoint = traceGeodesic(*g_geometry_plane, SurfacePoint(v), traceVec).endPoint;

    //
    /* TODO
    VectorHeatMethodSolver vhmSolver(*g_geometry);
    SurfacePoint sourceP(0,0);
    VertexData<Vector2> vdata = vhmSolver.computeLogMap(sourceP);*/

    // compute parameterization
    // VertexData<Vector2> parameterization = geometrycentral::surface::parameterizeBFF(*g_mesh, *g_geometry);
    // psMesh2->addVertexIntrinsicVectorQuantity("parameterization", parameterization);

    updateFlowField(g_flowFieldParameters.tMin);

    // polyscope::state::userCallback = doWork;

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}
