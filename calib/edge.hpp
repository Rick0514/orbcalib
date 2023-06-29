#pragma once

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace g2o
{
    
class EdgeSim3ProjectXYZForCalibr : public  BaseBinaryEdge<2, Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>
{
private:
    Matrix3d mR1w;
    Vector3d mt1w;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZForCalibr(Matrix3d R1w, Vector3d t1w);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

        Vector2d obs(_measurement);
        Vector3d P1 = mR1w*(v1->estimate().inverse().map(v2->estimate()))+mt1w;
        _error = obs-v1->cam_map1(project(P1));
    }

   // virtual void linearizeOplus();

};

class EdgeInverseSim3ProjectXYZForCalibr : public  BaseBinaryEdge<2, Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>
{
private:
    Matrix3d mR2w;
    Vector3d mt2w;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZForCalibr(Matrix3d R2w, Vector3d t2w);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

        Vector2d obs(_measurement);
        Vector3d P2 = mR2w*(v1->estimate().map(v2->estimate()))+mt2w;

        _error = obs-v1->cam_map2(project(P2));
    }

   // virtual void linearizeOplus();

};


} // namespace g2o