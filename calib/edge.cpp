#include "edge.hpp"

namespace g2o
{

EdgeSim3ProjectXYZForCalibr::EdgeSim3ProjectXYZForCalibr(Matrix3d R1w, Vector3d t1w):
    BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
{
    mR1w = R1w;
    mt1w = t1w;
}

bool EdgeSim3ProjectXYZForCalibr::read(std::istream& is)
{
    for (int i=0; i<2; i++)
    {
        is >> _measurement[i];
    }

    for (int i=0; i<2; i++){
        for (int j=i; j<2; j++){
            is >> information()(i,j);
            if (i!=j)   information()(j,i) = information()(i,j);
        }
    }
    return true;
}

bool EdgeSim3ProjectXYZForCalibr::write(std::ostream& os) const
{
    for (int i=0; i<2; i++){
        os  << _measurement[i] << " ";
    }

    for (int i=0; i<2; i++){
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    }
    return os.good();
}

/**InverseSim3ProjectXYZ*/

EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() :
    BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
{
}

bool EdgeInverseSim3ProjectXYZ::read(std::istream& is)
{
    for (int i=0; i<2; i++)
    {
        is >> _measurement[i];
    }

    for (int i=0; i<2; i++){
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)   information()(j,i) = information()(i,j);
        }
    }
    return true;
}

bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const
{
    for (int i=0; i<2; i++){
        os << _measurement[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }

    return os.good();
}

/**InverseSim3ProjectXYZForCalibr*/

EdgeInverseSim3ProjectXYZForCalibr::EdgeInverseSim3ProjectXYZForCalibr(Matrix3d R2w, Vector3d t2w) :
BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
{
    mR2w = R2w;
    mt2w = t2w;
} 

bool EdgeInverseSim3ProjectXYZForCalibr::read(std::istream& is)
{
    for (int i=0; i<2; i++)
    {
        is >> _measurement[i];
    }

    for (int i=0; i<2; i++){
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)   information()(j,i) = information()(i,j);
        }
    }
    return true;
}

bool EdgeInverseSim3ProjectXYZForCalibr::write(std::ostream& os) const
{
    for (int i=0; i<2; i++){
        os << _measurement[i] << " ";
    }

    for (int i=0; i<2; i++){
        for (int j=i; j<2; j++){
            os << " " << information()(i,j);
        }
    }
    return os.good();
}

} // namespace g2o
