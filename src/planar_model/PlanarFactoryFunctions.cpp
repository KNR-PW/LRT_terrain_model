
#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace terrain_model
{
  using namespace ocs2;
  TerrainPlane computeTerrainPlane(const vector3_t& position, 
    const vector3_t& eulerAngles, ocs2::scalar_t height)
  {
    const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerAngles);
    const vector3_t planePosition = position - rotationMatrix.col(2) * height; // normal vector in world times height

    return TerrainPlane(planePosition, rotationMatrix.transpose());
  }
  
  TerrainPlane computeTerrainPlane(const std::vector<vector3_t>& positions)
  {
    size_t pointNum = positions.size();

    if(pointNum < 3)
    {
      throw std::invalid_argument("There should be more than 3 points for creating plane using"
        "fit method");
    }
    else if (pointNum == 3)
    {

      vector3_t vectorOnPlane1 = positions[1] - positions[0];
      vector3_t vectorOnPlane2 = positions[2] - positions[0];

      vector3_t normalToPlane = (vectorOnPlane1).cross(vectorOnPlane2);

      normalToPlane.normalize();
      vectorOnPlane1.normalize();

      /** X axis is vectorOnPlane1, 
       *  Y axis will be  normalToPlane x vectorOnPlane1
       *  Z asix is normalToPlane */

      matrix3_t rotationMatrix;
      rotationMatrix.col(0) = normalToPlane.cross(vectorOnPlane1);

      if(normalToPlane.z() < 0)
      {
        rotationMatrix.col(0) = -vectorOnPlane1;
        rotationMatrix.col(2) = -normalToPlane;
      }
      else
      {
        rotationMatrix.col(0) = vectorOnPlane1;
        rotationMatrix.col(2) = normalToPlane;
      }

      return TerrainPlane(positions[0], rotationMatrix);
      
    }
    else
    {
      ocs2::matrix_t A = ocs2::matrix_t::Ones(pointNum, 3);
      ocs2::vector_t b(pointNum);

      for(size_t i = 0; i < pointNum; ++i)
      {
        A.block<1, 2>(i, 0) = positions[i].block<2, 1>(i, 0).transpose();
        b(i) = positions[i].z();
      }

      ocs2::matrix_t AtA = A.transpose() * A;
      vector3_t result = AtA.ldlt().solve(A * b);
      vector3_t normalToSurface{-result.x(), -result.y(), 1};

      normalToSurface.normalize();

      /** Get random vector on plane from plane equation */
      vector3_t pointOnPlane1{positions[0].x(), positions[0].y(), 
        result.x() * positions[0].x() + result.y() * positions[0].y() + result.z()};

      vector3_t pointOnPlane2{positions[1].x(), positions[1].y(), 
        result.x() * positions[1].x() + result.y() * positions[1].y() + result.z()};
      
      vector3_t vectorOnPlane = pointOnPlane1 - pointOnPlane2;
      vectorOnPlane.normalize();

      matrix3_t rotationMatrix;
      rotationMatrix.col(0) = vectorOnPlane;
      rotationMatrix.col(0) = normalToSurface.cross(vectorOnPlane);
      rotationMatrix.col(0) = normalToSurface;

      return TerrainPlane(pointOnPlane1, rotationMatrix);

    }
  }
} // namespace terrain_model
