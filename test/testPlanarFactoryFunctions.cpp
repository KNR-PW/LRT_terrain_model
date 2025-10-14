#include <gtest/gtest.h>

#include "Rotations.hpp"
#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

using namespace ocs2;
using namespace terrain_model;

const size_t numTests = 1;
const scalar_t tolerance = 1e-6;

TEST(TestPlanarFactoryFunctions, positionAndRotation)
{
  for(size_t i = 0; i < numTests; ++i)
  {
    vector3_t eulerXYZ = vector3_t::Random();
    const vector3_t position = vector3_t::Random();
    const TerrainPlane truePlane = TerrainPlane(position, 
      getRotationMatrixFromZyxEulerAngles(eulerXYZ).transpose());
    const vector3_t surfaceNormal = truePlane.getSurfaceNormalInWorld();
  
    const scalar_t height = ((double) std::rand()) / RAND_MAX;

    const vector3_t truePositon = position - height * surfaceNormal;

    const TerrainPlane newPlane = computeTerrainPlane(position, 
      eulerXYZ, height);
  
    EXPECT_TRUE(newPlane.getPosition() == truePositon);
    EXPECT_TRUE(newPlane.getOrientationToTerrain() == truePlane.getOrientationToTerrain());
  }
}

TEST(TestPlanarFactoryFunctions, _2positions)
{
  for(size_t i = 0; i < numTests; ++i)
  {
    const scalar_t randomScalar = 10.0 * (((double) std::rand()) / RAND_MAX - 0.5);
    std::vector<vector3_t> positions;
    for(size_t j = 0;  j < 2; ++j)
    {
      positions.push_back(randomScalar * vector3_t::Random());
    }

    EXPECT_THROW(computeTerrainPlane(positions), std::invalid_argument);
  }
}

TEST(TestPlanarFactoryFunctions, _3positions)
{
  for(size_t i = 0; i < numTests; ++i)
  {
    const scalar_t randomScalar = 10.0 * (((double) std::rand()) / RAND_MAX - 0.5);
    std::vector<vector3_t> positions;
    for(size_t j = 0;  j < 3; ++j)
    {
      positions.push_back(randomScalar * vector3_t::Random());
    }

    const TerrainPlane plane = computeTerrainPlane(positions);

    for(size_t j = 0;  j < 3; ++j)
    {
      EXPECT_TRUE(plane.projectPositionInWorldOntoPlaneAlongGravity(
        positions[j]).isApprox(positions[j], tolerance));
    }
  }
}



TEST(TestPlanarFactoryFunctions, _4positions)
{
  for(size_t i = 0; i < numTests; ++i)
  {
    vector3_t eulerXYZ = vector3_t::Random();
    const vector3_t position = vector3_t::Random();
    const TerrainPlane truePlane = TerrainPlane(position, 
      getRotationMatrixFromZyxEulerAngles(eulerXYZ).transpose());
    const scalar_t randomScalar = 10.0 * (((double) std::rand()) / RAND_MAX - 0.5);
    std::vector<vector3_t> positions;
    for(size_t j = 0;  j < 4; ++j)
    {
      positions.push_back(randomScalar * vector3_t::Random());
      positions[j] = truePlane.projectPositionInWorldOntoPlaneAlongGravity(positions[j]);
    }

    const TerrainPlane plane = computeTerrainPlane(positions);

    for(size_t j = 0;  j < 4; ++j)
    {
      EXPECT_TRUE(plane.projectPositionInWorldOntoPlaneAlongGravity(
        positions[j]).isApprox(positions[j], tolerance));
    }
    EXPECT_TRUE(plane.getSurfaceNormalInWorld().isApprox(
      truePlane.getSurfaceNormalInWorld(), tolerance));
  }
}

TEST(TestPlanarFactoryFunctions, _6positions)
{
  for(size_t i = 0; i < numTests; ++i)
  {
    vector3_t eulerXYZ = vector3_t::Random();
    const vector3_t position = vector3_t::Random();
    const TerrainPlane truePlane = TerrainPlane(position, 
      getRotationMatrixFromZyxEulerAngles(eulerXYZ));
    const scalar_t randomScalar = 10.0 * (((double) std::rand()) / RAND_MAX - 0.5);
    std::vector<vector3_t> positions;
    for(size_t j = 0;  j < 6; ++j)
    {
      positions.push_back(randomScalar * vector3_t::Random());
      positions[j] = truePlane.projectPositionInWorldOntoPlaneAlongGravity(positions[j]);
    }

    const TerrainPlane plane = computeTerrainPlane(positions);

    for(size_t j = 0;  j < 6; ++j)
    {
      EXPECT_TRUE(plane.projectPositionInWorldOntoPlaneAlongGravity(
        positions[j]).isApprox(positions[j], tolerance));
    }
    EXPECT_TRUE(plane.getSurfaceNormalInWorld().isApprox(
      truePlane.getSurfaceNormalInWorld(), tolerance));
  }
}