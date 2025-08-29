#include <terrain_model/TerrainModel.h>

namespace terrain_model
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane TerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t& positionInWorld) const
  {
    return getLocalTerrainAtPositionInWorldAlongGravity(positionInWorld,
      [](const vector3_t&) { return 0.0; });
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const
  {
    return getConvexTerrainAtPositionInWorld(positionInWorld, 
      [](const vector3_t&) { return 0.0; });
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld,
    std::function<scalar_t(const vector3_t&)>
    penaltyFunction) const
  {
    return {getLocalTerrainAtPositionInWorldAlongGravity(positionInWorld,
      std::move(penaltyFunction)), {}};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const SignedDistanceField* getSignedDistanceField() const 
  { 
    return nullptr; 
  }
} // namespace terrain_model