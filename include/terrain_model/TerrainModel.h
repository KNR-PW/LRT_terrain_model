//
// Created by rgrandia on 21.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __TERRAIN_MODEL_TERRAIN_MODEL__
#define __TERRAIN_MODEL_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.h>
#include <terrain_model/core/TerrainPlane.h>
#include <terrain_model/core/ConvexTerrain.h>

namespace terrain_model
{
  /**
   * This abstract class defines the interface for terrain models.
   */
  class TerrainModel
  {
    public:
      TerrainModel() = default;
      virtual ~TerrainModel() = default;
      TerrainModel(const TerrainModel&) = delete;
      TerrainModel& operator=(const TerrainModel&) = delete;

      /** Returns a linear approximation of the terrain at the query point projected along gravity onto the terrain  */
      TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(
        const vector3_t& positionInWorld) const;

      /** Penalty function needs to return values >= 0 */ 
      virtual TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(
        const vector3_t& positionInWorld,
        std::function<scalar_t(const vector3_t&)>
        penaltyFunction) const = 0;
      
      ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const;
      
      /** Penalty function needs to return values >= 0 */
      virtual ConvexTerrain getConvexTerrainAtPositionInWorld(
        const vector3_t& positionInWorld,
        std::function<scalar_t(const vector3_t&)>
        penaltyFunction) const;
      
      /** Returns the signed distance field for this terrain if one is available */
      virtual const SignedDistanceField* getSignedDistanceField() const;

      virtual vector3_t getHighestObstacleAlongLine(
        const vector3_t& position1InWorld,                                        
        const vector3_t& position2InWorld) const = 0;
      /**
       * Returns the height profiles between two points in world frame. Provided as a set of points {alpha, height}, where alpha in [0, 1] is
       * the progress along the line. position1InWorld -> alpha = 0, position2InWorld -> alpha = 1.
       * Height is the absolute height in world frame.
       */
      virtual std::vector<vector2_t> getHeightProfileAlongLine(
        const vector3_t& position1InWorld,
        const vector3_t& position2InWorld) const = 0;
  };
}; // namespace terrain_model

#endif