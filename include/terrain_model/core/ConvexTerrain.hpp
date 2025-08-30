//
// Created by rgrandia on 23.06.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __CONVEX_TERRAIN_TERRAIN_MODEL__
#define __CONVEX_TERRAIN_TERRAIN_MODEL__


#include <limits>
#include <utility>

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace terrain_model
{

		/**
  	 * Terrain defined ass plane with x-y bundy points
  	 * - The terrain frame is located at positionInWorld.
  	 * - The surface normal points in positive z direction in the terrain frame.
		 * - Boundry points are defined in the terrain frame
  	 */
    class ConvexTerrain
    {

			public:

				/** 
				 * Default constructor
				 */
    		ConvexTerrain();
			
				/** 
				 * Itemwise constructor
				 */
    		ConvexTerrain(TerrainPlane plane, std::vector<vector2_t> boundary);

				/** 
				 * Update terrain plane
				 * @param [in] newPlane: new terrain plane
				 */
				void updatePlane(TerrainPlane newPlane);

				/** 
				 * Update boundry points
				 * @param [in] getBoundryPoints: new boundry points
				 */
				void updateBoundryPoints(std::vector<vector2_t> getBoundryPoints);

				/** 
				 * Add new boundy points in terrain frame
				 * @param [in] newPointLocal: new point in terrain frame
				 */
				void addLocalBoundryPoint(const vector2_t& newPointLocal);

				/** 
				 * Add new boundy points in terrain frame
				 * @param [in] xLocal, yLocal: new point coordinates in terrain frame
				 */
				void addLocalBoundryPoint(ocs2::scalar_t xLocal, ocs2::scalar_t yLocal);

				/** 
				 * Add new boundy points in world frame
				 * @param [in] newPointWorld: new point in world frame
				 */
				void addWorldBoundryPoint(const vector2_t& newPointWorld);

				/** 
				 * Add new boundy points in world frame
				 * @param [in] xWorld, yWorld: new point coordinates in world frame
				 */
				void addWorldBoundryPoint(ocs2::scalar_t xWorld, ocs2::scalar_t yWorld);


				/**
    		 * Get boundry point with given input index (wrap around safe)
    		 * @param [in] currentPointIndex: index of queried point
    		 * @return Queried point
    		 */
				const vector2_t& getBoundryPoint(size_t currentPointIndex);

				/**
    		 * Get next boundry point with given input index (wrap around safe)
    		 * @param [in] currentPointIndex: index of queried point
    		 * @return Queried point next after queried index
    		 */
				const vector2_t& getNextBoundryPoint(size_t currentPointIndex);

				/**
    		 * Projects a 2D point into boundary of a 2D convex polygon.
    		 * @param [in] boundary: The vertices of the polygon in clockwise or counter-clockwise order.
    		 * @param [in] queryPoint: The 2D point.
    		 * @return A pair of signed squared distance to the boundary (negative inside, positive outside) and the projected point.
    		 */
    		std::pair<ocs2::scalar_t, vector2_t> projectToConvex2dPolygonBoundary(
					const vector2_t& queryPoint);

    		/**
    		 * Projects a 3D point onto a 3D convex polygon.
    		 * @param [in] convexTerrain: The 3D convex polygon.
    		 * @param [in] queryPoint: The 3D point.
    		 * @return The projected point.
    		 */
    		vector3_t projectToConvex3dPolygon(const vector3_t& queryPoint);

			private:

      	/** Plane coordinate defining the origin of the terrain */ 
      	TerrainPlane plane_;

      	/** 
				 * Boundary points x-y in the terrain frame, points are order counter-clockwise.
				 *  The last point is connected to the first point
				 */
      	std::vector<vector2_t> boundary_;
    };

    
} // namespace terrain_model

#endif
