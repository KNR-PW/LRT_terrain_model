
#include <terrain_model/core/ConvexTerrain.hpp>

namespace terrain_model
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain::ConvexTerrain(): plane_(), boundary_() {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain::ConvexTerrain(TerrainPlane plane, std::vector<vector2_t> boundary):
    plane_(std::move(plane)), boundary_(std::move(boundary)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const TerrainPlane& ConvexTerrain::getTerrainPlane() const
  {
    return plane_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const std::vector<vector2_t>& ConvexTerrain::getBoundryPoints() const
  {
    return boundary_;
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const vector2_t& ConvexTerrain::getBoundryPoint(size_t currentPointIndex) const
  {
  	return boundary_[currentPointIndex % boundary_.size()]; // wrap around
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const vector2_t& ConvexTerrain::getNextBoundryPoint(size_t currentPointIndex) const
  {
  	return boundary_[(currentPointIndex + 1) % boundary_.size()]; // next point with wrap around
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<scalar_t, vector2_t> ConvexTerrain::projectToConvex2dPolygonBoundary(
		const vector2_t& queryPoint) const
  {
    vector2_t image = queryPoint;
    scalar_t dist2 = std::numeric_limits<scalar_t>::max();
    auto saveIfCloser = [&queryPoint, &dist2, &image](const vector2_t& q)
    {
        const scalar_t newDist2 = (queryPoint - q).squaredNorm();
        if (newDist2 < dist2)
        {
            dist2 = newDist2;
            image = q;
        }
    };
    bool isInside = true;
    for (size_t i = 0; i < boundary_.size(); i++)
    {
        const auto& p1 = getBoundryPoint(i);
        const auto& p2 = getNextBoundryPoint(i);
        const vector2_t p12 = p2 - p1;
        const scalar_t r = p12.dot(queryPoint - p1) / p12.squaredNorm();
        if (r > 1.0)
        {
            saveIfCloser(p2);
        }
        else if (r < 0.0)
        {
            saveIfCloser(p1);
            isInside = false; // the point is outside since the angle is obtuse
        }
        else
        {
            const vector2_t q = p1 + r * p12;
            saveIfCloser(q);
        }
    } // end of i loop
    return {(isInside ? -dist2 : dist2), image};
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t ConvexTerrain::projectToConvex3dPolygon(const vector3_t& queryPoint) const
  {
    const vector3_t local_p = plane_.getPositionInTerrainFrameFromPositionInWorld(queryPoint);
    const vector2_t local_2d_p(local_p.x(), local_p.y());
    const auto distance2ImagePair = projectToConvex2dPolygonBoundary(local_2d_p);
    vector3_t local_q;
    if (distance2ImagePair.first <= 0.0)
    {
        // the 2d local point is inside polygon
        local_q << local_2d_p, 0.0;
    }
    else
    {
        // the 2d local point is outside polygon
        local_q << distance2ImagePair.second, 0.0;
    }
    return plane_.getPositionInWorldFrameFromPositionInTerrain(local_q);
  }
} // namespace terrain_model