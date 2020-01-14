#include <local_map/map_ray_caster.h>

namespace map_ray_caster
{

MapRayCaster::MapRayCaster(const int occupied_threshold) :
  occupied_threshold_(occupied_threshold)
{
}

/** Return the list of pixel indexes from map center to pixel at map border and given angle
 *
 * The Bresenham algorithm is used.
 *
 * @param[in] angle beam angle.
 * @param[in] nrow image height.
 * @param[in] ncol image width.
 *
 * @return The list of pixel indexes from map center to pixel at map border and given angle.
 */
const std::vector<size_t>& MapRayCaster::getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol, const double tolerance)
{

  raycast_lookup_.clear();

  std::vector<size_t> pts;

  // Twice the distance from map center to map corner.
  const double r = std::sqrt((double) nrow * nrow + ncol * ncol);
  // Start point, map center.
  // TODO: the sensor position (map origin)  may not be the map center
  int x0 = ncol / 2;
  int y0 = nrow / 2;
  // End point, outside the map.
  int x1 = (int) round(x0 + r * std::cos(angle)); // Can be negative
  int y1 = (int) round(y0 + r * std::sin(angle));
  int dx = x1 - x0;
  int dy = y1 - y0;
  bool steep = (std::abs(dy) >= std::abs(dx));
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
    // recompute Dx, Dy after swap
    dx = x1 - x0;
    dy = y1 - y0;
  }
  int xstep = 1;
  if (dx < 0)
  {
    xstep = -1;
    dx = -dx;
  }
  int ystep = 1;
  if (dy < 0)
  {
    ystep = -1;
    dy = -dy;
  }
  int twoDy = 2 * dy;
  int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
  int e = twoDy - dx; //2*Dy - Dx
  int y = y0;
  int xDraw, yDraw;
  for (int x = x0; x != x1; x += xstep)
  {
    if (steep)
    {
      xDraw = y;
      yDraw = x;
    }
    else
    {
      xDraw = x;
      yDraw = y;
    }
    if (pointInMap(yDraw, xDraw, nrow, ncol))//如果点在图像中
    {
      pts.push_back(offsetFromRowCol(yDraw, xDraw, ncol));//存的是return (row * ncol) + col，存的是栅格编号;
    }
    else
    {
      // We exit when the first point outside the map is encountered.
      raycast_lookup_[angle] = pts;
      return raycast_lookup_[angle];//返回值
    }
    // next
    if (e > 0)
    {
      e += twoDyTwoDx; //E += 2*Dy - 2*Dx;
      y = y + ystep;
    }
    else
    {
      e += twoDy; //E += 2*Dy;
    }
  }
}

} // namespace map_ray_caster
