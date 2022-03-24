#include "Costmap2DMap.h"

Costmap2DMap::Costmap2DMap(std::shared_ptr<costmap_2d::Costmap2DROS> costmap):costmap_(costmap)
{
  auto c = costmap_->getCostmap();
  double x1 = c->getOriginX();
  double x2 = x1 + c->getSizeInCellsX()*c->getResolution();
  double y1 = c->getOriginY();
  double y2 = y2 + c->getSizeInCellsY()*c->getResolution();

  if(x1 < x2)
  {
    m_Extremes[0] = x1;
    m_Extremes[1] = x2;
  }
  else
  {
    m_Extremes[0] = x2;
    m_Extremes[1] = x1;
  }
  if(y1 < y2)
  {
    m_Extremes[2] = y1;
    m_Extremes[3] = y2;
  }
  else
  {
    m_Extremes[2] = y2;
    m_Extremes[3] = y1;
  }
}

double Costmap2DMap::resolution() const
{
  return costmap_->getCostmap()->getResolution();
}

bool Costmap2DMap::isBlocked(double x, double y) const
{
  auto c = costmap_->getCostmap();

  unsigned int mx, my;
  if(c->worldToMap(x, y, mx, my))
  {
    //ROS_INFO_STREAM("isBlocked? " << x << ", " << y << " cost: " << int(c->getCost(mx, my)));

    if(c->getCost(mx, my) < blocked_threshold_)
      return false;
  }
  //else
    //ROS_INFO_STREAM("isBlocked? " << x << ", " << y << " outside map bounds!");

  return true;
}
