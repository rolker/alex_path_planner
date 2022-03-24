#ifndef SRC_COSTMAP2DMAP_H
#define SRC_COSTMAP2DMAP_H

#include "Map.h"
#include <costmap_2d/costmap_2d_ros.h>

class Costmap2DMap : public Map {
public:
    Costmap2DMap(std::shared_ptr<costmap_2d::Costmap2DROS> costmap);

    ~Costmap2DMap() override = default;

    bool isBlocked(double x, double y) const override;

    double resolution() const override;

private:
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
    unsigned char blocked_threshold_ = costmap_2d::LETHAL_OBSTACLE;
};


#endif
