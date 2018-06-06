#include "MapGrid.h"

#include "../../../../costmap/costmap_2d/CostValues.h"
#include <Console/Console.h>

using namespace std;

namespace NS_Planner
{

  MapGrid::MapGrid()
      : size_x_(0), size_y_(0)
  {
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y)
      : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  MapGrid::MapGrid(const MapGrid& mg)
  {
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void MapGrid::commonInit()
  {
    //don't allow construction of zero size grid
    assert(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i)
    {
      for(unsigned int j = 0; j < size_x_; ++j)
      {
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
    times = 0;
  }

  size_t MapGrid::getIndex(int x, int y)
  {
    return size_x_ * y + x;
  }

  MapGrid&
  MapGrid::operator=(const MapGrid& mg)
  {
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y)
  {
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y)
    {
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i)
      {
        for(unsigned int j = 0; j < size_x_; ++j)
        {
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }

  inline bool MapGrid::updatePathCell(MapCell* current_cell,
                                      MapCell* check_cell,
                                      const NS_CostMap::Costmap2D& costmap)
  {

    //if the cell is an obstacle set the max path distance
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(!getCell(check_cell->cx, check_cell->cy).within_robot && (cost == NS_CostMap::LETHAL_OBSTACLE || cost == NS_CostMap::INSCRIBED_INFLATED_OBSTACLE || cost == NS_CostMap::NO_INFORMATION))
    {
      check_cell->target_dist = obstacleCosts();
      return false;
    }

    float new_target_dist = current_cell->target_dist + 1;
    if(new_target_dist < check_cell->target_dist)
    {
      check_cell->target_dist = new_target_dist;
    }
    return true;
  }

  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist()
  {
    for(unsigned int i = 0; i < map_.size(); ++i)
    {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }

  void MapGrid::adjustPlanResolution(
      const std::vector< Pose2D >& global_plan_in,
      std::vector< Pose2D >& global_plan_out,
      float resolution)
  {
    if(global_plan_in.size() == 0)
    {
      return;
    }
    float last_x = global_plan_in[0].x();
    float last_y = global_plan_in[0].y();
    global_plan_out.push_back(global_plan_in[0]);

    // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
    float min_sq_resolution = resolution * resolution * 4;

    for(unsigned int i = 1; i < global_plan_in.size(); ++i)
    {
      float loop_x = global_plan_in[i].x();
      float loop_y = global_plan_in[i].y();
      float sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
//      printf("sqdist = %.4f,resolution = %.4f,min_sq_resolution = %.4f\n",
//             sqdist, resolution, min_sq_resolution);
      if(sqdist > min_sq_resolution)
      {
        int steps = ((sqrt(sqdist) - sqrt(min_sq_resolution)) / resolution) - 1;
        // add a points in-between
        float deltax = (loop_x - last_x) / steps;
        float deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for(int j = 1; j < steps; ++j)
        {
          Pose2D pose;
          pose.x() = last_x + j * deltax;
          pose.y() = last_y + j * deltay;
          pose.theta() = (global_plan_in[i].theta());
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }

  //update what map cells are considered path based on the global_plan
  void MapGrid::setTargetCells(
      const NS_CostMap::Costmap2D& costmap,
      const std::vector< Pose2D >& global_plan)
  {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue< MapCell* > path_dist_queue;

    std::vector < Pose2D > adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan,
                         costmap.getResolution());

    if(adjusted_global_plan.size() != global_plan.size())
    {
      printf("Adjusted global plan resolution, added %zu points\n",
             adjusted_global_plan.size() - global_plan.size());
    }
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for(i = 0; i < adjusted_global_plan.size(); ++i)
    {
      float g_x = adjusted_global_plan[i].x();
      float g_y = adjusted_global_plan[i].y();
      unsigned int map_x, map_y;
      if(costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x,
                                                                       map_y) != NS_CostMap::NO_INFORMATION)
      {
        MapCell& current = getCell(map_x, map_y);
        current.target_dist = 0.0;
        current.target_mark = true;
        path_dist_queue.push(&current);
        started_path = true;
      }
      else if(started_path)
      {
        break;
      }
    }
    if(!started_path)
    {
      printf(
          "None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free\n",
          i, adjusted_global_plan.size(), global_plan.size());
      return;
    }

    computeTargetDistance(path_dist_queue, costmap);
  }

  //mark the point of the costmap as local goal where global_plan first leaves the area (or its last point)
  void MapGrid::setLocalGoal(
      const NS_CostMap::Costmap2D& costmap,
      const std::vector< Pose2D >& global_plan)
  {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector < Pose2D > adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan,
                         costmap.getResolution());

    // skip global path points until we reach the border of the local map
    for(unsigned int i = 0; i < adjusted_global_plan.size(); ++i)
    {
      float g_x = adjusted_global_plan[i].x();
      float g_y = adjusted_global_plan[i].y();
      unsigned int map_x, map_y;
      if(costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x,
                                                                       map_y) != NS_CostMap::NO_INFORMATION)
      {
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      }
      else
      {
        if(started_path)
        {
          break;
        } // else we might have a non pruned path, so we just continue
      }
    }
    if(!started_path)
    {
      printf(
          "None of the points of the global plan were in the local costmap, global plan points too far from robot\n");
      return;
    }

    queue< MapCell* > path_dist_queue;
    if(local_goal_x >= 0 && local_goal_y >= 0)
    {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }

    computeTargetDistance(path_dist_queue, costmap);
  }

  void MapGrid::computeTargetDistance(queue< MapCell* >& dist_queue,
                                      const NS_CostMap::Costmap2D& costmap)
  {
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty())
    {
      current_cell = dist_queue.front();

      dist_queue.pop();

      if(current_cell->cx > 0)
      {
        check_cell = current_cell - 1;
        if(!check_cell->target_mark)
        {
          //mark the cell as visisted
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap))
          {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cx < last_col)
      {
        check_cell = current_cell + 1;
        if(!check_cell->target_mark)
        {
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap))
          {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy > 0)
      {
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark)
        {
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap))
          {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy < last_row)
      {
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark)
        {
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap))
          {
            dist_queue.push(check_cell);
          }
        }
      }
    }
  }

}
;
