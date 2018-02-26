#include "CostMapLayer.h"

namespace NS_CostMap
{
  CostmapLayer::CostmapLayer(): layered_costmap_(NULL), current_(false), enabled_(false){

  }
  void CostmapLayer::updateWithTrueOverwrite(Costmap2D& master_grid, int min_i,
                                             int min_j, int max_i, int max_j)
  {
    if(!enabled_)
      return;
    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    for(int j = min_j; j < max_j; j++)
    {
      unsigned int it = span * j + min_i;
      for(int i = min_i; i < max_i; i++)
      {
        master[it] = costmap_[it];
        it++;
      }
    }
  }

  void CostmapLayer::initialize(LayeredCostmap* parent)
   {
     layered_costmap_ = parent;
     onInitialize();
   }

   const std::vector< NS_DataType::Point >&
   CostmapLayer::getFootprint() const
   {
     return layered_costmap_->getFootprint();
   }

}  // namespace costmap_2d
