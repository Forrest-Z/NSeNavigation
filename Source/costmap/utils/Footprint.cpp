#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include "Footprint.h"
#include "ArrayParser.h"
#include <DataSet/DataType/Point32.h>
#include <Console/Console.h>
#include "Math.h"

namespace NS_CostMap
{

  void calculateMinAndMaxDistances(
      const std::vector< sgbot::sensor::Point2D >& footprint, double& min_dist,
      double& max_dist)
  {
    min_dist = std::numeric_limits< double >::max();
    max_dist = 0.0;

    if(footprint.size() <= 2)
    {
      return;
    }

    for(unsigned int i = 0; i < footprint.size() - 1; ++i)
    {
      // check the distance from the robot center point to the first vertex
      double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
      double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x,
                                        footprint[i].y, footprint[i + 1].x,
                                        footprint[i + 1].y);
      min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
      max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    // we also need to do the last vertex and the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint.back().x,
                                  footprint.back().y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x,
                                      footprint.back().y, footprint.front().x,
                                      footprint.front().y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }



  void transformFootprint(
      double x, double y, double theta,
      const std::vector< sgbot::sensor::Point2D >& footprint_spec,
      std::vector< sgbot::sensor::Point2D >& oriented_footprint)
  {
    // build the oriented footprint at a given location
    oriented_footprint.clear();
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    for(unsigned int i = 0; i < footprint_spec.size(); ++i)
    {
      sgbot::sensor::Point2D new_pt;
      new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
      new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }
  }


  void padFootprint(std::vector< sgbot::sensor::Point2D >& footprint,
                    double padding)
  {
    // pad footprint in place
    for(unsigned int i = 0; i < footprint.size(); i++)
    {
      sgbot::sensor::Point2D& pt = footprint[i];
      pt.x += sign0(pt.x) * padding;
      pt.y += sign0(pt.y) * padding;
    }
  }

  std::vector< sgbot::sensor::Point2D > makeFootprintFromRadius(double radius)
  {
    std::vector < sgbot::sensor::Point2D > points;

    // Loop over 16 angles around a circle making a point each time
    int N = 16;
    sgbot::sensor::Point2D pt;
    for(int i = 0; i < N; ++i)
    {
      double angle = i * 2 * M_PI / N;
      pt.x = cos(angle) * radius;
      pt.y = sin(angle) * radius;
      printf("i = %d,x = %.4f,y = %.4f\n", i, pt.x, pt.y);
      points.push_back(pt);
    }

    return points;
  }

  bool makeFootprintFromString(const std::string& footprint_string,
                               std::vector< sgbot::sensor::Point2D >& footprint)
  {
    std::string error;
    std::vector < std::vector< float > > vvf = parseVVF(footprint_string,
                                                        error);

    if(error != "")
    {
      printf("Error parsing footprint parameter: '%s'\n", error.c_str());
      printf("  Footprint string was '%s'.\n", footprint_string.c_str());
      return false;
    }

    // convert vvf into points.
    if(vvf.size() < 3)
    {
      printf(
          "You must specify at least three points for the robot footprint, reverting to previous footprint.\n");
      return false;
    }
    footprint.reserve(vvf.size());
    for(unsigned int i = 0; i < vvf.size(); i++)
    {
      if(vvf[i].size() == 2)
      {
        sgbot::sensor::Point2D point;
        point.x = vvf[i][0];
        point.y = vvf[i][1];
        footprint.push_back(point);
      }
      else
      {
        printf(
            "Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.\n",
            int(vvf[i].size()));
        return false;
      }
    }

    return true;
  }

}  // end namespace costmap_2d
