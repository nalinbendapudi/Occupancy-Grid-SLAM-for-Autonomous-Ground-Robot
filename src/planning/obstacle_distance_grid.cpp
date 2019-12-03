#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <math.h>
#include <cmath>
#include <numeric>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map. 

    // std::cout << "\n\n\nmap size: " <<  map.heightInCells() << " " << map.widthInCells() << "\n\n\n" << std::endl;
    
    for(int y = 0; y < map.heightInCells(); ++y)
    {
        for(int x = 0; x < map.widthInCells(); ++x)
        {
            // Check current cell
            if(map.logOdds(x,y) >= 0){
                cells_[y*map.widthInCells()+x] = 0;
                continue;
            }
            
            // 1D map storing whether cells have been explored 
            std::vector<int> explored(map.widthInCells()*map.heightInCells(),0);

            // Mark current code as explored
            explored[y*map.widthInCells()+x] = 1; 

            int stop = 0;
            float length = 0;
            float minDistance = (map.heightInCells()+map.widthInCells())*metersPerCell_;

            while(!stop)
            {
                
                // increment the search radius
                length+=metersPerCell_;

                // loop through all cells in the square with length
                for(int y_search = y - int(length/metersPerCell_); y_search < y + int(length/metersPerCell_); y_search++)
                {
                    for(int x_search = x - int(length/metersPerCell_); x_search < x + int(length/metersPerCell_); x_search++)
                    {   
                        
                        // check if the cell is in grid
                        if (!map.isCellInGrid(x_search,y_search)){
                            // std::cout << "not cell in grid" << std::endl;
                            continue;
                        }
                        
                        // check if the cell is explored
                        if (explored[y_search*map.widthInCells()+x_search]){
                            // std::cout << "cell explored before" << std::endl;
                            continue;
                        }
                        
                        // check if the cell is inside the circle
                        float distance = metersPerCell_*sqrt(((x_search-x)*(x_search-x)+(y_search-y)*(y_search-y)));
                        if (distance > length){
                            // std::cout << "cell not in search circle" << std::endl;
                            continue;
                        }

                        // check if the cell is occupied 
                        if (map.logOdds(x_search,y_search) > 0){
                            if (distance < minDistance) minDistance = distance;
                        }
                        
                        explored[y_search*map.widthInCells()+x_search] = 1;
                        
                        // std::cout << "vector sum: " << std::accumulate(explored.begin(), explored.end(),0) << std::endl;        
                    }
                }

                // std::cout << "minDistance: " << minDistance << "vector sum: " << std::accumulate(explored.begin(), explored.end(),0) << std::endl;

                // If minDistance is renewed or all cells are explored
                if ((std::abs(minDistance-(map.heightInCells()+map.widthInCells())*metersPerCell_) > 0.01) || *std::min_element(explored.begin(), explored.end())) stop = 1;
            }

            // std::cout << " value of current cell: " << minDistance << std::endl;

            cells_[y*map.widthInCells()+x] = minDistance;
        }
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
