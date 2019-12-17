#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // check goal position

    Point<double> global_origin_pose = distances.originInGlobalFrame();

    // std::cout << "global origin: " << global_origin_pose.x << " , " << global_origin_pose.y << std::endl;

    Point<double> goal_pos(goal.x,goal.y);
    Point<int> goal_idx = global_position_to_grid_cell(goal_pos, distances);

    // if (!distances.isCellInGrid(round(goal.x/distances.metersPerCell()+distances.widthInCells()/2), \
    //                 round(-goal.y/distances.metersPerCell()+distances.heightInCells()/2))){
    //                     std::cout << "goal is not in grid" << std::endl;
    //                 }
    
    if (!distances.isCellInGrid(goal_idx.x, goal_idx.y)){
                        std::cout << "goal is not in grid" << std::endl;
                    } 

    // check if the child is an obstacle
    // float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
    //     int(-y/distances.metersPerCell()+distances.heightInCells()/2));
    // float obsDistance = distances(round(goal.x/distances.metersPerCell()+distances.widthInCells()/2), \
    //     round(-goal.y/distances.metersPerCell()+distances.heightInCells()/2));
    float obsDistance = distances(goal_idx.x,goal_idx.y);
    if (obsDistance <= params.minDistanceToObstacle){
        std::cout << "goal too close to obstacle" << std::endl;
    } 


    // 2D configuration map for closed list

    pose_xyt_t initial;
    initial.utime = -1;
    std::vector<std::vector<pose_xyt_t>> closed_list(distances.heightInCells(),std::vector<pose_xyt_t>(distances.widthInCells(),initial));

    // open list
    // std::priority_queue<Node> open_list;

    std::priority_queue<Node,std::vector<Node>,compare_f> open_list;

    // add start node to openlist
    Node start_node;
    start_node.self = start;
    start_node.parent = start;
    start_node.g = 0;
    open_list.push(start_node);

    // create current node
    Node current_node;

    // create path
    robot_path_t path;

    // check if open list is empty
    while(!open_list.empty()){
        
        // std::cout << "in loop" << std::endl;

        // get current node in open list
        current_node = open_list.top();
        open_list.pop();

        // check if the current node has been explored
        // if (closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
        //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)].utime != -1){
        
        // convert global coordinate to cell index
        Point<double> current_node_pos(current_node.self.x,current_node.self.y);
        Point<int> current_node_idx = global_position_to_grid_cell(current_node_pos, distances);

        if (closed_list[current_node_idx.y][current_node_idx.x].utime != -1){
            // std::cout << "current has been explored" << std::endl;
            continue;
        }
        
        // if (closed_list[round(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2)][round( \
        //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2)].utime != -1){
        //     // std::cout << "current has been explored" << std::endl;
        //     continue;
        // } 

        // insert current node to closed list
        // closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
        //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)] = current_node.parent;
        closed_list[current_node_idx.y][current_node_idx.x] = current_node.parent;

        // Goal check
        if (sqrt((current_node.self.x - goal.x)*(current_node.self.x - goal.x)+ \
            (current_node.self.y - goal.y)*(current_node.self.y - goal.y)) < distances.metersPerCell())
        {         
            std::cout << "reach goal" << std::endl;
            path.path.push_back(goal);

            // parent node
            pose_xyt_t parent;
            pose_xyt_t current = current_node.self;

            std::cout << "back track start" << std::endl;

            // back trace to start
            while (sqrt((current.x - start.x)*(current.x - start.x)+ \
                    (current.y - start.y)*(current.y - start.y)) > 0.001)
            {
                // parent = closed_list[int(-current.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int(current.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)];
                // parent = closed_list[round(-current.y/distances.metersPerCell()+distances.heightInCells()/2)][round(current.x/distances.metersPerCell()+distances.widthInCells()/2)];
                
                Point<double> current_pos(current.x,current.y);
                Point<int> current_idx = global_position_to_grid_cell(current_pos, distances);
                parent = closed_list[current_idx.y][current_idx.x];

                parent.theta = atan2(current.y - parent.y, current.x - parent.x);
                path.path.push_back(parent);
                current = parent;
            }

            std::cout << "back track end" << std::endl;

            std::reverse(path.path.begin(),path.path.end());
            break;
        }


        // expand current node
        for(float x = current_node.self.x - distances.metersPerCell();x <= current_node.self.x + distances.metersPerCell(); x=x+distances.metersPerCell())
        {
            for(float y = current_node.self.y - distances.metersPerCell();y <= current_node.self.y + distances.metersPerCell(); y=y+distances.metersPerCell())
            {
                // convert global coordinate to cell index
                Point<double> pos(x,y);
                Point<int> idx = global_position_to_grid_cell(pos, distances);

                // check if the child is current node
                if (sqrt((current_node.self.x - x)*(current_node.self.x - x)+ \
                         (current_node.self.y - y)*(current_node.self.y - y) < 0.001)){
                            // std::cout << "child is current node" << std::endl;
                            continue;
                         } 

                // check if the child is in grid
                // if (!distances.isCellInGrid(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
                //     int(-y/distances.metersPerCell()+distances.heightInCells()/2))){
                if (!distances.isCellInGrid(idx.x,idx.y)){
                        // std::cout << "child is not in grid" << std::endl;
                        continue;
                    } 

                // check if the child is an obstacle
                // float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
                //     int(-y/distances.metersPerCell()+distances.heightInCells()/2));
                float obsDistance = distances(idx.x,idx.y);
                if (obsDistance <= params.minDistanceToObstacle){
                    // std::cout << "child is an obstacle" << std::endl;
                    continue;
                } 

                // // check if the child has been explored
                // if (closed_list[int(-y/distances.metersPerCell()+distances.heightInCells()/2)][int(x/distances.metersPerCell()+distances.widthInCells()/2)].utime != -1){
                //     // std::cout << "child has been explored" << std::endl;
                //     continue;
                // } 

                // add child to open list
                Node child;
                pose_xyt_t child_pose;
                child_pose.x = x;
                child_pose.y = y;

                child.self = child_pose;
                child.parent = current_node.self;
                child.g = current_node.g + sqrt((x-current_node.self.x)*(x-current_node.self.x)+(y-current_node.self.y)*(y-current_node.self.y));
                child.f = child.g + sqrt((x-goal.x)*(x-goal.x)+(y-goal.y)*(y-goal.y));

                // double obs_cost = (obsDistance < params.maxDistanceWithCost) ? \
                //             (obsDistance + std::pow(params.maxDistanceWithCost - obsDistance, params.distanceCostExponent)) : obsDistance;
                double obs_cost = (obsDistance < params.maxDistanceWithCost+0.05) ? 10*(params.maxDistanceWithCost+0.05-obsDistance) : 10*(obsDistance-(params.maxDistanceWithCost+0.05));
                child.f += obs_cost;

                // std::cout << "add child" << std::endl;
                open_list.push(child);
            }
        } 
    }
    
    
    path.utime = start.utime;    
    path.path_length = path.path.size();
    std::cout << "length of A* path: " << path.path_length << std::endl;

    // return path;



    // Simplify the astar path by skipping unnecessary poses which is in a line.

    // create a simplified path.
    robot_path_t simp_path;
    simp_path.path.push_back(path.path[0]);
    pose_xyt_t startPose = path.path[0];
    pose_xyt_t endPose;
    bool addPose = false;
    for (int i=2; i<path.path_length; i++){
        endPose = path.path[i];

        // check if the start and end has obstacle in between.
        addPose = isObstacleMiddle(startPose, endPose, distances, params);

        // If line between current pose and previous pose cross no obstacle, just skip this pose. 
        if (addPose){
            simp_path.path.push_back(path.path[i-1]);
            startPose = path.path[i-1];
        }
        
        // Add the last pose.
        if(i == path.path_length-1) {
            simp_path.path.push_back(path.path[i]);
        }
    }

    simp_path.path_length = simp_path.path.size();
    std::cout << "length of A* shortened path: " << simp_path.path_length << std::endl;


    return simp_path;
}

bool operator < (const Node& a, const Node& b) {
  return a.f > b.f;
}

void round_theta(float& theta)
{
    if(theta < 0){
        theta += 2 * M_PI;
    }
    if(theta > 2 * M_PI){
        theta -= 2 * M_PI;
    }
}


bool isObstacleMiddle(const pose_xyt_t& start, const pose_xyt_t& end, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    
    Point<int> start_idx = global_position_to_grid_cell(Point<double>(start.x,start.y), distances);
    Point<int> end_idx = global_position_to_grid_cell(Point<double>(end.x,end.y), distances);
    
    std::vector<Point<int>> bresenham_path = bresenham(start_idx, end_idx, distances);
    for (auto it = bresenham_path.cbegin(); it < bresenham_path.cend() - 1; ++it) {
        const Point<int> grid_cell = *it;

        // Check if there is obstacle in the middle.
        float obsDistance = distances(grid_cell.x,grid_cell.y);
        if (obsDistance <= params.minDistanceToObstacle){
            return true;
        }

    }
    return false;
}


static std::vector<Point<int>> bresenham(const Point<int>& start, const Point<int>& end, const ObstacleDistanceGrid& distances)
{
    std::vector<Point<int>> coords;
    int dx = std::abs(end.x - start.x);
    int dy = std::abs(end.y - start.y);
    int sx = start.x < end.x ? 1 : -1;
    int sy = start.y < end.y ? 1 : -1;
    int err = dx - dy;
    Point<int> current = start;

    while (distances.isCellInGrid(current.x, current.y) && (current.x != end.x || current.y != end.y))
    {
        coords.push_back(current);
        int e2 = 2 * err;
        if (e2 >= -dy)
        {
            err -= dy;
            current.x += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            current.y += sy;
        }
    }
    // add the actual endpoint
    if (distances.isCellInGrid(current.x, current.y)) {
        coords.push_back(current);

    }
    return coords;
}