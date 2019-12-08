#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // 3D configuration map for closed list

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
        if (closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
                    current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)].utime != -1){
            // std::cout << "current has been explored" << std::endl;
            continue;
        } 

        // insert current node to closed list
        closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
                    current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)] = current_node.parent;

        // Goal check
        if (sqrt((current_node.self.x - goal.x)*(current_node.self.x - goal.x)+ \
            (current_node.self.y - goal.y)*(current_node.self.y - goal.y)) < distances.metersPerCell())
        {         
            std::cout << "reach goal" << std::endl;
            path.path.push_back(goal);

            // parent node
            pose_xyt_t parent;
            pose_xyt_t current = current_node.self;

            // back trace to start
            while (sqrt((current.x - start.x)*(current.x - start.x)+ \
                    (current.y - start.y)*(current.y - start.y)) > 0.001)
            {
                parent = closed_list[int(-current.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int(current.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)];
                parent.theta = atan2(current.y - parent.y, current.x - parent.x);
                path.path.push_back(parent);
                current = parent;
            }

            std::reverse(path.path.begin(),path.path.end());
            break;
        }


        // expand current node
        for(float x = current_node.self.x - distances.metersPerCell();x <= current_node.self.x + distances.metersPerCell(); x=x+distances.metersPerCell())
        {
            for(float y = current_node.self.y - distances.metersPerCell();y <= current_node.self.y + distances.metersPerCell(); y=y+distances.metersPerCell())
            {
                // check if the child is current node
                if (sqrt((current_node.self.x - x)*(current_node.self.x - x)+ \
                         (current_node.self.y - y)*(current_node.self.y - y) < 0.001)){
                            // std::cout << "child is current node" << std::endl;
                            continue;
                         } 

                // check if the child is in grid
                if (!distances.isCellInGrid(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
                    int(-y/distances.metersPerCell()+distances.heightInCells()/2))){
                        // std::cout << "child is not in grid" << std::endl;
                        continue;
                    } 

                // check if the child is an obstacle
                float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
                    int(-y/distances.metersPerCell()+distances.heightInCells()/2));
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

                // obs_cost = (obsDistance < params.maxDistanceWithCost) ? \
                //             (obsDistance + std::pow(params.maxDistanceWithCost - obsDistance, params.distanceCostExponent)) : obsDistance;
                // child.f += obs_cost;

                // std::cout << "add child" << std::endl;
                open_list.push(child);
            }
        } 
    }
    
    
    path.utime = start.utime;    
    path.path_length = path.path.size();
    std::cout << "length of A* path: " << path.path_length << std::endl;
    return path;
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
