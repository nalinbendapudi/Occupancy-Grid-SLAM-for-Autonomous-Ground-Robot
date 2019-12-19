#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>


// robot_path_t search_for_path(pose_xyt_t start, 
//                              pose_xyt_t goal, 
//                              const ObstacleDistanceGrid& distances,
//                              const SearchParams& params)
// {
//     ////////////////// TODO: Implement your A* search here //////////////////////////
    
//     // check goal position

//     Point<double> global_origin_pose = distances.originInGlobalFrame();

//     // std::cout << "global origin: " << global_origin_pose.x << " , " << global_origin_pose.y << std::endl;

//     Point<double> goal_pos(goal.x,goal.y);
//     Point<int> goal_idx = global_position_to_grid_cell(goal_pos, distances);

//     std::cout << "goal_idx: " << goal_idx.x << "," << goal_idx.y << std::endl;

//     // if (!distances.isCellInGrid(round(goal.x/distances.metersPerCell()+distances.widthInCells()/2), \
//     //                 round(-goal.y/distances.metersPerCell()+distances.heightInCells()/2))){
//     //                     std::cout << "goal is not in grid" << std::endl;
//     //                 }
    
//     if (!distances.isCellInGrid(goal_idx.x, goal_idx.y)){
//                         std::cout << "goal is not in grid" << std::endl;
//                     } 

//     // check if the child is an obstacle
//     // float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
//     //     int(-y/distances.metersPerCell()+distances.heightInCells()/2));
//     // float obsDistance = distances(round(goal.x/distances.metersPerCell()+distances.widthInCells()/2), \
//     //     round(-goal.y/distances.metersPerCell()+distances.heightInCells()/2));
//     float obsDistance = distances(goal_idx.x,goal_idx.y);
//     if (obsDistance <= params.minDistanceToObstacle){
//         std::cout << "goal too close to obstacle" << std::endl;
//     }

//     std::cout << "start: " << start.x << "," << start.y << "," << start.utime << std::endl;
//     std::cout << "goal: " << goal.x << "," << goal.y << "," << goal.utime << std::endl;

//     // 2D configuration map for closed list

//     pose_xyt_t initial;
//     initial.utime = -1;
//     std::vector<std::vector<pose_xyt_t>> closed_list(distances.heightInCells(),std::vector<pose_xyt_t>(distances.widthInCells(),initial));

//     // open list
//     // std::priority_queue<Node> open_list;

//     std::priority_queue<Node,std::vector<Node>,compare_f> open_list;

//     // add start node to openlist
//     Node start_node;
//     start_node.self = start;
//     start_node.parent = start;
//     start_node.g = 0.0;
//     open_list.push(start_node);

//     // create current node
//     Node current_node;

//     // create path
//     robot_path_t path;

//     // check if open list is empty
//     while(!open_list.empty()){
        
//         // std::cout << "in loop" << std::endl;

//         // get current node in open list
//         current_node = open_list.top();
//         open_list.pop();
//         // std::cout << "openlist size: " << open_list.size() << std::endl;

//         // check if the current node has been explored
//         // if (closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
//         //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)].utime != -1){
        
//         // convert global coordinate to cell index
//         Point<double> current_node_pos(current_node.self.x,current_node.self.y);
//         Point<int> current_node_idx = global_position_to_grid_cell(current_node_pos, distances);

//         // std::cout << "world coordinate: " << current_node.self.x << "," << current_node.self.y << std::endl;

//         if (closed_list[current_node_idx.y][current_node_idx.x].utime != -1){
//             // std::cout << "current has been explored" << std::endl;
//             continue;
//         }

//         // std::cout << "cell coordinate: " << current_node_idx.x << "," << current_node_idx.y << std::endl;
        
//         // if (closed_list[round(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2)][round( \
//         //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2)].utime != -1){
//         //     // std::cout << "current has been explored" << std::endl;
//         //     continue;
//         // } 

//         // insert current node to closed list
//         // closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
//         //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)] = current_node.parent;
//         closed_list[current_node_idx.y][current_node_idx.x] = current_node.parent;

//         // Goal check
//         if (sqrt((current_node.self.x - goal.x)*(current_node.self.x - goal.x)+ \
//             (current_node.self.y - goal.y)*(current_node.self.y - goal.y)) < distances.metersPerCell())
//         {         
//             std::cout << "reach goal" << std::endl;
//             path.path.push_back(goal);

//             // parent node
//             pose_xyt_t parent;
//             pose_xyt_t current = current_node.self;

//             std::cout << "back track start" << std::endl;

//             // back trace to start
//             while (sqrt((current.x - start.x)*(current.x - start.x)+ \
//                     (current.y - start.y)*(current.y - start.y)) > 0.001)
//             {
//                 // parent = closed_list[int(-current.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int(current.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)];
//                 // parent = closed_list[round(-current.y/distances.metersPerCell()+distances.heightInCells()/2)][round(current.x/distances.metersPerCell()+distances.widthInCells()/2)];
                
//                 Point<double> current_pos(current.x,current.y);
//                 Point<int> current_idx = global_position_to_grid_cell(current_pos, distances);
//                 parent = closed_list[current_idx.y][current_idx.x];
                
//                 parent.theta = atan2(current.y - parent.y, current.x - parent.x);
//                 path.path.push_back(parent);
//                 current = parent;
//             }

//             std::cout << "back track end" << std::endl;

//             std::reverse(path.path.begin(),path.path.end());
//             break;
//         }


//         // expand current node
//         for(float x = current_node.self.x - distances.metersPerCell();x <= current_node.self.x + distances.metersPerCell(); x=x+distances.metersPerCell())
//         {
//             for(float y = current_node.self.y - distances.metersPerCell();y <= current_node.self.y + distances.metersPerCell(); y=y+distances.metersPerCell())
//             {
//                 // convert global coordinate to cell index
//                 Point<double> pos(x,y);
//                 Point<int> idx = global_position_to_grid_cell(pos, distances);

//                 // check if the child is current node
//                 if (sqrt((current_node.self.x - x)*(current_node.self.x - x)+ \
//                          (current_node.self.y - y)*(current_node.self.y - y) < 0.001)){
//                             // std::cout << "child is current node" << std::endl;
//                             continue;
//                          } 

//                 // check if the child is in grid
//                 // if (!distances.isCellInGrid(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
//                 //     int(-y/distances.metersPerCell()+distances.heightInCells()/2))){
//                 if (!distances.isCellInGrid(idx.x,idx.y)){
//                         // std::cout << "child is not in grid" << std::endl;
//                         continue;
//                     } 

//                 // check if the child is an obstacle
//                 // float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
//                 //     int(-y/distances.metersPerCell()+distances.heightInCells()/2));
//                 float obsDistance = distances(idx.x,idx.y);
//                 if (obsDistance <= params.minDistanceToObstacle){
//                     // std::cout << "child is an obstacle" << std::endl;
//                     continue;
//                 } 

//                 // // check if the child has been explored
//                 // if (closed_list[int(-y/distances.metersPerCell()+distances.heightInCells()/2)][int(x/distances.metersPerCell()+distances.widthInCells()/2)].utime != -1){
//                 //     // std::cout << "child has been explored" << std::endl;
//                 //     continue;
//                 // } 

//                 // add child to open list
//                 Node child;
//                 pose_xyt_t child_pose;
//                 child_pose.x = x;
//                 child_pose.y = y;

//                 child.self = child_pose;
//                 child.parent = current_node.self;
//                 child.g = current_node.g + sqrt((x-current_node.self.x)*(x-current_node.self.x)+(y-current_node.self.y)*(y-current_node.self.y));
                
//                 // std::cout << "child.g: " << child.g << std::endl;

//                 child.f = sqrt((x-goal.x)*(x-goal.x)+(y-goal.y)*(y-goal.y));
                
//                 double cost_from_start = child.g;
//                 child.f += cost_from_start;

//                 // child.f = child.g + sqrt((x-goal.x)*(x-goal.x)+(y-goal.y)*(y-goal.y));
                

//                 double obs_cost = (obsDistance < params.maxDistanceWithCost) ? \
//                             10.0*(params.maxDistanceWithCost - obsDistance) : 10.0*(obsDistance - params.maxDistanceWithCost);
//                 child.f += obs_cost;

//                 open_list.push(child);
//                 // std::cout << "add child" << std::endl;
//             }
//         } 
//     }

//     int finish = 1;
//     for(std::vector<std::vector<pose_xyt_t>>::iterator it1 = closed_list.begin();it1 != closed_list.end();it1++){
//         for(std::vector<pose_xyt_t>::iterator it2 = (*it1).begin();it2 != (*it1).end();it2++){        
//             if ((*it2).utime != -1)
//             {
//                 finish = 0;
//                 break;
//             }
//         }
//         if(finish == 0) break;
//     }

//     if(finish) std::cout << "all map explored" << std::endl;
//     else std::cout << "not all map explored" << std::endl;
    
//     path.utime = start.utime;    
//     path.path_length = path.path.size();
//     std::cout << "length of A* path: " << path.path_length << std::endl;
//     return path;
// }

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // check goal position

    // std::cout << "enter A*" << std::endl;

    // Point<double> global_origin_pose = distances.originInGlobalFrame();

    // std::cout << "global origin: " << global_origin_pose.x << " , " << global_origin_pose.y << std::endl;

    Point<int> start_idx = global_position_to_grid_cell(Point<double>(start.x,start.y), distances);
    Point<int> goal_idx = global_position_to_grid_cell(Point<double>(goal.x,goal.y), distances);

    // if (!distances.isCellInGrid(round(goal.x/distances.metersPerCell()+distances.widthInCells()/2), \
    //                 round(-goal.y/distances.metersPerCell()+distances.heightInCells()/2))){
    //                     std::cout << "goal is not in grid" << std::endl;
    //                 }

    // create path
    robot_path_t path;

    // std::cout << "here-1" << std::endl;
    
    if (!distances.isCellInGrid(start_idx.x, start_idx.y) || !distances.isCellInGrid(goal_idx.x, goal_idx.y)){
        path.utime = start.utime;    
        path.path_length = path.path.size();
        return path;                
        // std::cout << "goal is not in grid" << std::endl;
    } 

    // std::cout << "here0" << std::endl;

    // check if the child is an obstacle
    // float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
    //     int(-y/distances.metersPerCell()+distances.heightInCells()/2));
    // float obsDistance = distances(round(goal.x/distances.metersPerCell()+distances.widthInCells()/2), \
    //     round(-goal.y/distances.metersPerCell()+distances.heightInCells()/2));
    double obsDistance_start = distances(start_idx.x,start_idx.x);
    double obsDistance_goal = distances(goal_idx.x,goal_idx.y);

    // std::cout << "here0.5" << std::endl;

    if (obsDistance_goal < params.minDistanceToObstacle || obsDistance_start < params.minDistanceToObstacle){
        path.utime = start.utime;    
        path.path_length = path.path.size();
        return path;
        // std::cout << "goal too close to obstacle" << std::endl;
    }

    // std::cout << "here1" << std::endl;

    // std::cout << "start: " << start.x << "," << start.y << "," << start.utime << std::endl;
    // std::cout << "goal: " << goal.x << "," << goal.y << "," << goal.utime << std::endl;

    // 2D configuration map for closed list
    map_point initial;
    std::vector<std::vector<map_point>> closed_list(distances.heightInCells(),std::vector<map_point>(distances.widthInCells(),initial));

    // open list
    // std::priority_queue<Node> open_list;

    std::priority_queue<Node,std::vector<Node>,compare_f> open_list;

    // add start node to openlist
    Node start_node;
    start_node.self = start_idx;
    start_node.parent = start_idx;
    start_node.g = 0;
    open_list.push(start_node);

    // create current node
    Node current_node;

    // std::cout << "here2" << std::endl;

    // check if open list is empty
    while(!open_list.empty()){
        
        // std::cout << "in loop" << std::endl;

        // get current node in open list
        current_node = open_list.top();
        open_list.pop();
        // std::cout << "openlist size: " << open_list.size() << std::endl;

        // check if the current node has been explored
        // if (closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
        //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)].utime != -1){
        
        // convert global coordinate to cell index
        // Point<double> current_node_pos(current_node.self.x,current_node.self.y);
        // Point<int> current_node_idx = global_position_to_grid_cell(current_node_pos, distances);

        // std::cout << "world coordinate: " << current_node.self.x << "," << current_node.self.y << std::endl;
        // std::cout << "cell coordinate: " << current_node_idx.x << "," << current_node_idx.y << std::endl;

        if (closed_list[current_node.self.y][current_node.self.x].explored){
            // std::cout << "current has been explored" << std::endl;
            continue;
        }

        // std::cout << "here" << std::endl;
        
        // if (closed_list[round(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2)][round( \
        //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2)].utime != -1){
        //     // std::cout << "current has been explored" << std::endl;
        //     continue;
        // } 

        // insert current node to closed list
        // closed_list[int(-current_node.self.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int( \
        //             current_node.self.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)] = current_node.parent;
        closed_list[current_node.self.y][current_node.self.x].parent = current_node.parent;
        closed_list[current_node.self.y][current_node.self.x].explored = 1;

        // Goal check
        // if (sqrt((current_node.self.x - goal.x)*(current_node.self.x - goal.x)+ \
        //     (current_node.self.y - goal.y)*(current_node.self.y - goal.y)) < distances.metersPerCell())
        // if (sqrt((current_node.self.x - goal.x)*(current_node.self.x - goal.x)+ \
        //     (current_node.self.y - goal.y)*(current_node.self.y - goal.y)) < 0.001)
        if(current_node.self.x == goal_idx.x && current_node.self.y == goal_idx.y)
        {         
            // std::cout << "reach goal" << std::endl;
            path.path.push_back(goal);

            // parent node
            Point<int> parent;
            Point<int> current = current_node.self;
            Point<double> parent_pose;
            Point<double> current_pose = grid_position_to_global_position(Point<double>(current.x,current.y), distances);

            // std::cout << "back track start" << std::endl;

            // back trace to start
            // while (sqrt((current.x - start.x)*(current.x - start.x)+ \
            //         (current.y - start.y)*(current.y - start.y)) > 0.001)
            while (current.x != start_idx.x || current.y != start_idx.y)
            {
                // parent = closed_list[int(-current.y/distances.metersPerCell()+distances.heightInCells()/2+0.5)][int(current.x/distances.metersPerCell()+distances.widthInCells()/2+0.5)];
                // parent = closed_list[round(-current.y/distances.metersPerCell()+distances.heightInCells()/2)][round(current.x/distances.metersPerCell()+distances.widthInCells()/2)];
                
                parent = closed_list[current.y][current.x].parent;
                parent_pose = grid_position_to_global_position(Point<double>(parent.x,parent.y), distances);
                
                pose_xyt_t parent_state;
                parent_state.x = parent_pose.x;
                parent_state.y = parent_pose.y;
                parent_state.theta = atan2(current_pose.y - parent_pose.y, current_pose.x - parent_pose.x);
                path.path.push_back(parent_state);
                current = parent;
                current_pose = parent_pose;
            }

            // std::cout << "back track end" << std::endl;

            std::reverse(path.path.begin(),path.path.end());
            break;
        }


        // expand current node
        // Point<int> current_idx = global_position_to_grid_cell(Point<double>(current_node.self.x,current_node.self.y), distances);
        for(int x = current_node.self.x - 1;x <= current_node.self.x + 1; x++)
        {
            for(int y = current_node.self.y - 1;y <= current_node.self.y + 1; y++)
            {
                // convert global coordinate to cell index
                // Point<double> pos(x,y);
                // Point<int> idx = global_position_to_grid_cell(pos, distances);

                // check if the child is current node
                // if (sqrt((current_node.self.x - x)*(current_node.self.x - x)+ \
                //          (current_node.self.y - y)*(current_node.self.y - y) < 0.001)){
                if(x == current_node.self.x && y == current_node.self.y){
                            // std::cout << "child is current node" << std::endl;
                            continue;
                         } 

                // check if the child is in grid
                // if (!distances.isCellInGrid(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
                //     int(-y/distances.metersPerCell()+distances.heightInCells()/2))){
                if (!distances.isCellInGrid(x,y)){
                        // std::cout << "child is not in grid" << std::endl;
                        continue;
                    } 

                // check if the child is an obstacle
                // float obsDistance = distances(int(x/distances.metersPerCell()+distances.widthInCells()/2), \
                //     int(-y/distances.metersPerCell()+distances.heightInCells()/2));
                float obsDistance = distances(x,y);
                if (obsDistance < params.minDistanceToObstacle){
                    // std::cout << "child is an obstacle" << std::endl;
                    continue;
                } 

                // // check if the child has been explored
                // if (closed_list[int(-y/distances.metersPerCell()+distances.heightInCells()/2)][int(x/distances.metersPerCell()+distances.widthInCells()/2)].utime != -1){
                //     // std::cout << "child has been explored" << std::endl;
                //     continue;
                // } 

                // add child to open list
                // Point<double> child_pos = grid_position_to_global_position(Point<double>(x,y),distances);
                // pose_xyt_t child_pose;
                // child_pose.x = child_pos.x;
                // child_pose.y = child_pos.y;

                Node child;
                child.self.x = x;
                child.self.y = y;
                child.parent = current_node.self;
                child.g = current_node.g + distances.metersPerCell()*sqrt(1.0*((child.self.x-current_node.self.x)*(child.self.x-current_node.self.x)+(child.self.y-current_node.self.y)*(child.self.y-current_node.self.y)));
                
                // child.f = distances.metersPerCell()*sqrt(1.0*((child.self.x-goal_idx.x)*(child.self.x-goal_idx.x)+(child.self.y-goal_idx.y)*(child.self.y-goal_idx.y)));
                
                child.f = child.g + distances.metersPerCell()*sqrt(1.0*((child.self.x-goal_idx.x)*(child.self.x-goal_idx.x)+(child.self.y-goal_idx.y)*(child.self.y-goal_idx.y)));

                float obs_cost = (obsDistance < params.maxDistanceWithCost) ? \
                            10.0*(params.maxDistanceWithCost - obsDistance) : 10.0*(obsDistance - params.maxDistanceWithCost);
                child.f += obs_cost;

                open_list.push(child);
                // std::cout << "add child" << std::endl;
            }
        } 
    }

    // int finish = 1;
    // for(std::vector<std::vector<map_point>>::iterator it1 = closed_list.begin();it1 != closed_list.end();it1++){
    //     for(std::vector<map_point>::iterator it2 = (*it1).begin();it2 != (*it1).end();it2++){        
    //         if ((*it2).explored == 0)
    //         {
    //             finish = 0;
    //             break;
    //         }
    //     }
    //     if(finish == 0) break;
    // }

    // if(finish) std::cout << "all map explored" << std::endl;
    // else std::cout << "not all map explored" << std::endl;
    
    
    path.utime = start.utime;    
    path.path_length = path.path.size();
    // std::cout << "length of A* path: " << path.path_length << std::endl;
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
