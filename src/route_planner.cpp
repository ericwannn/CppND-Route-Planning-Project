#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return this->end_node->distance(*node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->visited = true;
    current_node->FindNeighbors();
    for (auto & neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(const RouteModel::Node *n1, const RouteModel::Node *n2) {
    float cost1 = n1->g_value + n1->h_value;
    float cost2 = n2->g_value + n2->h_value;
    return cost1 > cost2;
}

void SortNodes(std::vector<RouteModel::Node *> & nodes) {
    std::sort(nodes.begin(), nodes.end(), Compare);
}

RouteModel::Node *RoutePlanner::NextNode() {
    SortNodes(this->open_list);
    RouteModel::Node *node = this->open_list.back();
    this->open_list.pop_back();
    return node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

bool CheckSameNode(const RouteModel::Node *n1, const RouteModel::Node *n2) {
    return n1->x == n2->x && n1->y == n2->y;
}

bool RoutePlanner::IsStartNode(RouteModel::Node const *current_node) {
    return CheckSameNode(current_node, this->start_node);
}

bool RoutePlanner::IsEndNode(RouteModel::Node const *current_node) {
    return CheckSameNode(current_node, this->end_node);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.push_back(*current_node);
    while(current_node != nullptr){
        // Stop construction when start_node is reached.
        if (this->IsStartNode(current_node)) {
            break;
        }
        RouteModel::Node parent_node = *current_node->parent;
        path_found.push_back(parent_node);
        distance += current_node->distance(parent_node);
        current_node = current_node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    this->open_list.push_back(this->start_node);
    while (this->open_list.size() > 0) {
        current_node = this->NextNode();
        // Stop expanding neighbors when goal is reached. 
        if (this->IsEndNode(current_node)) {
            // Final path is found!
            this->m_Model.path = this->ConstructFinalPath(current_node);
            break;
        }
        this->AddNeighbors(current_node);
    }
}