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
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // populate current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();

    // For each node in current_node.neighbors, set the parent, the h_value, the g_value and visited attribute to true
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;

        // add the neighbor to open_list
        this->open_list.push_back(neighbor);
    }
}

bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
    // calculate the F Value for node a
    const float a_f_value = a->h_value + a->g_value;

    // calculate the F Value for node b
    const float b_f_value = b->h_value + b->g_value;

    // return compare result
    return a_f_value > b_f_value;
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(this->open_list.begin(), this->open_list.end(), Compare);

    // Create a pointer to the node in the list with the lowest sum
    RouteModel::Node* lowest_f_node  = this->open_list.back();

    // Remove that node from the open_list.
    this->open_list.pop_back();

    // Return the pointer.
    return lowest_f_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // add the end node first to the path vector
    path_found.push_back(*current_node);

    // iteratively follow the chain of parents of nodes until the starting node is found
    while (!(current_node->x == this->start_node->x && current_node->y == this->start_node->y)) {
        // add the nodes to form the path
        path_found.push_back(*(current_node->parent));

        // add the distance from the node to its parent to the distance variable
        distance += current_node->distance(*(current_node->parent));

        // set the current node to its parent
        current_node = current_node->parent;
    }

    // reverse the order of the path_found vector to make the start node the first node in the path
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

    // initialize the start_node parameters
    this->start_node->g_value = 0.0f;
    this->start_node->h_value = CalculateHValue(this->start_node);
    this->start_node->visited = true;

    // set the current_node to be the start node
    current_node = this->start_node;

    // add the start node to the open list
    this->open_list.push_back(current_node);

    // loop over the open vector till the vector becomes empty 
    do {
        // sort the open_list and return the next node
        current_node = this->NextNode();

        // check if we reached the end node
        if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
            // use the ConstructFinalPath method to return the final path that was found
            auto final_path = ConstructFinalPath(current_node);

            // Store the final path in the m_Model.path
            m_Model.path = final_path;

            // return from the method
            return;
        }

        // add all of the neighbors of the current node to the open_list
        this->AddNeighbors(current_node);
    } while (!this->open_list.empty());

    // no path was found
}