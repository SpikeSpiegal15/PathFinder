#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &this->m_Model.FindClosestNode(start_x, start_y);  
    end_node = &this->m_Model.FindClosestNode(end_x, end_y); 
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node); 
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors(); 
    for(auto n: current_node->neighbors) {
      n->parent = current_node; 
      n->h_value = this->CalculateHValue(n);
      n->g_value = current_node->distance(*n) + current_node->g_value; 
      n->visited = true; 
      this->open_list.push_back(n); 
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), [](RouteModel::Node* x, RouteModel::Node* y) { 
        return (x->h_value + x->g_value) < (y->h_value + y->g_value); 
    }); 
    RouteModel::Node* lowest = this->open_list.front(); 
    this->open_list.erase(this->open_list.begin()); 
    return lowest; 
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  
    // TODO: Implement your solution here.
    while (current_node != nullptr) {
    	path_found.push_back(*current_node); 
        if(current_node->parent != nullptr) {
          distance += current_node->distance(*(current_node->parent));
        } 
    	current_node = current_node->parent; 
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found; 
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
    
    // TODO: Implement your solution here.
    do {
      this->AddNeighbors(current_node);
      current_node->visited = true; 
      current_node = this->NextNode(); 
      if (current_node->distance(*(this->end_node)) == 0) {
        this->m_Model.path = this->ConstructFinalPath(this->end_node); 
        return; 
      }
    } while (!this->open_list.empty()); 
}