#include "route_planner.h"
#include <algorithm>
#include <map>
  
using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    RouteModel::Node *closest_to_start = &m_Model.FindClosestNode(start_x, start_y);
    RouteModel::Node *closest_to_end = &m_Model.FindClosestNode(end_x, end_y);  
  	start_node = closest_to_start;
    end_node = closest_to_end;
  
  	start_node->h_value = CalculateHValue(start_node);
  	start_node->g_value = 0.0f;
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->neighbors.clear();	
  	current_node->FindNeighbors();  	
  
  	for(auto node_ptr: current_node->neighbors) {     
      	
    	node_ptr->h_value = CalculateHValue(node_ptr);
      	node_ptr->g_value = current_node->g_value + node_ptr->distance(*current_node);
      	node_ptr->parent = current_node;      	
      
      	if(!node_ptr->visited && std::find(open_list.begin(), open_list.end(), node_ptr) == open_list.end()) {
      		node_ptr->visited = true;
      		open_list.push_back(node_ptr);
        }
            
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
  std::map<float, RouteModel::Node*> sort_map;
  
  for(auto node_ptr: open_list ) {
   	if(sort_map.find(node_ptr->g_value + node_ptr->h_value) == sort_map.end()) {
    	sort_map.insert(pair<float,RouteModel::Node*> (node_ptr->g_value + node_ptr->h_value, node_ptr));      
   }
  }
  
  //save the next node to return
  auto iter = sort_map.begin();
  auto next_node = iter->second;      
  sort_map.erase(iter->first);  
  
  open_list.clear();  
  
  for(auto itr = sort_map.begin(); itr != sort_map.end(); ++itr) {
  	open_list.push_back(itr->second);
  }
    
  return next_node; 

}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
      
  	path_found.push_back(*current_node);  	
  	float dist = 0.0f;
  
    while(current_node != start_node) {      	
        distance += current_node->distance(*(current_node->parent));        
      	path_found.push_back(*(current_node->parent));      	
      	current_node = current_node->parent;      	
    }
	
  	reverse(path_found.begin(), path_found.end());  	
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;  	    
  	current_node = start_node;  	  	  
  
  	while(current_node != end_node) {                
    	AddNeighbors(current_node);     
      	current_node = NextNode();          
    }
  	
  	m_Model.path = ConstructFinalPath(current_node); 
  	cout << "g value at the end" << current_node->g_value << "\n";
  	cout << "Completed A star. " << "\n";

}
