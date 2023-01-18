#ifndef SRC_FLOORPLANING_FLOORPLAN_HPP_
#define SRC_FLOORPLANING_FLOORPLAN_HPP_


#include <floorplaning/system.hpp>

namespace floorplaning {

/* Fixed-ouline floorplanning implemented in B*-Tree using simulated annealing(SA) */

class FloorPlan {
 public:
    using system_ptr_type = std::shared_ptr<backend::System>;
    using string_type = std::string;
    using node_ptr_type = std::shared_ptr<backend::TreeNode>;

    /* --- Constructor & Destructor --- */
    explicit FloorPlan(const system_ptr_type& system_ptr) : system_ptr_(system_ptr) {}
    virtual ~FloorPlan() = default;
    /*----------------------------------*/

    /* Main API */
    bool buildTreeNode();
    bool SimulatedAnnealing();
    bool getReport(std::fstream& out, double time);

 private:
    system_ptr_type system_ptr_{nullptr};
    node_ptr_type root_node_{nullptr};
    std::unordered_map<std::string, std::shared_ptr<backend::TreeNode>> node_map_;
    size_t max_width_;
    size_t max_height_;

    struct ContourNode {
        ContourNode(int x1, int x2, int y2) : x1(x1), x2(x2), y2(y2) {}
        int x1, x2, y2;
    };

    std::list<ContourNode> contour_line_;

    // update tree
    void initializeFloorplan();
    void calCoordinate(const node_ptr_type& node_ptr);
    int updateContour(const node_ptr_type& node_ptr);

    // Tree operation
    void rotateBlock(const node_ptr_type& node_ptr);
    void deleteAndInsert(int delete_id, int insert_id);
    void swap(int node1_id, int node2_id);

    // get cost
    double countCost();
    double calHPWL();
    double calRealCost();
};

bool FloorPlan::buildTreeNode() {
    const auto& num_blocks = system_ptr_->num_blocks;
    auto& node_list = system_ptr_->node_list;
    size_t iter{0};
    node_list.resize(num_blocks);
    node_map_.reserve(num_blocks);
    for (const auto& block : system_ptr_->block_map) {
        backend::TreeNode treeNode(block.first);
        auto treeNode_ptr = std::make_shared<backend::TreeNode> (treeNode);
        node_list[iter] = treeNode_ptr;
        if (!node_map_.count(treeNode_ptr->name))  // plz modified!!
            node_map_[treeNode_ptr->name] = node_list[iter];
        iter++;
        // std::cout <<iter<<" "<< block.first << std::endl;
    }

    /*Create Tree Node*/
    for (size_t i = 0; i < num_blocks-1; ++i) {
        if (i == 0) {
            root_node_ = node_list[0];
            root_node_->left = node_list[1];
            node_list[1]->parent = root_node_;
        } else {
            node_list[i]->left = node_list[i+1];
            node_list[i+1]->parent = node_list[i];
        }
    }
    return true;
}


bool FloorPlan::SimulatedAnnealing() {

    // initialize simulated annealing
    const auto& num_block_ = system_ptr_->num_blocks;
    auto& node_list = system_ptr_->node_list;
    const auto& outline_width = system_ptr_->outline_width;
    const auto& outline_height = system_ptr_->outline_height;
    std::random_device rd;                                  // Will be used to obtain a seed for the random number engine
    std::default_random_engine  generator(rd());            // Standard mersenne_twister_engine seeded with rd()
    double temperature = 1.0e+05;                           // initialize temperature
    double not_forzen = 1.0e-05;                            // not forzen temperature
    size_t k = 1000;
    double cost1{0}, cost2{0};
    bool converge{false};


    // SA starts
    while (!converge) {
        int undo = 0;
        std::uniform_real_distribution<>  prob_dis(0.0, 1.0);   // probability distribution of random numbers
        std::uniform_int_distribution<>   method_dis(1, 3);
        for (size_t i = 0; i < k; ++i) {

            initializeFloorplan();
            cost1 = countCost();
            // std::uniform_int_distribution<>   method_dis(1, 3);
            int method = method_dis(generator);

            if (method == 1) {
                std::uniform_int_distribution<> solution_space(0, num_block_-1);
                int rot_block_id = solution_space(generator);

                /*Perturb the floorplan to get a neighboring S' from S*/
                rotateBlock(node_list[rot_block_id]);  // Rotate Block*
                initializeFloorplan();
                cost2 = countCost();

                /*cal probability*/
                double delta_cost = cost1 - cost2;
                double probability = prob_dis(generator);


                // downhill move (not do this step)
                if (delta_cost < 0 && probability >= std::exp(delta_cost/temperature)) {
                    undo++;
                    rotateBlock(node_list[rot_block_id]);
                }

            } else if (method == 2) {
                int delete_id;
                int insert_id;
                while (1) {
                    std::uniform_int_distribution<> delete_dis(1, num_block_-1);
                    std::uniform_int_distribution<> insert_dis(0, num_block_-1);
                    delete_id = delete_dis(generator);
                    insert_id = insert_dis(generator);
                    if ((node_list[delete_id] != root_node_) &&
                        (node_list[delete_id] != node_list[insert_id]) &&
                        (node_list[insert_id]->left == nullptr || node_list[insert_id]->right == nullptr) &&
                        (node_list[delete_id]->left == nullptr || node_list[delete_id]->right == nullptr)) {
                            break;
                    }
                }

                /*store info before perturbing*/
                bool remove_is_left = true;
                auto origin_parent = node_list[delete_id]->parent;
                if (node_list[delete_id]->parent->left == node_list[delete_id])
                    remove_is_left = true;
                else
                    remove_is_left  = false;


                int child_at_left = 0;
                node_ptr_type origin_child{nullptr};
                if (node_list[delete_id] ->left != nullptr) {
                    origin_child = node_list[delete_id]->left;
                    child_at_left = 1;
                } else if (node_list[delete_id] ->right != nullptr) {
                    origin_child = node_list[delete_id]->right;
                    child_at_left = 0;
                } else {
                    child_at_left = -2;  // no child
                }


                /*Perturb the floorplan to get a neighboring S' from S*/
                deleteAndInsert(delete_id, insert_id);  // delete & insert block
                initializeFloorplan();
                cost2 = countCost();

                /*cal probability*/
                double delta_cost = cost1 - cost2;
                double probability = prob_dis(generator);


                /*Undo*/
                if (delta_cost < 0 && probability >= std::exp(delta_cost/temperature)) {
                    //  std::cout <<delta_cost <<" "<< probability <<" " << std::exp(delta_cost/temperature)<< std::endl;
                    // std::cout << "--------------" << std::endl;
                    // fgetc(stdin);
                    auto& delete_node = node_list[delete_id];
                    auto& insert_node = node_list[insert_id];
                    undo++;
                    if (insert_node->left == delete_node)
                        insert_node->left = nullptr;
                    else
                        insert_node->right = nullptr;

                     if (remove_is_left) {
                        if (child_at_left  == 1) {
                            delete_node->left = origin_child;
                            origin_child->parent = delete_node;
                        } else if (child_at_left == 0) {
                            delete_node->right = origin_child;
                            origin_child->parent = delete_node;
                        }
                        origin_parent->left = delete_node;
                        delete_node->parent = origin_parent;
                    }  else if (!remove_is_left)  {
                        if (child_at_left == 1) {
                            delete_node->left = origin_child;
                            origin_child->parent = delete_node;
                        } else if (child_at_left == 0) {
                            delete_node->right = origin_child;
                            origin_child->parent = delete_node;
                        }
                        origin_parent->right = delete_node;
                        delete_node->parent = origin_parent;
                    }

                }

            } else if (method == 3) {
                // std::cout << "method 3" << std::endl;
                std::uniform_int_distribution<> dis(0, num_block_-1);
                std::uniform_int_distribution<> dis2(0, num_block_-1);
                int node1_id = dis(generator);
                int node2_id = dis2(generator);

                while (node1_id == node2_id) {
                    std::uniform_int_distribution<> dis3(0, num_block_-1);
                    node2_id = dis3(generator);
                }

                /*Perturb the floorplan to get a neighboring S' from S*/
                swap(node1_id, node2_id);  // swap two node
                initializeFloorplan();
                cost2 = countCost();

                /*cal probability*/
                double delta_cost = cost1 - cost2;
                double probability = prob_dis(generator);

                /*Undo*/
                if (delta_cost < 0 && probability >= std::exp(delta_cost/temperature)) {
                    undo++;
                    swap(node1_id, node2_id);
                }
            }
        }

        // std::cout << max_width_ << " " << max_height_ << std::endl;

        // reduce temperature
        temperature *= 0.3;

        double undo_rate = static_cast<double>(undo)/static_cast<double>(k);
        if ((undo_rate > 0.96) && (max_width_ <= outline_width) && (max_height_ <= outline_height)
            ||(temperature < not_forzen && (max_width_ <= outline_width) && (max_height_ <= outline_height))) {
            converge = true;
        }
    }
    /*Final update tree*/
    initializeFloorplan();

    return true;
}

bool FloorPlan::getReport(std::fstream& out, double caltime) {

    // std::cout <<"----------Show Results------------" << std::endl;
    // std::cout << "Area : "  << max_width_*max_height_ << std::endl;
    // std::cout << "HPWL : "  << calHPWL() << std::endl;
    // std::cout << "Cost : "  << calRealCost() << std::endl;
    // std::cout << "Height : "  << max_height_ << std::endl;
    // std::cout << "Width : "  << max_width_ << std::endl;
    // std::cout <<"-----------------------------------" << std::endl;

    out << std::fixed << calRealCost() << std::endl;
    out << std::fixed << calHPWL() << std::endl;
    out << max_width_*max_height_ << std::endl;
    out << max_width_ << " " << max_height_ << std::endl;
    out << caltime << std::endl;
    for (int i = 0; i < system_ptr_->num_blocks; ++i) {
        const auto& node = system_ptr_->node_list[i];
        const auto& block = system_ptr_->block_map[node->name];
        out << node->name << " "<< node->x << " " << node->y << " "
        << node->x + block->width << " " <<node->y + block->height << std::endl;
    }

    out.close();
    return true;
}




/*------------------------
   Private Implementation
-------------------------*/

void FloorPlan::initializeFloorplan() {
    max_width_ = 0;
    max_height_ = 0;
    contour_line_.clear();
    calCoordinate(root_node_);  // Calculate TreeNode Coordinate
}

double FloorPlan::countCost() {
    const auto& alpha = system_ptr_->alpha;
    double outline_width = system_ptr_->outline_width;
    double outline_height = system_ptr_->outline_height;
    double total_res = std::max(max_width_ - outline_width, 0.) + std::max(max_height_ - outline_height, 0.);

    if (total_res < 1.0e-8) {
        auto area = max_width_*max_height_;
        auto total_wire_length  = calHPWL();
        return static_cast<double>(outline_width*outline_height)*-5.0/(alpha*area + (1-alpha)*total_wire_length);
        // return static_cast<double>(alpha*area + (1-alpha)*total_wire_length)*-10.0/static_cast<double>(outline_width*outline_height);
    } else {
        return total_res*10.;
    }
}

double FloorPlan::calHPWL() {
    auto& block_map = system_ptr_->block_map;
    auto& terminal_map = system_ptr_->terminal_map;
    const auto& net_list = system_ptr_->net_list;

    /*Calculate total wirelength of a set net*/
    int total_length{0};
    double min_x, min_y, max_x, max_y;
    for (const auto& net : net_list) {
        min_x = std::numeric_limits<double>::max();
        min_y = std::numeric_limits<double>::max();
        max_x = -1;
        max_y = -1;
        for (const auto& name : net->blocks_in_net) {
            if (block_map.count(name)) {
                const auto& block = block_map[name];
                const auto& node = node_map_[name];
                /*half-perimeter is block center*/
                auto block_center_x = static_cast<double>(node->x) + static_cast<double>(block->width)*0.5;
                auto block_center_y = static_cast<double>(node->y) + static_cast<double>(block->height)*0.5;

                min_x = std::min(block_center_x, min_x);
                max_x = std::max(block_center_x, max_x);
                min_y = std::min(block_center_y, min_y);
                max_y = std::max(block_center_y, max_y);
            } else if (terminal_map.count(name)) {
                const auto& terminal = terminal_map[name];
                auto terminal_x = static_cast<double> (terminal->x);
                auto terminal_y = static_cast<double> (terminal->y);

                min_x = std::min(terminal_x, min_x);
                max_x = std::max(terminal_x, max_x);
                min_y = std::min(terminal_y, min_y);
                max_y = std::max(terminal_y, max_y);
                // std::cout << block->name << " " << block->type << " " << block->x << " " << block->y << std::endl;
            }
        }

        /*Sum up the half-perimeter wire length of each net*/
        total_length += static_cast<int>(max_x)-static_cast<int>(min_x)
                        + static_cast<int>(max_y)-static_cast<int>(min_y);
    }
    return total_length;
}

double FloorPlan::calRealCost() {
    const auto& alpha = system_ptr_->alpha;
    auto area = max_height_*max_width_;
    auto total_wire_len = calHPWL();
    return 1.0 * alpha * area + (1.0-alpha) * total_wire_len;
}

void FloorPlan::rotateBlock(const node_ptr_type& node_ptr) {
    auto& block_map = system_ptr_->block_map;
    const auto& block_ptr = block_map[node_ptr->name];
    size_t temp;
    temp = block_ptr->width;
    block_ptr->width = block_ptr->height;
    block_ptr->height = temp;
}

void FloorPlan::deleteAndInsert(int delete_id, int insert_id) {
    const auto& node_list = system_ptr_->node_list;
    const auto& delete_node = node_list[delete_id];
    const auto& insert_node = node_list[insert_id];


    /*delete relation & rebuild tree relation*/
    if (delete_node->left != nullptr) {
        // 1. change child's parent
        delete_node->left->parent =  delete_node->parent;
        //  2. change parent's child
        if (delete_node->parent->left == delete_node) {
            delete_node->parent->left = delete_node->left;
        } else {
            delete_node->parent->right = delete_node->left;
        }
        //  3. delete node's relation
        delete_node->left = nullptr;
    } else if (delete_node->right != nullptr) {
        delete_node->right->parent =  delete_node->parent;
        if (delete_node->parent->left == delete_node)
            delete_node->parent->left = delete_node->right;
        else
            delete_node->parent->right = delete_node->right;

        delete_node->right = nullptr;
    } else {
        /*no child & change parent's relation*/
        if (delete_node->parent->left == delete_node)
            delete_node->parent->left = nullptr;
        else
            delete_node->parent->right = nullptr;
    }

    /*Insert and rebuild tree relation*/
    if (insert_node->left == nullptr && insert_node->right == nullptr) {
        // random insert left or right
        insert_node->left = delete_node;
    } else if (insert_node->left == nullptr) {
        insert_node->left = delete_node;
    } else if (insert_node->right == nullptr) {
        insert_node->right = delete_node;
    } else {
        std::cerr << "please check!!" << std::endl;
        fgetc(stdin);
    }

    delete_node->parent = insert_node;
}

void FloorPlan::swap(int node1_id, int node2_id) {
    auto& node_list = system_ptr_->node_list;
    auto& block_map = system_ptr_->block_map;
    auto node1_name = node_list[node1_id]->name;
    auto node2_name = node_list[node2_id]->name;
    auto block1 = block_map[node1_name];
    auto block2 = block_map[node2_name];

    // swap map ptr
    block_map[block1->name] = block2;
    block_map[block2->name] = block1;
    node_map_[block1->name] = node_list[node2_id];
    node_map_[block2->name] = node_list[node1_id];
 
    // update node info
    std::string temp_name = block1->name;
    node_list[node1_id]->name = node_list[node2_id]->name;
    node_list[node2_id]->name = temp_name;

    // swap info
    size_t temp_w = block1->width;
    size_t temp_h = block1->height;
    block1->name = block2->name;
    block1->width = block2->width;
    block1->height = block2->height;
    block2->name = temp_name;
    block2->width = temp_w;
    block2->height = temp_h;
}

void FloorPlan::calCoordinate(const node_ptr_type& node_ptr) {
    if (node_ptr) {
        auto& block_map = system_ptr_->block_map;
        const auto& curr_block_ptr = block_map[node_ptr->name];
        const auto& parent_node = node_ptr->parent;
        if (parent_node == nullptr) {
            node_ptr->x = 0;
            node_ptr->y = 0;
            ContourNode node(node_ptr->x, curr_block_ptr->width, curr_block_ptr->height);
            contour_line_.push_back(node);
            max_width_ = curr_block_ptr->width;
            max_height_ = curr_block_ptr->height;

        } else if (node_ptr == parent_node->left) {
            const auto& prev_block_ptr =  block_map[parent_node->name];
            node_ptr->x = parent_node->x + prev_block_ptr->width;
            if (node_ptr->x + curr_block_ptr->width > max_width_)
                max_width_ = node_ptr->x + curr_block_ptr->width;
            node_ptr->y = updateContour(node_ptr);
        }  else if (node_ptr == parent_node->right) {
            node_ptr->x = parent_node->x;
            if (node_ptr->x + curr_block_ptr->width > max_width_)
                max_width_ =  node_ptr->x + curr_block_ptr->width;
            node_ptr->y = updateContour(node_ptr);
        }

        /*TreeNode loop*/
        calCoordinate(node_ptr->left);
        calCoordinate(node_ptr->right);
    }
}

/*
 *				       
 *		#############
 *		|           # 
 *		|           #
 *		|           #
 *		|-----------##########
 *		|     |              #    
 *      |     |              #
 *	    |     |              #    
 *		|--------------------###############  <-- contour
 *		|         |              |         #
 *      |         |              |         #
 *	    |         |              |         #
 *      |----------------------------------#  
*/
int FloorPlan::updateContour(const node_ptr_type& node_ptr) {
    const auto& block_ptr = system_ptr_->block_map[node_ptr->name];
    int node_y1{0}, node_y2{0};
    int node_x1 = node_ptr->x;
    int node_x2 = node_x1 + block_ptr->width;
    // std::cout << max_width_ << " " << max_height_ << " " << contour_line_.size() << std::endl;
    std::list<ContourNode>::iterator  point = contour_line_.begin();
    // for (point = contour_line_.begin(); point != contour_line_.end(); ++point) {
    while (point != contour_line_.end()) {
        if (point->x2 <= node_x1) {
            ++point;
        } else if (point->x1 >= node_x2) {
            break;
        } else {
            node_y1 = std::max(point->y2, node_y1);

            if (point->x1 >= node_x1 && point->x2 <= node_x2) {
                /*Case: Both sides of coutour are smaller than block, so remove the contour*/
                point = contour_line_.erase(point);
            } else if (point->x1 >= node_x1 && point->x2 >= node_x2) {
                /*Case: One side of coutour are smaller than block, therfore, update one side value*/
                point->x1 = node_x2;
                continue;
            } else if (point->x1 <= node_x1 && point->x2 <= node_x2) {
                /*Case: One side of coutour are smaller than block, therfore, update one side value*/
                point->x2 = node_x1;
                continue;
            } else {
                /*Case: point->x1 <= x1 && point->x2 >= x2 */
                contour_line_.insert(point, ContourNode(point->x1, node_x1, point->y2));
                point->x1 = node_x2;
                continue;
            }
        }
    }

    node_y2 = node_y1 + block_ptr->height;
    contour_line_.insert(point, ContourNode(node_x1, node_x2, node_y2));
    if (node_y2 > max_height_) max_height_ = node_y2;

    return node_y1;
}


}  // namespace floorplaning

#endif  // SRC_FLOORPLANING_FLOORPLAN_HPP_
