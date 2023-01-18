#ifndef SRC_FLOORPLANING_SYSTEM_HPP_
#define SRC_FLOORPLANING_SYSTEM_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <random>
#include <list>

namespace floorplaning::backend {


/*Block & Terminal*/
struct Block {
    Block(std::string name, size_t w, size_t h, int x = 0 , int y = 0)
    : name(name), width(w), height(h), x(x), y(y) {}
    std::string name;
    size_t width;
    size_t height;

    // Only for Terminal coordinate info
    int x;
    int y;
};


struct TreeNode {
    explicit TreeNode(std::string name): name(name) {}
    std::string name;
    std::shared_ptr<TreeNode> parent{nullptr};
    std::shared_ptr<TreeNode> right{nullptr};
    std::shared_ptr<TreeNode> left{nullptr};
    size_t x;
    size_t y;
};

struct Net {
    std::vector<std::string> blocks_in_net;  // only name!!
};

/*system infomation*/
struct System {

    size_t outline_width;
    size_t outline_height;
    size_t num_nets;
    size_t num_blocks;
    size_t num_terminals;
    double alpha;

    /*Input info*/
    std::unordered_map<std::string, std::shared_ptr<Block>> block_map;
    std::unordered_map<std::string, std::shared_ptr<Block>> terminal_map;
    std::vector<std::shared_ptr<Net>> net_list;

    /*output info*/
    std::vector<std::shared_ptr<TreeNode>> node_list;
};

}  // namespace floorplaning::backend
#endif  // SRC_FLOORPLANING_SYSTEM_HPP_

