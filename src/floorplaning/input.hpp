#ifndef SRC_FLOORPLANING_INPUT_HPP_
#define SRC_FLOORPLANING_INPUT_HPP_


#include <floorplaning/system.hpp>

namespace floorplaning {


class Input {
 public:
    using system_ptr_type = std::shared_ptr<backend::System>;
    using string_type = std::string;

    /* --- Constructor & Destructor --- */
    explicit Input(const double& alpha, std::ifstream& blocks, std::ifstream& nets);
    virtual ~Input() = default;
    system_ptr_type getData(void) noexcept;
    /*----------------------------------*/

    /*read input file to get layout info*/
    bool readFile(void);

 private:
    system_ptr_type system_ptr_{nullptr};
    std::ifstream& blocks_;
    std::ifstream& nets_;

    // read main file
    bool readBlocks();
    bool readNets();
};

inline Input::Input(const double& alpha, std::ifstream& blocks, std::ifstream& nets)
: blocks_(blocks), nets_(nets) {
    system_ptr_ = std::make_shared<backend::System>();
    system_ptr_->alpha = alpha;
}

inline bool Input::readFile() {
    readBlocks();
    readNets();
    return true;
}

inline typename Input::system_ptr_type Input::getData() noexcept {
    return system_ptr_;
}

/*------------------------
   Private Implementation
-------------------------*/
inline bool Input::readBlocks() {
    auto& outline_width = system_ptr_->outline_width;
    auto& outline_height = system_ptr_->outline_height;
    auto& num_blocks = system_ptr_->num_blocks;
    auto& num_terminals = system_ptr_->num_terminals;
    auto& block_map = system_ptr_->block_map;
    auto& terminal_map = system_ptr_->terminal_map;
    std::string temp;
    blocks_ >> temp >> outline_width >> outline_height;
    blocks_ >> temp >> num_blocks;
    blocks_ >> temp >> num_terminals;
    block_map.reserve(num_blocks);
    terminal_map.reserve(num_terminals);

    /*read block*/
    for (size_t i = 0; i < num_blocks; ++i) {
       std::string name;
       size_t width, height;

       blocks_ >> name >> width >> height;
       backend::Block block(name, width, height);
       block_map[name] = std::make_shared<backend::Block>(block);
    }

    /*read terminal*/
    for (size_t i = 0; i < num_terminals; ++i) {
       std::string name;
       int x, y;

       blocks_ >> name >> temp>> x >> y;
       backend::Block termianl(name, 0, 0, x, y);
       terminal_map[name] = std::make_shared<backend::Block>(termianl);
       // std::cout << "check ==> " << terminal_list[i]->name
       // << " "  << terminal_list[i]->x << " " << terminal_list[i]->y << std::endl;
    }

    return true;
}

inline bool Input::readNets() {
    auto& num_nets = system_ptr_->num_nets;
    auto& net_list = system_ptr_->net_list;
    const auto& block_map = system_ptr_->block_map;
    const auto& terminal_map = system_ptr_->terminal_map;
    std::string temp;
    size_t degree, iter{0};
    nets_ >> temp >> num_nets;
    nets_ >> temp >> degree;
    net_list.resize(num_nets);
    while (iter < num_nets) {
       auto net_ptr = std::make_shared<backend::Net>();
       net_ptr->blocks_in_net.resize(degree);
       for (size_t i = 0; i < degree; ++i) {
          std::string id;
          nets_ >> id;
          if (block_map.find(id) != block_map.end()) {
             net_ptr->blocks_in_net[i] = block_map.find(id)->first;
          } else {
             if (terminal_map.find(id) != terminal_map.end()) {
                net_ptr->blocks_in_net[i] = terminal_map.find(id)->first;
             }
          }
       }
       net_list[iter] = net_ptr;
       nets_ >> temp >> degree;
       iter += 1;
    }
    return true;
}

}  // namespace floorplaning

#endif  // SRC_FLOORPLANING_INPUT_HPP_
