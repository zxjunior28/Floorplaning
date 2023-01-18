// Copyright (c) 2022 Katelyn Bai
#include <floorplaning/lab2.hpp>
#include <chrono> 

void clientCode(int argc, char *argv[]);

int main(int argc, char *argv[]) {

    clientCode(argc, argv);
    return 0;
}


void clientCode(int argc, char *argv[]) {
    if (argc < 4) {
        throw std::invalid_argument("Usage: ./Lab2 <Alpha> <Input_block flie> <Input_nets flie> <Output rpt file>");
    }
    const double alpha = std::stod(argv[1]);
    const std::string output_rpt_file = argv[4];
    std::ifstream blocks(argv[2], std::ifstream::in);
    std::ifstream nets(argv[3], std::ifstream::in);
    std::fstream outs(argv[4], std::fstream::out);

    if (blocks.fail() || nets.fail()) {
        std::cerr << "no such file!! " <<  std::endl;
    }

    /* input */
    floorplaning::Input input(alpha, blocks, nets);
    input.readFile();
    const auto& system_ptr = input.getData();
    blocks.close();
    nets.close();

    /*Floorplanning B*-Tree using SA*/
    // clock_t start, end;
    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();
    floorplaning::FloorPlan floorplan(system_ptr);
    floorplan.buildTreeNode();
    floorplan.SimulatedAnnealing();
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    floorplan.getReport(outs, duration.count());

    // elapsed time
    // std::cout << "Timer: " << (end - start)/CLOCKS_PER_SEC << " sec" << std::endl;

}
