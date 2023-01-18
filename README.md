# Fixed-outline Floorplanning
fixed-ouline floorplanning implemented in B*-Tree using simulated annealing(SA)


## Build with Makefile directly  (Homework format)
```console
$ make clean
$ make
$ ./Lab2 <Alpha> <Input_block> <Input_nets> <Output_rpt_file> 
```

## Build with CMake by scripts
```console
$ source scripts/boost.sh <Alpha> <case_name>
```

## Build with CMake directly 
```console
$ rm -rf build/
$ cmake -S . -B build/ -DBUILD_EXAMPLES=ON
$ cmake --build build/ -j4
$ ./build/bin/Lab2 <Alpha> <Input_block> <Input_nets> <Output_rpt_file>
```
