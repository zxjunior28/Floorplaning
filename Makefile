# Compiler
CC = g++

# Flags, Libraries and Includes
CFLAGS = -O3
Linking =

# The Directories, Source, Includes, Objects, Binary
INC_DIR = -I src/
OBJ_DIR = build/obj
BIN_DIR = .


# The Target Executor
EXE = $(addprefix $(BIN_DIR)/,\
		Lab2 \
	)


OBJS =  $(addprefix $(OBJ_DIR)/,\
		example_lab2.o \
	)



# Linking
EXE :  $(OBJS) 
	@mkdir -p $(BIN_DIR)
	@$(CC) -o $(EXE) \
    $(OBJS) \
    $(CFLAGS) $(Linking)




	@echo "   ************ Build successful *****************  "
	@echo "    |  Author:  Kaiyi Bai                           "
	@echo "    |  Lab :  Floorplaning                          "
	@echo "   ***********************************************  "


# Compile
${OBJ_DIR}/%.o: examples/%.cpp
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) $(INC_DIR) -c $< -o $@



# Full Clean, Objects and Binaries
clean:
	@$(RM) -f $(OBJ_DIR)/*.o
	@$(RM) -f $(EXE)


