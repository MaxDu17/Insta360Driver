# Compiler and linker settings
CXX = g++
CXXFLAGS = -Iinclude  # Include directory for header files
LDFLAGS = -Llib/ # Path to the library
LDLIBS = -lCameraSDK # Name of the library (without lib prefix and extension)

# Directories
SRCDIR = src
OBJDIR = obj
BINDIR = bin

# Files
SRC = src/main.cpp # All source files in src/
# SRC = src/simple_cam.cpp # All source files in src/

# SRC = $(wildcard $(SRCDIR)/*.cpp)  # All source files in src/

OBJ = $(SRC:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)  # Object files in obj/
EXEC = $(BINDIR)/main  # Output executable

# Default target (build the executable)
all: $(EXEC)

# Build the executable
$(EXEC): $(OBJ)
	$(CXX) $(OBJ) $(LDFLAGS) $(LDLIBS) -o $@

# Rule to compile source files to object files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up the generated files
clean:
	rm -rf $(OBJDIR)/*.o $(EXEC)

# Create the necessary directories if they don't exist
$(OBJDIR) $(BINDIR):
	mkdir -p $@

.PHONY: all clean
