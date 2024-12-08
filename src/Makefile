CXX = g++
CXXFLAGS = -I./include
LDFLAGS = -L./some -lstdc++ -lpthread -lpigpio -lsl_lidar_sdk

TARGET = lidar
SRC = lidar.cpp

$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(CXXFLAGS) $(LDFLAGS)
run:
	sudo ./lidar

clean:
	rm lidar
