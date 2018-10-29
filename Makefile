CXX=g++

CFLAG= -g -o -Wall
INCLUDES= -I /usr/local/include/opencv2 #`pkg-config opencv --cflags`
LIBS=-L /usr/local/lib -lopencv_core #`pkg-config opencv --libs`


SOURCE := AlgoKalman.cpp AlgoGPSDataProcessor.cpp
OBJS :=AlgoKalman.o AlgoGPSDataProcessor.o
TARGET :=libalgogpsdataprocessor.so


$(TARGET):AlgoGPSDataProcessor.o
	$(CXX) $(CFLAGS) -shared -fPIC -o $(TARGET)   $(OBJS) $(LIBS)

$(OBJS):$(SOURCE)
	$(CXX) $(CFLAGS) -fPIC -c $(SOURCE)  $(INCLUDES)

clean:
	rm -rf *.o $(TARGET)
