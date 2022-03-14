CC = g++
FLAGS = -lm -pthread
EXE = main.exe
OBJS = main.o src/graph.o src/struc2vec.o src/utils.o
W2V_OBJ = src/word2vec/build/lib/CMakeFiles/word2vec.dir/*.cpp.o

$(EXE): $(OBJS) $(W2V_OBJ)
	$(CC) $(FLAGS) -o $(EXE) $(OBJS) $(W2V_OBJ)

%.o: %.cpp
	$(CC) $(FLAGS) -c -o $@ $<


clean:
	rm $(EXE) $(OBJS)