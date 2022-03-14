CC = g++
FLAGS = -lm -pthread
EXE = main.exe
OBJS = main.o graph.o struc2vec.o utils.o
W2V_OBJ = word2vec/build/lib/CMakeFiles/word2vec.dir/*.cpp.o

$(EXE): $(OBJS) $(W2V_OBJ)
	$(CC) $(FLAGS) -o $(EXE) $(OBJS) $(W2V_OBJ)

%.o: %.cpp
	$(CC) $(FLAGS) -c -o $@ $<


clean:
	rm $(EXE) $(OBJS)
