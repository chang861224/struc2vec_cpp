CC = g++
FLAGS = -lm
EXE = main.exe
OBJS = main.o graph.o struc2vec.o utils.o

$(EXE): $(OBJS)
	$(CC) $(FLAGS) -o $(EXE) $(OBJS)

%.o: %.cpp
	$(CC) $(FLAGS) -c -o $@ $<


clean:
	rm $(EXE) $(OBJS)
