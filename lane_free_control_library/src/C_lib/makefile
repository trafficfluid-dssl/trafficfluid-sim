CC = gcc

default: clean libcontroller.a

libcontroller.a: controller_g.o controller_api.o
	ar rcs $@ $^

controller_g.o: controller_g.c controller_api.c
	$(CC)  $^ -c -fPIC -lm -fopenmp 

clean:
	rm -f *.o *.a
