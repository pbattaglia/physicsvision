CC = g++
INCDIR = include
OBJDIR = obj
SYSDIR = -I/usr/local/include/bullet -L/usr/local/lib
CFLAGS =  $(SYSDIR) -I$(INCDIR)

LIBS = -lBulletCollision -lBulletDynamics -lLinearMath
GLLIBS = -lGL -lGLU -lglut

_DEPS = physics_score.h
DEPS = $(patsubst %,$(INCDIR)/%,$(_DEPS))

_OBJ = physics_score.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))


$(OBJDIR)/%.o: %.cpp $(INCDIR)/%.h
	$(CC) -g -c -o $@ $< $(CFLAGS)

main: main.cpp $(OBJ) 
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

viewer: viewer.cpp $(OBJ) $(OBJDIR)/GLDebugDrawer.o
	$(CC) -g -o $@ $^ $(CFLAGS) $(LIBS) $(GLLIBS)

.PHONY: clean

clean:
	rm -f $(OBJDIR)/*.o *.o


# g++ -g -I/usr/local/include/bullet -lBulletCollision -lBulletDynamics -lLinearMath -o physics_score physics_score.cpp
