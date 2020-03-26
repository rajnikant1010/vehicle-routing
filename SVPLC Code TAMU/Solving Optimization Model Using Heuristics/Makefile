SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic

CPLEXDIR      = /home/tyler/Documents/cplex
CONCERTDIR    = /home/tyler/Documents/concert

CCC = g++ -O0

CCOPT = -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD -std=c++11

CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -pthread -L/home/tyler/Desktop/lemon-1.3.1/build/lemon -lemon # -L$(IGRAPHLIBDIR) -ligraph 

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR)


all : main

main : main.o
	@echo "Linking started"
	@echo
	g++ $^ $(CCLNFLAGS) -o $@
	@echo
	@echo "Created executable main"
	
main.o : main.cpp
	@echo "Compiling main.cpp"
	@echo
	g++ $< $(CCFLAGS) -c $<
	@echo
	@echo "Created object file main.o"
	@echo


clean :
	rm -f main.o main

