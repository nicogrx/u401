OBJ = u401.o
DEPLIBS = -lusb -lpthread
STATIC_LIB = libu401.a
LIB = libu401.so
TESTEXE = test

all: $(STATIC_LIB) $(LIB) test

$(STATIC_LIB) : $(OBJ)
	$(AR) rcs $@ $(OBJ)

$(LIB) : $(OBJ)
	$(CC) -shared -Wl,-soname,$(LIB) -o $(LIB) $(OBJ) $(DEPLIBS)

%.o : %.c
	$(CC) -fpic -g -c $< -DDEBUG

$(TESTEXE) : $(TESTEXE).o $(LIB)
	$(CC) -g -o $(TESTEXE) $(TESTEXE).o -lu401 -L.

install: $(LIB)
	install -D -m 755 $(LIB) $(PREFIX)/lib

clean:
	rm -f $(OBJ) $(LIB) $(STATIC_LIB) $(TESTEXE).o $(TESTEXE)
