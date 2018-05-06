IDIR =../includes
CC=gcc
CFLAGS=-I$(IDIR)
MAKEFLAGS += --no-print-directory
WARNINGFLAGS=-Wall -Werror

ODIR=obj
LDIR =../lib

LIBS=-lm

_DEPS = *
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = CLmain.o newFile.o 
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

	
$(ODIR)/%.o: %.c $(DEPS)
	@echo Compiling src file $<
	@$(CC) $(WARNINGFLAGS) -c -o $@ $< $(CFLAGS)
	
finalOut: $(OBJ)
	@echo Linking the Object Files $^
	@gcc $(WARNINGFLAGS) -o ../$@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	@echo Clean the object Files
	@rm -f $(ODIR)/*.o *~ ../finalOut 