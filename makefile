SHELL=cmd.exe
AS=as    
CC=gcc
CPLUS=g++

LD=g++ 
RC=windres

ASFLAGS=-g
#-fexec-charset=UTF-8可編譯中文
CCFLAGS=-g -Wall -D UNICODE -D _UNICODE 
#-Wall -O3 -std=c99 -fexec-charset=UTF-8
LDFLAGS=-lgdi32 
#-s -lcomctl32 -Wl,--subsystem,windows 
RCFLAGS=

OBJECT=main.o \
		neogdi.o \
		world.o \
		worldtimer.o \
		vehicle.o \
		SteeringBehaviors.o \
		obstacle.o

TARGET=neo.exe


all: $(TARGET)

$(TARGET) : $(OBJECT)
		@echo  [ LD] $<
		@$(LD) $(OBJECT) -o $(TARGET) $(LDFLAGS)

# $@表示輸出文件，$<表示輸入文件
%.o : %.c
		@echo  [GCC] $<
		@$(CC) $(CCFLAGS) -c -o $@ $<

%.o : %.cpp
		@echo  [G++] $<
		@$(CPLUS) $(CCFLAGS) -c -o $@ $<

%.o : %.s
		@echo  [ AS] $<
		@$(AS) $(ASFLAGS) -o $@ $<
       
%.o : %.rc
		@echo  [ RC] $<
		@$(RC) $(RCFLAGS) $< $@
   
clean:
	-rm -f $(TARGET)
	-rm -f $(OBJECT)
	-del /q $(TARGET)
	-del /q $(OBJECT)

run:
	$(TARGET)
