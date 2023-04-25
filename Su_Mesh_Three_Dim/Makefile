#预定义变量
CXX = clang++

#预定义处理args
CPPFLAGS=-g
#CPPFLAGS+= -Wall
CPPFLAGS+= -pedantic
CPPFLAGS+= -pthread
CPPFLAGS+= -fuse-ld=lld
CPPFLAGS+= -stdlib=libc++
CPPFLAGS+= -std=c++2b

#预定义目标
TARGET = Su_Mesh.exe
OBJS_DIR = obj
SRCS = $(wildcard *.cpp)
OBJS = $(patsubst %.cpp, ./$(OBJS_DIR)/%.o, $(SRCS))
HEADER_PATH = -I D:\\CGAL-5.5.2\\include -I D:\\boost_1_82_0 #-I D:\\CGAL-5.5.2\\auxiliary\\gmp\\include
LIB_PATH = -L D:\\CGAL-5.5.2\\auxiliary\\gmp\\lib

LIBS = -lgmp-10 -lmpfr-4

all:CHECK_OBJS_DIR $(TARGET)

CHECK_OBJS_DIR:
	@mkdir -p $(OBJS_DIR)
#	@if(-not (Test-Path $(OBJS_DIR))) {New-Item -ItemType Directory -Path $(OBJS_DIR)}
	
$(TARGET):$(OBJS)
	@$(CXX) $(CPPFLAGS) $^ $(LIB_PATH) $(LIBS) -o $@ 

./$(OBJS_DIR)/%.o:%.cpp                                                                                   
	@$(CXX) $(CPPFLAGS) -c $< -o $@ $(HEADER_PATH)

.PHONY:clean

clean:
	@rm -rf $(OBJS_DIR) $(TARGET)

.PHONY:run

run:
	@$(TARGET)

