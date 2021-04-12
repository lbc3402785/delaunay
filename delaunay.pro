QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
OBJECTS_DIR=$${PWD}/build
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
OPENCV_INCLUDE_DIRS=g:\soft\opencv3\build\include
OPENCV_LIBRARY_DIRS=g:\soft\opencv3\build\x64\vc15\lib
ZLIB_INCLUDE_DIRS="G:\soft\zlib\include"
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH+=g:\soft\eigen-eigen-323c052e1731
INCLUDEPATH+=$$(DLIB_ROOT)\include
INCLUDEPATH+=$$OPENCV_INCLUDE_DIRS
INCLUDEPATH+=$$ZLIB_INCLUDE_DIRS
LIBS+=-L$$OPENCV_LIBRARY_DIRS -lopencv_world344
LIBS+=-L$$(DLIB_ROOT)\lib -ldlib19.16.99_release_64bit_msvc1916
SOURCES += main.cpp \
    test.cpp \
    Dlib.cpp

HEADERS += \
    delaunay.h \
    test.h \
    Dlib.h
