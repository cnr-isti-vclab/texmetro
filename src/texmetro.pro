VCGPATH = ../../vcglib

CONFIG += console
CONFIG += c++11
CONFIG -= app_bundle

TARGET = texmetro



#### QT STUFF ##################################################################

TEMPLATE = app
QT = core gui opengl



##### INCLUDE PATH #############################################################

INCLUDEPATH += $$VCGPATH $$VCGPATH/eigenlib



#### LIBS ######################################################################

unix {
  CONFIG += link_pkgconfig
  PKGCONFIG += glew
  LIBS += -lGL -lGLEW
}

win32 {
  WIN_GLEW_PATH = $$PWD/glew       # set to glew dir

  INCLUDEPATH += $$WIN_GLEW_PATH/include
  LIBS += -L$$WIN_GLEW_PATH/lib/Release/x64 -lglew32

  LIBS += -lopengl32
}



#### PLATFORM SPECIFIC FLAGS ###################################################

win32 {
  DEFINES += NOMINMAX
}



#### SOURCE FILES ##############################################################

HEADERS += \
    mesh.h \
    measure.h \
    gl_utils.h \
    color_consistency.h

SOURCES += \
    $$VCGPATH/wrap/ply/plylib.cpp \
    $$VCGPATH/wrap/openfbx/src/ofbx.cpp \
    $$VCGPATH/wrap/openfbx/src/miniz.c \
    $$VCGPATH/wrap/system/qgetopt.cpp

SOURCES += \
    texmetro.cpp \
    mesh.cpp \
    measure.cpp \
    gl_utils.cpp \
    color_consistency.cpp
