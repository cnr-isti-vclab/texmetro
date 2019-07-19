VCGPATH = ../../vcglib

CONFIG += console
CONFIG += c++11
CONFIG -= app_bundle

TARGET = texmetro



#### QT STUFF ##################################################################

TEMPLATE = app
QT = core gui svg



##### INCLUDE PATH #############################################################

INCLUDEPATH += $$VCGPATH $$VCGPATH/eigenlib



#### LIBS ######################################################################

unix {
  CONFIG += link_pkgconfig
  PKGCONFIG += glfw3 glew
  LIBS += -lGL -lGLEW
}

win32 {
  WIN_GLFW_PATH = $$PWD/glfw       # set to glfw dir
  WIN_GLEW_PATH = $$PWD/glew       # set to glew dir

  DEFINES += GLFW_DLL
  INCLUDEPATH += $$WIN_GLFW_PATH/include
  LIBS += -L$$WIN_GLFW_PATH/lib-vc2015 -lglfw3dll

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
    gl_utils.h \
    types.h \
    measure.h

SOURCES += \
    $$VCGPATH/wrap/ply/plylib.cpp \
    $$VCGPATH/wrap/openfbx/src/ofbx.cpp \
    $$VCGPATH/wrap/openfbx/src/miniz.c \
    measure.cpp

SOURCES += \
    texmetro.cpp \
    gl_utils.cpp

