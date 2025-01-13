QT += core gui

TEMPLATE = lib
DEFINES += ARUCOAPI_LIBRARY

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    arucoapi.cpp \
    markerthread.cpp \
    yamlhandler.cpp

HEADERS += \
    TestLib_global.h \
    arucoapi.h \
    markerthread.h \
    yamlhandler.h

# OPENCV
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/third_party/opencv490_mingw810/x64/mingw/bin/ -llibopencv_world490
else:unix: LIBS += -L$$PWD/third_party/opencv490_mingw810/x64/mingw/lib/ -llibopencv_world490.dll

INCLUDEPATH += $$PWD/third_party/opencv490_mingw810/include
DEPENDPATH += $$PWD/third_party/opencv490_mingw810/include

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
