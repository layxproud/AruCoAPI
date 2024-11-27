QT += core gui

TEMPLATE = lib
DEFINES += ARUCOAPI_LIBRARY

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    capturethread.cpp \
    arucoapi.cpp \
    yamlhandler.cpp

HEADERS += \
    TestLib_global.h \
    capturethread.h \
    arucoapi.h \
    yamlhandler.h

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/third_party/opencv_mingw810/x64/mingw/lib/ -llibopencv_world4100
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/third_party/opencv_mingw810/x64/mingw/lib/ -llibopencv_world4100d
else:unix: LIBS += -L$$PWD/third_party/opencv_mingw810/x64/mingw/lib/ -llibopencv_world4100.dll

INCLUDEPATH += $$PWD/third_party/opencv_mingw810/include
DEPENDPATH += $$PWD/third_party/opencv_mingw810/include

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
