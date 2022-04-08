TEMPLATE = subdirs

# DESTDIR = $$PWD/build
# OUT_PWD = $$PWD/build
CONFIG += console
DEFINES += EIGEN_DONT_VECTORIZE \
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

SUBDIRS += \
    CoreLib \
    GraphicsLib \
    PointCloudEditor \
    DetectionLib

message(allPro: outpwd is : $$OUT_PWD)
message(allPro: pwd is : $$PWD)
message(allPro: dest is : $$DESTDIR)
