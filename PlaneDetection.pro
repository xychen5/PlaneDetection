TEMPLATE = subdirs

DESTDIR = $$PWD/build
OUT_PWD = $$PWD/build

DEFINES += EIGEN_DONT_VECTORIZE \
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

SUBDIRS += \
    CoreLib \
    GraphicsLib \
    PointCloudEditor \
    DetectionLib
