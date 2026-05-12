#ifndef FRAMEHELPERTYPES_H
#define FRAMEHELPERTYPES_H

namespace frame_helper
{
    enum ResizeAlgorithm
    {
        INTER_LINEAR,
        INTER_NEAREST,
        INTER_AREA,
        INTER_CUBIC,
        INTER_LANCZOS4,
        BAYER_RESIZE
    };
    enum UndistortAlgorithm
    {
        UNDISTORT_LINEAR,
        UNDISTORT_NEAREST,
        UNDISTORT_AREA,
        UNDISTORT_CUBIC,
        UNDISTORT_LANCZOS4
    };
};
#endif
