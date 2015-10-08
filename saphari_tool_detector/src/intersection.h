#include <opencv2/opencv.hpp>

namespace cv
{

enum RectanglesIntersectTypes {
    INTERSECT_NONE = 0, //!< No intersection
    INTERSECT_PARTIAL  = 1, //!< There is a partial intersection
    INTERSECT_FULL  = 2 //!< One of the rectangle is fully enclosed in the other
};

int rotatedRectangleIntersection(const RotatedRect& rect1, const RotatedRect& rect2, OutputArray intersectingRegion);

}
