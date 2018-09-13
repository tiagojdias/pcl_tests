#include <pcl/range_image/range_image.h>
#include <elapse_timer.hpp>

int main (int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    prodrone::ElapseTimer ep_0;
    // Generate the data
    for (float y=-2.5f; y<=2.5f; y+=0.01f)
    {
        for (float z=-2.5f; z<=2.5f; z+=0.01f)
        {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point);
        }
    }
    ep_0.printElapsed();
    pointCloud.width = (uint32_t) pointCloud.points.size();
    pointCloud.height = 1;

    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    prodrone::ElapseTimer ep_1;
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    ep_1.printElapsed();

    std::cout << rangeImage << "\n";
}