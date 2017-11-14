#include <librealsense/rs.hpp>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// NOTE REQUIRES OPENCV 2.4.13 - 2.4.9 DOES NOT READ THE DEPTH IMAGE FROM FILE AS UINT16 LIKE IT SHOULD


namespace rs
{
  /// \brief Video stream intrinsics
  struct my_intrinsics : rs_intrinsics
  {
      float       hfov() const                                                        { return (atan2f(ppx + 0.5f, fx) + atan2f(width - (ppx + 0.5f), fx)) * 57.2957795f; }
      float       vfov() const                                                        { return (atan2f(ppy + 0.5f, fy) + atan2f(height - (ppy + 0.5f), fy)) * 57.2957795f; }
      distortion  my_model() const                                                       { return (distortion)rs_intrinsics::model; }

                  // Helpers for mapping between pixel coordinates and texture coordinates
      float2      pixel_to_texcoord(const float2 & pixel) const                       { return {(pixel.x+0.5f)/width, (pixel.y+0.5f)/height}; }
      float2      texcoord_to_pixel(const float2 & coord) const                       { return {coord.x*width - 0.5f, coord.y*height - 0.5f}; }

                  // Helpers for mapping from image coordinates into 3D space
      float3      deproject(const float2 & pixel, float depth) const                  { float3 point = {}; rs_deproject_pixel_to_point(&point.x, this, &pixel.x, depth); return point; }
      float3      deproject_from_texcoord(const float2 & coord, float depth) const    { return deproject(texcoord_to_pixel(coord), depth); }

                  // Helpers for mapping from 3D space into image coordinates
      float2      project(const float3 & point) const                                 { float2 pixel = {}; rs_project_point_to_pixel(&pixel.x, this, &point.x); return pixel; }
      float2      project_to_texcoord(const float3 & point) const                     { return pixel_to_texcoord(project(point)); }

      bool        operator == (const intrinsics & r) const                            { return memcmp(this, &r, sizeof(r)) == 0; }

  };
}


int main(int argc, char** argv) try
{
    if( argc != 3)
    {
        std::cout << "Usage: ./align_depth_to_rgb depth_image aligned_depth_image_name.png" << std::endl;
        return -1;
    }

    cv::Mat depth_image_cv;
    depth_image_cv = cv::imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH);

    // Fill pre-captured camera parameters for mapping between depth and color
    rs::my_intrinsics depth_intrin;
    rs::extrinsics depth_to_color;
    rs::my_intrinsics color_intrin;

    depth_intrin.model = RS_DISTORTION_INVERSE_BROWN_CONRADY;
    // depth_intrin.model = RS_DISTORTION_NONE;
    color_intrin.model = RS_DISTORTION_NONE;

    float scale = 0.00012498664727900;
    depth_intrin.height = 480;
    depth_intrin.width = 640;
    depth_intrin.fx = 475.96459960937500000;
    depth_intrin.fy = 475.96447753906250000;
    depth_intrin.ppx = 321.28894042968750000;
    depth_intrin.ppy = 245.87298583984375000;
    depth_intrin.coeffs[0] = 0.14374029636383057;
    depth_intrin.coeffs[1] = 0.04620679095387459;
    depth_intrin.coeffs[2] = 0.00425983592867851;
    depth_intrin.coeffs[3] = 0.00561223411932588;
    depth_intrin.coeffs[4] = 0.10718701034784317;
    // depth_intrin.coeffs[0] = 0.00000000000000000;
    // depth_intrin.coeffs[1] = 0.00000000000000000;
    // depth_intrin.coeffs[2] = 0.00000000000000000;
    // depth_intrin.coeffs[3] = 0.00000000000000000;
    // depth_intrin.coeffs[4] = 0.00000000000000000;
    color_intrin.height = 480;
    color_intrin.width = 640;
    color_intrin.fx=603.6311147583593311;
    color_intrin.fy=604.4090047756201329;
    color_intrin.ppx=321.9590675957765598;
    color_intrin.ppy=237.7862899820069060;
    color_intrin.height = 360;
    color_intrin.width = 640;
    color_intrin.fx = 1386.69458007812500000/3.0;
    color_intrin.fy = 1386.69482421875000000/3.0;
    color_intrin.ppx = 945.50714111328125000/3.0;
    color_intrin.ppy = 545.70349121093750000/3.0;
    color_intrin.coeffs[0] = 0.00000000000000000;
    color_intrin.coeffs[1] = 0.00000000000000000;
    color_intrin.coeffs[2] = 0.00000000000000000;
    color_intrin.coeffs[3] = 0.00000000000000000;
    color_intrin.coeffs[4] = 0.00000000000000000;
    depth_to_color.rotation[0] = 0.99999058246612549;
    depth_to_color.rotation[1] = -0.00421905471011996;
    depth_to_color.rotation[2] = 0.00097743538208306;
    depth_to_color.rotation[3] = 0.00422254949808121;
    depth_to_color.rotation[4] = 0.99998456239700317;
    depth_to_color.rotation[5] = -0.00360147259198129;
    depth_to_color.rotation[6] = -0.00096222554566339;
    depth_to_color.rotation[7] = 0.00360556622035801;
    depth_to_color.rotation[8] = 0.99999302625656128;
    depth_to_color.translation[0] = 0.02569999732077122;
    depth_to_color.translation[1] = 0.00056902528740466;
    depth_to_color.translation[2] = 0.00396214984357357;

    // REVERSE THE SCALING!!!
    cv::Mat depth_image_cv_in_scaled(depth_intrin.height, depth_intrin.width, CV_64FC1);

    depth_image_cv.convertTo(depth_image_cv_in_scaled, CV_64FC1);
    depth_image_cv_in_scaled /= (scale*65536.0/2.0);
    depth_image_cv_in_scaled.convertTo(depth_image_cv, CV_16UC1);

    cv::Mat depth_in_color_frame(color_intrin.height, color_intrin.width, CV_16UC1);

    // Pre-fill image with 0's to account for pixels that fall outside of depth image sweep
    for (int h=0;h<color_intrin.height;h++) {
        for (int w=0;w<color_intrin.width;w++) {
            depth_in_color_frame.at<uint16_t>(cv::Point(w,h)) = 0;
        }
    }

    for (int dy=0; dy<depth_intrin.height; ++dy){
        for (int dx=0; dx<depth_intrin.width; ++dx){
            // Obtain the depth value and apply scale factor
            uint16_t depth_value = depth_image_cv.at<uint16_t>(cv::Point(dx,dy));
            float depth_in_meters = depth_value * scale;

            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            rs::float2 depth_pixel = {(float)dx, (float)dy};
            rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
            rs::float3 color_point = depth_to_color.transform(depth_point);
            rs::float2 color_pixel = color_intrin.project(color_point);

            // Choose the nearest pixel in the color image to the reprojected depth pixel
            // Ignore this point if it falls outside the color image
            const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
            if (!(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height))
            {
                depth_in_color_frame.at<uint16_t>(cv::Point(cx,cy)) = depth_value;
            }
        }
    }

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    cv::Mat depth_image_cv_out(depth_intrin.height, depth_intrin.width, CV_16UC1);
    cv::Mat depth_image_cv_scaled(depth_intrin.height, depth_intrin.width, CV_64FC1);

    depth_in_color_frame.convertTo(depth_image_cv_scaled, CV_64FC1);
    depth_image_cv_scaled *= scale*65536.0/2.0;
    depth_image_cv_scaled.convertTo(depth_image_cv_out, CV_16UC1);

    // Depth values are in: (metres * 65536/2)
    // CV_16UC1 range is 0-65536, camera depth range is 0.3 to 2.0 m

    std::stringstream depth_fn;
    depth_fn << argv[2];
    cv::imwrite(depth_fn.str(), depth_image_cv_out, compression_params);

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
