
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <stdio.h>
#include <iostream>
#include <functional>
#define NOMINMAX
#include <string.h>
#include <string>
#include <windows.h>
#include <iomanip>
#include <errno.h>
#include <sstream>
#include <locale>
#include <codecvt>
#include <dirent.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;
using namespace std;
#define INVALID INT32_MIN

typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
    float weight[4];
} coordinate_t;

typedef enum
{
    INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
    INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
    INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                 data with value 0 */
} interpolation_t;


void readFrame(char filename[], int height, int width, uint8_t* img_data) {
    FILE* file = fopen(filename, "rb");
    if (file == NULL) {
        printf("Error reading bin file");
    }
    fread(img_data, sizeof(img_data[0]), width * height * 2, file);
    fclose(file);
}

void writeFrame_uint16(char filename[], int height, int width, uint16_t* img_data) {
    FILE* file = fopen(filename, "wb");
    if (file == NULL) {
        printf("Error writing bin file");
    }
    fwrite(img_data, sizeof(img_data[0]), width * height, file);
    fclose(file);
}

void createDirIfNotExists(std::wstring files_path, std::string files_path1) {
    DIR* dir = opendir(files_path1.c_str());
    if (dir) {
        /* Directory exists. */
        closedir(dir);
    }
    else if (ENOENT == errno) {
        CreateDirectory((LPCWSTR)files_path.c_str(), NULL);
    }
    else {
        printf("Failed creating or opening directory");
    }
}

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
static void compute_xy_range(const k4a_calibration_t* calibration,
    const k4a_calibration_type_t camera,
    const int width,
    const int height,
    float& x_min,
    float& x_max,
    float& y_min,
    float& y_max)
{
    // Step outward from the centre point until we find the bounds of valid projection
    const float step_u = 0.25f;
    const float step_v = 0.25f;
    const float min_u = 0;
    const float min_v = 0;
    const float max_u = (float)width - 1;
    const float max_v = (float)height - 1;
    const float center_u = 0.5f * width;
    const float center_v = 0.5f * height;

    int valid;
    k4a_float2_t p;
    k4a_float3_t ray;

    // search x_min
    for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_min = ray.xyz.x;
    }

    // search x_max
    for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_max = ray.xyz.x;
    }

    // search y_min
    for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_min = ray.xyz.y;
    }

    // search y_max
    for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_max = ray.xyz.y;
    }
}

static pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t* calibration, const k4a_calibration_type_t camera)
{
    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        width = calibration->color_camera_calibration.resolution_width;
        height = calibration->color_camera_calibration.resolution_height;
    }

    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

    pinhole_t pinhole;

    float fx = 1.f / (x_max - x_min);
    float fy = 1.f / (y_max - y_min);
    float px = -x_min * fx;
    float py = -y_min * fy;

    pinhole.fx = fx * width;
    pinhole.fy = fy * height;
    pinhole.px = px * width;
    pinhole.py = py * height;
    pinhole.width = width;
    pinhole.height = height;

    return pinhole;
}

static void create_undistortion_lut(const k4a_calibration_t* calibration,
    const k4a_calibration_type_t camera,
    const pinhole_t* pinhole,
    k4a_image_t lut,
    interpolation_t type)
{
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                // Remapping via nearest neighbor interpolation
                src.x = (int)floorf(distorted.xy.x + 0.5f);
                src.y = (int)floorf(distorted.xy.y + 0.5f);
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                // Remapping via bilinear interpolation
                src.x = (int)floorf(distorted.xy.x);
                src.y = (int)floorf(distorted.xy.y);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;

                if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // Compute the floating point weights, using the distance from projected point src to the
                    // image coordinate of the upper left neighbor
                    float w_x = distorted.xy.x - src.x;
                    float w_y = distorted.xy.y - src.y;
                    float w0 = (1.f - w_x) * (1.f - w_y);
                    float w1 = w_x * (1.f - w_y);
                    float w2 = (1.f - w_x) * w_y;
                    float w3 = w_x * w_y;

                    // Fill into lut
                    lut_data[idx].weight[0] = w0;
                    lut_data[idx].weight[1] = w1;
                    lut_data[idx].weight[2] = w2;
                    lut_data[idx].weight[3] = w3;
                }
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}
static void remap_color(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int src_height = k4a_image_get_height_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint8_t* src_data = (uint8_t*)(void*)k4a_image_get_buffer(src);
    uint8_t* dst_data = (uint8_t*)(void*)k4a_image_get_buffer(dst);
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint8_t));

    for (int i = 0; i < dst_width * dst_height * 4; i += 4)
    {
        if (lut_data[(int)i/4].x != INVALID && lut_data[(int)i / 4].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {   
                if ((lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x) * 4 + 1 < src_width * src_height * 4 &&
                    (lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x + 1) * 4 < src_width * src_height * 4 &&
                    ((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x) * 4 < src_width*src_height*4 &&
                    ((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x + 1) * 4 < src_width * src_height * 4) {
                    const uint8_t neighbors_b[4]{ src_data[(lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x) * 4],
                        src_data[(lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x + 1) * 4],
                        src_data[((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x) * 4],
                        src_data[((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x + 1) * 4] };
                    const uint8_t neighbors_g[4]{ src_data[(lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x) * 4 + 1],
                                                    src_data[(lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x + 1) * 4 + 1],
                                                    src_data[((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x) * 4 + 1],
                                                    src_data[((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x + 1) * 4 + 1] };
                    const uint8_t neighbors_r[4]{ src_data[(lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x) * 4 + 2],
                                                    src_data[(lut_data[(int)i / 4].y * src_width + lut_data[(int)i / 4].x + 1) * 4 + 2],
                                                    src_data[((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x) * 4 + 2],
                                                    src_data[((lut_data[(int)i / 4].y + 1) * src_width + lut_data[(int)i / 4].x + 1) * 4 + 2] };
                    //std::cout << i << std::endl;
                    if (neighbors_b[0] == 0 || neighbors_b[1] == 0 || neighbors_b[2] == 0 || neighbors_b[3] == 0 ||
                        neighbors_g[0] == 0 || neighbors_g[1] == 0 || neighbors_g[2] == 0 || neighbors_g[3] == 0 ||
                        neighbors_r[0] == 0 || neighbors_r[1] == 0 || neighbors_r[2] == 0 || neighbors_r[3] == 0)
                    {
                        continue;
                    }

                    dst_data[i] = (uint8_t)(neighbors_b[0] * lut_data[(int)i / 4].weight[0] + neighbors_b[1] * lut_data[(int)i / 4].weight[1] +
                        neighbors_b[2] * lut_data[(int)i / 4].weight[2] + neighbors_b[3] * lut_data[(int)i / 4].weight[3] +
                        0.5f);
                    dst_data[i + 1] = (uint8_t)(neighbors_g[0] * lut_data[(int)i / 4].weight[0] + neighbors_g[1] * lut_data[(int)i / 4].weight[1] +
                        neighbors_g[2] * lut_data[(int)i / 4].weight[2] + neighbors_g[3] * lut_data[(int)i / 4].weight[3] +
                        0.5f);
                    dst_data[i + 2] = (uint8_t)(neighbors_r[0] * lut_data[(int)i / 4].weight[0] + neighbors_r[1] * lut_data[(int)i / 4].weight[1] +
                        neighbors_r[2] * lut_data[(int)i / 4].weight[2] + neighbors_r[3] * lut_data[(int)i / 4].weight[3] +
                        0.5f);
                    dst_data[i + 3] = 0;
                }
                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    /*const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]), std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                        std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }*/
                }

            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}

static void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t* src_data = (uint16_t*)(void*)k4a_image_get_buffer(src);
    uint16_t* dst_data = (uint16_t*)(void*)k4a_image_get_buffer(dst);
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                        std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                    neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                    0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}

static void write_csv_file(const char* file_name, const k4a_image_t src)
{
    FILE* fp = 0;
#ifdef _MSC_VER
    fopen_s(&fp, file_name, "w");
#else
    fp = fopen(file_name, "w");
#endif

    int width = k4a_image_get_width_pixels(src);
    int height = k4a_image_get_height_pixels(src);
    uint16_t* src_data = (uint16_t*)(void*)k4a_image_get_buffer(src);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            fprintf(fp, "%d", src_data[y * width + x]);
            if (x < width - 1)
            {
                fprintf(fp, ",");
            }
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
}

void writeTimeToFile(float time[], std::string path, int framenum) {
    char time_path[255];
    strcpy(time_path, (path + "\\acqTime.bin").c_str());
    FILE* file = fopen(time_path, "wb");
    if (file == NULL) {
        printf("Error in writingTimeToFile\n");
    }
    for (int i = 0; i < framenum; i++) {
        fwrite(&time[i], sizeof(&time[0]), 1, file);
    }
    fclose(file);
}

void saveFrames(std::string str_name, char str[], int iter, std::string files_path, int MAX_ROW, int MAX_COL, uint16_t* array3D) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << iter;
    str_name = ss.str();
    strcpy(str, (files_path + "\\").c_str());
    strcat(str, str_name.c_str());
    strcat(str, ".bin");
    FILE* file = fopen(str, "wb");
    if (file == NULL) {
        printf("Error");
    }
    fwrite(array3D, sizeof(array3D[0]), MAX_COL*MAX_ROW, file);
    fclose(file);
}

void saveFramesColor(std::string str_name, char str[], int iter, std::string files_path, int MAX_ROW, int MAX_COL, uint8_t* array3D) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << iter;
    str_name = ss.str();
    strcpy(str, (files_path + "\\").c_str());
    strcat(str, str_name.c_str());
    strcat(str, ".bin");
    FILE* file = fopen(str, "wb");
    if (file == NULL) {
        printf("Error");
    }
    fwrite(array3D, sizeof(array3D[0]), MAX_COL * MAX_ROW * 4, file);
    fclose(file);
}

void saveFrames_png(std::string str_name, char str[], int iter, std::string files_path, cv::Mat img) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << iter;
    str_name = ss.str();
    strcpy(str, (files_path + "\\").c_str());
    strcat(str, str_name.c_str());
    strcat(str, ".png");
    cv::imwrite(str, img);
}

void saveFramesColor_png(std::string str_name, char str[], int iter, std::string files_path, cv::Mat img) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << iter;
    str_name = ss.str();
    strcpy(str, (files_path + "\\").c_str());
    strcat(str, str_name.c_str());
    strcat(str, ".png");
    cv::imwrite(str, img);
}

static void paintAlphaMat(cv::Mat& mat)
{
    CV_Assert(mat.channels() == 4);
    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j < mat.cols; ++j)
        {
            Vec4b& bgra = mat.at<Vec4b>(i, j);
            bgra[0] = UCHAR_MAX; // Blue
            bgra[1] = saturate_cast<uchar>((float(mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX); // Green
            bgra[2] = saturate_cast<uchar>((float(mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX); // Red
            bgra[3] = saturate_cast<uchar>(0.5 * (bgra[1] + bgra[2])); // Alpha
        }
    }
}

// utility wrapper to adapt locale-bound facets for wstring/wbuffer convert
template <typename Facet>
struct deletable_facet : Facet
{
    using Facet::Facet;
};


int main(int argc, char** argv)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    double* intrinsic_pinhole = new double[6];
    pinhole_t pinhole;
    pinhole_t pinhole_color;
    printf("FRAMECOUNT\n");
    printf("Capture FRAMECOUNT color and depth frames from the device using the separate get frame APIs\n");
    returnCode = 2;
    cv::Mat depth_cv,color_cv, trans_depth_cv;
    k4a_image_t transformed_color_image = NULL;
    k4a_image_t transformed_depth_image = NULL;

    uint32_t device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
    }
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

    char str_filepath[255];
    char str_file[255];
    strcpy(str_filepath, "");
    strcat(str_filepath, ".\\config\\AzureKinectDK_config.txt");
    FILE* fp = fopen(str_filepath, "r");
    fscanf(fp, "%s", str_file);
    char* p = strtok(str_file, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(p, "P");
    int color_width = atoi(p);
    int color_height = 0;
    if (color_width == 720) {
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        color_height = 1280;
    }
    else if (color_width == 2160)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
        color_height = 3840;
    }
    else if (color_width == 1440)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
        color_height = 2560;
    }
    else if (color_width == 1080)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        color_height = 1920;
    }
    else if (color_width == 3072)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
        color_height = 4096;
    }
    else if (color_width == 1536)
    {
        config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
        color_height = 2048;
    }
    else
    {
        printf("Color format not recognized\n");
        return 0;
    }
    fscanf(fp, "%s", str_file);
    p = strtok(str_file, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    int depth_height = 0;
    int depth_width = 0;
    printf(p);
    printf("\n");
    if (!strcmp(p, "NFOV")) {
        p = strtok(NULL, "_");
        if (!strcmp(p, "2X2BINNED")) {
            config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
            depth_height = 288;
            depth_width = 320;
        }
        else if (!strcmp(p, "UNBINNED")) {
            config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
            depth_height = 576;
            depth_width = 640;
        }
        else
        {
            printf("Depth format not recognized\n");
            return 0;
        }
    }
    else if (!strcmp(p, "WFOV")) {
        p = strtok(NULL, "_");
        if (!strcmp(p, "2X2BINNED")) {
            config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
            depth_height = 512;
            depth_width = 512;
        }
        else if (!strcmp(p, "UNBINNED")) {
            config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
            depth_height = 1024;
            depth_width = 1024;
        }
        else
        {
            printf("Depth format not recognized\n");
            return 0;
        }
    }
    else
    {
        printf("Depth format not recognized\n");
        return 0;
    }
    fscanf(fp, "%s", str_file);
    p = strtok(str_file, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    p = strtok(NULL, "_");
    int fps = atoi(p);
    if (fps == 5) {
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    }
    else if (fps == 15) {
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    }
    else if (fps == 30) {
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    }
    else {
        printf("FPS format not recognized\n");
        return 0;
    }
    fscanf(fp, "%s", str_file);
    p = strtok(str_file, ":");
    p = strtok(NULL, ":");
    int captureFrameCount = atoi(p);
    fclose(fp);
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_calibration_t calibration;
    k4a_image_t undistorted_color_image = NULL;
    k4a_image_t undist_trans_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
    }
    k4a_transformation_t transformation = NULL;
    transformation = k4a_transformation_create(&calibration);
    // Generate a pinhole model for depth camera
    pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);
    pinhole_color = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_COLOR);
    intrinsic_pinhole[0] = pinhole_color.fx;
    intrinsic_pinhole[1] = pinhole_color.fy;
    intrinsic_pinhole[2] = pinhole_color.px;
    intrinsic_pinhole[3] = pinhole_color.py;
    intrinsic_pinhole[4] = pinhole_color.width;
    intrinsic_pinhole[5] = pinhole_color.height;
    const char** pinhole_intr_str = new const char* [6];
    for (int i = 0; i < 6; i++) {
        pinhole_intr_str[i] = new const char[255];
    }
    pinhole_intr_str[0] = "fx: ";
    pinhole_intr_str[1] = "fy: ";
    pinhole_intr_str[2] = "cx: ";
    pinhole_intr_str[3] = "cy: ";
    pinhole_intr_str[4] = "width: ";
    pinhole_intr_str[5] = "height: ";
    printf("Capturing %d frames\n", captureFrameCount);
    k4a_capture_t capture = NULL;
    uint64_t* timestamp_depth_array = new uint64_t[captureFrameCount];
    uint64_t* timestamp_ir_array = new uint64_t[captureFrameCount];
    uint64_t* timestamp_color_array = new uint64_t[captureFrameCount];
    for (int i = 0; i < captureFrameCount; i++) {
        timestamp_color_array[i] = 0;
        timestamp_depth_array[i] = 0;
        timestamp_ir_array[i] = 0;
    }
    int counter = 0;
    FILE* fp_depth;
    FILE* fp_color;
    std::string str_name;
    char str[255];
    std::wstring files_path = L"..\\data";
    std::string files_path1 = "..\\data";
    createDirIfNotExists(files_path, files_path1);
    std::string possible_folders[100];
    for (int i = 0; i < 100; i++) {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string s = ss.str();
        char str_possible_folder[255];
        strcpy_s(str_possible_folder, "");
        strcat(str_possible_folder, files_path1.c_str());
        strcat(str_possible_folder, "\\");
        strcat(str_possible_folder, "Acquisition_AzureKinectDK_");
        strcat(str_possible_folder, s.c_str());
        possible_folders[i] = str_possible_folder;
    }
    const char* PATH = files_path1.c_str();
    DIR* dir = opendir(PATH);
    struct dirent* entry = readdir(dir);
    std::string possible_existing_folders[1000];
    int poss_count = 0;
    while (entry != NULL)
    {
        if (entry->d_type == DT_DIR && strcmp(entry->d_name,".") && strcmp(entry->d_name, "..")) {
            char existing_folder[255];
            strcpy(existing_folder, "");
            strcat(existing_folder, files_path1.c_str());
            strcat(existing_folder, "\\");
            strcat(existing_folder, entry->d_name);
            possible_existing_folders[poss_count] = existing_folder;
            poss_count ++;
        }
        entry = readdir(dir);
    }
    closedir(dir);
    int folder_status = 0;
    int idx = 0;
    if (poss_count > 0) {
        for (idx = 0; idx < 1000; idx++) {
            for (int j = 0; j < poss_count; j++) {
                if (!strcmp(possible_existing_folders[idx].c_str(), possible_folders[j].c_str())) {
                    folder_status = 0;
                    break;
                }
                else
                {
                    folder_status = 1;
                }
            }
            if (folder_status == 1) {
                break;
            }
        }
    }
    printf("here");
    std::wstring_convert<
        deletable_facet<std::codecvt<wchar_t, char, std::mbstate_t>>> conv;
    files_path = conv.from_bytes(possible_folders[idx]);
    files_path1 = possible_folders[idx];
    createDirIfNotExists(files_path, files_path1);
    std::wstring color_path = files_path + L"\\color";
    std::wstring depth_path = files_path + L"\\depth";
    std::string color_path1 = files_path1 + "\\color";
    std::string depth_path1 = files_path1 + "\\depth";
    createDirIfNotExists(files_path, files_path1);
    createDirIfNotExists(color_path, color_path1);
    createDirIfNotExists(depth_path, depth_path1);
    uint16_t* depth = new uint16_t[depth_width * depth_height];
    uint16_t* depth_cv_buffer = new uint16_t[depth_width * depth_height];
    uint16_t* trans_depth_cv_buffer = new uint16_t[color_width * color_height];
    char params_path[255];
    strcpy(params_path, "");
    strcat(params_path, files_path1.c_str());
    strcat(params_path, "\\");
    strcat(params_path, "\\sensor_params.txt");
    fp = fopen(str_filepath, "r");
    FILE* fp_w = fopen(params_path, "w");
    fscanf(fp, "%s", str_file);
    fprintf(fp_w, "%s\n", str_file);
    fscanf(fp, "%s", str_file);
    fprintf(fp_w, "%s\n", str_file);
    fscanf(fp, "%s", str_file);
    fprintf(fp_w, "%s\n", str_file);
    fscanf(fp, "%s", str_file);
    fprintf(fp_w, "%s\n", str_file);
    fclose(fp);
    fclose(fp_w);
    char intrinsics_str[255];
    strcpy(intrinsics_str, "");
    strcat(intrinsics_str, files_path1.c_str());
    strcat(intrinsics_str, "\\camera_intrinsics.txt");
    fp = fopen(intrinsics_str, "w");
    for (int i = 0; i < 6; i++) {
        fprintf(fp, pinhole_intr_str[i]);
        fprintf(fp, "%.6f", intrinsic_pinhole[i]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    int frame_counter = 0;
    k4a_image_t lut = NULL;
    k4a_image_t lut_color = NULL;
    k4a_image_t undistorted = NULL;
    k4a_image_t trans_depth_undistorted = NULL;
    interpolation_t interpolation_type = INTERPOLATION_BILINEAR_DEPTH;
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start device\n");
    }
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        pinhole_color.width,
        pinhole_color.height,
        pinhole_color.width* (int)sizeof(coordinate_t),
        &lut_color);
    interpolation_type = INTERPOLATION_BILINEAR;
    create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_COLOR, &pinhole_color, lut_color, interpolation_type);
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        pinhole.width,
        pinhole.height,
        pinhole.width * (int)sizeof(coordinate_t),
        &lut);
    interpolation_type = INTERPOLATION_BILINEAR_DEPTH;
    create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_DEPTH, &pinhole, lut, interpolation_type);
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        pinhole.width,
        pinhole.height,
        pinhole.width * (int)sizeof(uint16_t),
        &trans_depth_undistorted);
    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        pinhole_color.width,
        pinhole_color.height,
        pinhole_color.width* (int)sizeof(uint8_t) * 4,
        &undist_trans_color_image);
    while (captureFrameCount-- > 0)
    {
        k4a_image_t image_color;
        k4a_image_t image_depth;
        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            counter = counter + 1;
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            goto Exit;
        }
        printf("Capture");
        image_color = k4a_capture_get_color_image(capture);
        if (image_color)
        {
            timestamp_color_array[counter-1] = k4a_image_get_device_timestamp_usec(image_color);
            uint8_t* image_data = k4a_image_get_buffer(image_color);
            cv::Mat color_frame = cv::Mat(k4a_image_get_height_pixels(image_color), k4a_image_get_width_pixels(image_color), CV_8UC4, image_data, cv::Mat::AUTO_STEP);
            printf(" | Color timestamp: %10d", timestamp_color_array[counter-1]);
        }
        else
        {
            printf(" | Color None                       ");
            timestamp_color_array[counter-1] = 0;
        }
        image_depth = k4a_capture_get_depth_image(capture);
        if (image_depth != NULL && image_color != NULL)
        {
            timestamp_depth_array[counter-1] = k4a_image_get_device_timestamp_usec(image_depth);
            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                k4a_image_get_width_pixels(image_color),
                k4a_image_get_height_pixels(image_color),
                k4a_image_get_width_pixels(image_color) * (int)sizeof(uint16_t),
                &transformed_depth_image))
            {
                printf("Failed to create transformed color image\n");
                return false;
            }
            /*if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation,
                image_depth,
                transformed_depth_image))
            {
                printf("Failed to compute transformed depth image\n");
                return false;
            }*/
            /*if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation,
                image_depth,
                transformed_depth_image))
            {
                printf("Failed to compute transformed color image\n");
                return false;
            }*/
            //remap(transformed_depth_image, lut_color, trans_depth_undistorted, interpolation_type);
            remap(image_depth, lut, trans_depth_undistorted, interpolation_type);
            remap_color(image_color, lut_color, undist_trans_color_image, interpolation_type);
            uint8_t* trans_depth_buffer = k4a_image_get_buffer(trans_depth_undistorted);
            size_t trans_depth_buffer_size = k4a_image_get_size(trans_depth_undistorted);
            printf(" | depth timestamp: %10d\n", timestamp_depth_array[counter - 1]);
            int counter2 = 0;
            for (size_t i = 0; i < trans_depth_buffer_size; i += 2)
            {
                depth[counter2] = (uint16_t)(trans_depth_buffer[i + 1] << 8 | trans_depth_buffer[i]);
                depth_cv_buffer[counter2] = (uint16_t)(trans_depth_buffer[i + 1] << 8 | trans_depth_buffer[i]);
                depth_cv_buffer[counter2] *= 10;
                counter2 = counter2 + 1;
            }
            trans_depth_cv = cv::Mat(depth_height, depth_width, CV_16U, depth_cv_buffer, cv::Mat::AUTO_STEP);
            uint8_t* undist_trans_color_image_data = k4a_image_get_buffer(undist_trans_color_image);
            size_t trans_color_buffer_size = k4a_image_get_size(undist_trans_color_image);
            cv::Mat undist_trans_color_frame = cv::Mat(k4a_image_get_height_pixels(undist_trans_color_image), k4a_image_get_width_pixels(undist_trans_color_image),
                CV_8UC4, undist_trans_color_image_data, cv::Mat::AUTO_STEP);
            //cv::Mat converted;
            cvtColor(undist_trans_color_frame, undist_trans_color_frame, cv::COLOR_RGB2BGR);
            cvtColor(undist_trans_color_frame, undist_trans_color_frame, cv::COLOR_BGR2RGB);
            //cv::imshow("depth", depth_cv);
            //paintAlphaMat(undist_trans_color_frame);
            saveFramesColor_png(str_name, str, counter - 1, color_path1, undist_trans_color_frame);
            saveFrames_png(str_name, str, counter - 1, depth_path1, trans_depth_cv);
            cv::imshow("depth_aligned", trans_depth_cv);
            cv::imshow("transf color", undist_trans_color_frame);
            //cv::imshow("transf color", undist_color_frame);

            int key = cv::waitKey(1);
            
            if (key == 27) {
                break;
            }

            //saveFramesColor(str_name, str, counter - 1, color_path1, color_height, color_width, undist_trans_color_image_data);
            //saveFrames(str_name, str, counter-1, depth_path1, color_height, color_width, depth);
        }
        else
        {
            printf(" | Depth16 None\n");
            timestamp_depth_array[counter-1] = 0;
        }
        k4a_image_release(image_color);
        k4a_image_release(image_depth);
        // release capture
        k4a_capture_release(capture);
        fflush(stdout);
    }
    char depth_timestamp_file_path[255];
    strcpy(depth_timestamp_file_path, "");
    strcat(depth_timestamp_file_path, files_path1.c_str());
    strcat(depth_timestamp_file_path, "\\");
    strcat(depth_timestamp_file_path, "depth_timestamp.txt");
    fp_depth = fopen(depth_timestamp_file_path, "w");
    for (int i = 0; i < counter; i++) {
        fprintf(fp_depth, "%4d\n", timestamp_depth_array[i]);
    }
    fclose(fp_depth);
    char color_timestamp_file_path[255];
    strcpy(color_timestamp_file_path, "");
    strcat(color_timestamp_file_path, files_path1.c_str());
    strcat(color_timestamp_file_path, "\\");
    strcat(color_timestamp_file_path, "color_timestamp.txt");
    fp_color = fopen(color_timestamp_file_path, "w");
    for (int i = 0; i < counter; i++) {
        fprintf(fp_color, "%4d\n", timestamp_color_array[i]);
    }
    fclose(fp_color);
    cv::destroyAllWindows();
    returnCode = 0;
Exit:
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    
    return returnCode;
}