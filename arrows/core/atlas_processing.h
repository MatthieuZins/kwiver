#ifndef KWIVER_ARROWS_CORE_ATLAS_PROCESSING_H
#define KWIVER_ARROWS_CORE_ATLAS_PROCESSING_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <arrows/core/mesh_uv_parameterization.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/types/camera.h>
#include <vital/types/matrix.h>
#include <vital/types/camera_rpc.h>
#include <vital/types/geodesy.h>

using namespace kwiver::vital;

namespace kwiver {
namespace arrows {
namespace core {

KWIVER_ALGO_CORE_EXPORT
bool is_point_inside_triangle(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c);
KWIVER_ALGO_CORE_EXPORT
Eigen::Vector3d barycentric_coordinates(const Eigen::Vector2d& p,
                                        const Eigen::Vector2d& a,
                                        const Eigen::Vector2d& b,
                                        const Eigen::Vector2d& c);
KWIVER_ALGO_CORE_EXPORT
kwiver::vital::image_container_sptr generate_triangles_map(const kwiver::arrows::core::uv_parameterization_t& param, int exterior_margin);


template <class T>
void dilate_atlas(image& atlas, image_of<bool> _mask, int nb_iter);


template <class T>
T bilinear_interpolation(const vital::image& image,  double u, double v, unsigned int depth);


template<class T>
KWIVER_ALGO_CORE_EXPORT
std::tuple<image_container_sptr, image_container_sptr, image_container_sptr>
rasterize(mesh_sptr mesh, const uv_parameterization_t& param,
          image_container_sptr triangles_id_map, image_container_sptr image,
          camera_sptr camera, image_container_sptr depthmap, const std::vector<float>& faces_rating)
{
    unsigned int nb_vertices = mesh->num_verts();

    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
    vital::mesh_regular_face_array<3>& faces = dynamic_cast< vital::mesh_regular_face_array<3>& >(mesh->faces());
    // project all points on image
    std::vector<Eigen::Vector2d> points_image(nb_vertices);
    if (dynamic_cast<camera_rpc*>(camera.get()))
    {
        // RPC cameras expect points in lat/long coordinates
        unsigned int utm_zone = dynamic_cast<camera_rpc*>(camera.get())->get_utm_zone();
        for (unsigned int i=0; i < nb_vertices; ++i)
        {
            vital::vector_2d pt2d_latlong = vital::geo_conv({vertices[i][0], vertices[i][1]},
                                                            kwiver::vital::SRID::UTM_WGS84_north + utm_zone,
                                                            kwiver::vital::SRID::lat_lon_WGS84);
            points_image[i] = camera->project({pt2d_latlong[0], pt2d_latlong[1], vertices[i][2]});
        }
    }
    else
    {
        for (unsigned int i=0; i < nb_vertices; ++i)
        {
            points_image[i] = camera->project(vertices[i]);
        }
    }

    // Compute the points depth
    std::vector<double> points_depth(nb_vertices);
    for (unsigned int i=0; i < vertices.size(); ++i)
    {
        points_depth[i] = camera->depth(vertices[i]);
    }

    std::cout << "points projected and depth computed" << std::endl;
//    std::vector<Eigen::Vector2d> uvs_in_sun;
//    std::vector<double> points_sun_depth(nb_vertices);
//    if (!sun_depth_map.empty() && sun_camera)     // only if shadow is enabled
//    {
//        // project points in the sun depthmap using the camera
//        sun_camera->project_points(vertices, uvs_in_sun);

//        // Compute the points depth relative to the sun camera
//        std::pair<Eigen::Vector3d, Eigen::Vector3d> sun_axis_and_position = sun_camera->get_optical_axis_and_position(utm_zone);
//        Eigen::Vector3d sun_axis = sun_axis_and_position.first;
//        Eigen::Vector3d sun_position = sun_axis_and_position.second;    // a possible camera position
//        for (int i=0; i < vertices.size(); ++i)
//        {
//            points_sun_depth[i] = sun_axis.dot(vertices[i] - sun_position);
//        }
//    }

    size_t width = triangles_id_map->width();
    size_t height = triangles_id_map->height();
    size_t depth = image->depth();
    image_of<T> texture(width, height, depth);
    image_of<unsigned char> visibility(width, height, 1);
    image_of<unsigned char> shadow(width, height, 1);
    image_of<float> scores(width, height, 1);

    for (unsigned int i=0; i < height; ++i)
    {
        for (unsigned int j=0; j < width; ++j)
        {
            for (unsigned int d=0; d < depth; ++d)
            {
                texture(j, i, d) = 0.0;
            }
            visibility(j, i) = 1;
            shadow(j, i) = 0;
            scores(j, i) = -1;
        }
    }
    // texture
//    cv::Mat texture(id_map.size(), CV_16UC(nb_bands), 0.0);
//    cv::Mat visibility(id_map.size(), CV_8UC1, 0.0);
//    cv::Mat shadow (id_map.size(), CV_8UC1, 0.0);
//    cv::Mat score(id_map.size(), CV_32FC1, -1.0);
//    cv::Mat normals(id_map.size(), CV_32FC3, 0.0);

    for (unsigned int v=0; v < height; ++v)
    {
        for (unsigned int u=0; u < width; ++u)
        {
            int face_id = triangles_id_map->get_image().at<int>(u, v);
            if (face_id < 0)
            {
                continue; // empty pixel (no texture)
            }
            // Get the uvs of the corresponding face
            const Eigen::Vector2d& a_uv = param.tcoords[param.face_mapping[face_id][0]];
            const Eigen::Vector2d& b_uv = param.tcoords[param.face_mapping[face_id][1]];
            const Eigen::Vector2d& c_uv = param.tcoords[param.face_mapping[face_id][2]];

            unsigned int A_id = faces(face_id, 0);
            unsigned int B_id = faces(face_id, 1);
            unsigned int C_id = faces(face_id, 2);

            Eigen::Vector2d p(u, v);
            Eigen::Vector3d bary_coords = barycentric_coordinates(p, a_uv, b_uv, c_uv);

            Eigen::Vector2d p_img = points_image[A_id] * bary_coords[0] +
                                    points_image[B_id] * bary_coords[1] +
                                    points_image[C_id] * bary_coords[2];
            // Check that the point is inside the image, otherwise ignore it
            if (p_img[0] < 0 || p_img[0] > image->width()-1||
                p_img[1] < 0 || p_img[1] > image->height()-1)
            {
                // texture, visibility, shadow remain at 0
                // score gets 0
                scores(u, v) = 0;
                continue;
            }

            // Generate scores for this pixel
            // the score is the ratio between the area (in sq. pixels) of the projected face
            // and the area of the face in the mesh/texture atlas (converted in sq. pixels with the resolution used)
            scores(u, v) = faces_rating[face_id] > 0 ? faces_rating[face_id] : 0;


            // Occlusions
            // compute the real depth of a point
            double point_depth = points_depth[A_id] * bary_coords[0] +
                                 points_depth[B_id] * bary_coords[1] +
                                 points_depth[C_id] * bary_coords[2];

            bool is_occluded = true;
            if (depthmap.get())
            {
                double depth_threshold = 0.07;
                // here we check the 4 nearest pixels of the point on the depthmap
                Eigen::Vector2i p_floor(floor(p_img[0]), floor(p_img[1]));
                double d0 = depthmap->get_image().at<double>(p_floor[0], p_floor[1]);
                double d1 = depthmap->get_image().at<double>(p_floor[0]+1, p_floor[1]);
                double d2 = depthmap->get_image().at<double>(p_floor[0], p_floor[1]+1);
                double d3 = depthmap->get_image().at<double>(p_floor[0]+1, p_floor[1]+1);
                // if the real value is close to one of the for values from the depthmap,
                // we consider the point as non occluded
                if (fabs(point_depth - d0) < depth_threshold)
                {
                    is_occluded = false;
                }
                else if (fabs(point_depth - d1) < depth_threshold)
                {
                    is_occluded = false;
                }
                else if (fabs(point_depth - d2) < depth_threshold)
                {
                    is_occluded = false;
                }
                else if (fabs(point_depth - d3) < depth_threshold)
                {
                    is_occluded = false;
                }
                else
                {   // compute the min and max depth of these 4 neighbours
                    // if the real depth is between min and max -> the point is considered non occluded
                    // if the min depth is std::numeric_limits<double>::max() (equivalent to no-data), the point is considered non occluded
                    // (this was added to handle the case where the view angle is almost parallel
                    //  to the wall which represent only a few pixel on the depthmap)
                    double min_depth = std::min(d0, std::min(d1, std::min(d2, d3)));
                    double max_depth = std::max(d0, std::max(d1, std::max(d2, d3)));
                    if ((std::abs(min_depth - std::numeric_limits<double>::max()) < 0.0001
                         && max_depth > -std::numeric_limits<double>::max())
                            || (point_depth >= min_depth && point_depth <= max_depth))
                    {
                        is_occluded = false;
                    }
                }
            }
            else
            {
                is_occluded = false;
            }


            // Shadows (same method as occlusions)
//            bool is_in_shadow = true;
//            if (!sun_depth_map.empty())
//            {
//                Eigen::Vector2d p_sun_img = uvs_in_sun[A_id] * bary_coords[0] +
//                                            uvs_in_sun[B_id] * bary_coords[1] +
//                                            uvs_in_sun[C_id] * bary_coords[2];
//                if (p_sun_img[0] < 0 || p_sun_img[0] > sun_depth_map.cols-1 ||
//                    p_sun_img[1] < 0 || p_sun_img[1] > sun_depth_map.rows-1)
//                {
//                    is_in_shadow = false;
//                }
//                else
//                {
//                    double point_sun_depth = points_sun_depth[A_id] * bary_coords[0] +
//                                             points_sun_depth[B_id] * bary_coords[1] +
//                                             points_sun_depth[C_id] * bary_coords[2];
//                    // here we check the 4 nearest pixels of the point on the depthmap
//                    Eigen::Vector2i p_floor(floor(p_sun_img[0]), floor(p_sun_img[1]));
//                    double d0 = sun_depth_map.at<double>(p_floor[1], p_floor[0]);
//                    double d1 = sun_depth_map.at<double>(p_floor[1], p_floor[0]+1);
//                    double d2 = sun_depth_map.at<double>(p_floor[1]+1, p_floor[0]);
//                    double d3 = sun_depth_map.at<double>(p_floor[1]+1, p_floor[0]+1);
//                    // if the real value is close to one of the for values from the depthmap,
//                    // we consider the point as non occluded
//                    if (fabs(point_sun_depth - d0) < 0.5)
//                    {
//                        is_in_shadow = false;
//                    }
//                    else if (fabs(point_sun_depth - d1) < 0.5)
//                    {
//                        is_in_shadow = false;
//                    }
//                    else if (fabs(point_sun_depth - d2) < 0.5)
//                    {
//                        is_in_shadow = false;
//                    }
//                    else if (fabs(point_sun_depth - d3) < 0.5)
//                    {
//                        is_in_shadow = false;
//                    }
//                    else
//                    {
//                        double min_depth = std::min(d0, std::min(d1, std::min(d2, d3)));
//                        double max_depth = std::max(d0, std::max(d1, std::max(d2, d3)));
//                        if (min_depth == std::numeric_limits<double>::max() ||
//                                point_sun_depth >= min_depth && point_sun_depth <= max_depth)
//                        {
//                            is_in_shadow = false;
//                        }
//                    }
//                }
//            }
//            else
//            {   // shadows are disabled
//                is_in_shadow = false;
//            }


            // Score computation:
            //  We take the absolute value to handle wrong oriented faces
            // - score = 0 : the face area in the image is 0. In this case,
            //               the pixel is considered as occluded
            // - score > 0 : the face area is not null, either positive
            //               or negative depending on the face orientation
            // score = -1 (default value for the score image) when the pixel
            // is outside all the triangles. This kind of pixels are skipped.
//            pixel_score = std::abs(pixel_score);
//            score.at<float>(v, u) = pixel_score;
            if (is_occluded)
            {
                // by default, occluded faces get 0 for every band
                for (unsigned int b=0; b < depth; ++b)
                {
                    texture(u, v) = 0;
                }
                visibility(u, v) = 0;
            }
//            else if (is_in_shadow)
//            {
//                score.at<float>(v, u) /= 2;
//            }
            else
            {
                for (unsigned int b=0; b < depth; ++b)
                {
                    texture(u, v, b) = bilinear_interpolation<T>(image->get_image(), p_img[0], p_img[1], b);
//                    texture(u, v, b) = image->get_image().at<T>(p_img[0], p_img[1], b);
                }
            }

        }
    }

    // create the mask used for dilation
    //  0 -> outside triangles
    //  1 -> inside triangles
//    cv::Mat mask = cv::min(0, triangles_id_map);      // between -1 and 0
//    mask.convertTo(mask, CV_8U, 1.0, 1);              // between 0 and 1
    image_of<bool> mask(width, height);
    for (unsigned int v=0; v < height; ++v)
    {
        for (unsigned int u=0; u < width; ++u)
        {
            mask(u, v) = triangles_id_map->get_image().at<int>(u, v) >= 0 ? 1 : 0;
        }
    }

    // dilate texture
    dilate_atlas<T>(texture, mask, 2);
    dilate_atlas<unsigned char>(visibility, mask, 2);
    dilate_atlas<unsigned char>(shadow, mask, 2);
    dilate_atlas<float>(scores, mask, 2);

    return std::tuple<image_container_sptr, image_container_sptr, image_container_sptr>(
                image_container_sptr(new simple_image_container(texture)),
                image_container_sptr(new simple_image_container(visibility)),
                image_container_sptr(new simple_image_container(scores)));
}



/**
 * @brief dilate_texture This functions performs a dilation horizontally
 * and vertically
 * @param texture [in/out] image to dilate
 * @param mask [in] binary mask used for dilation
 * @param nb_iter [in] the dilation is repeated nb_iter times
 */
template <class T>
void dilate_atlas(image& atlas, image_of<bool> _mask, int nb_iter)
{
    image_of<bool> mask;
    mask.copy_from(_mask);

    size_t height = atlas.height();
    size_t width = atlas.width();
    size_t depth = atlas.depth();

    for (int n=0; n<nb_iter; ++n)
    {
        image_of<bool> tmp;
        tmp.copy_from(mask);

        // vertically
        for (int r=0; r < height; ++r)
        {
            for (int c=0; c < width; ++c)
            {
                if (mask(c, r) == false)
                {
                    std::vector<T> values(depth, 0);
                    unsigned int nb = 0;
                    if ((r-1) >= 0 && mask(c, r-1))
                    {
                        for (int d=0; d < depth; ++d)
                        {
                            values[d] += atlas.at<T>(c, r-1, d);
                        }
                        nb++;
                    }
                    if ((r+1) < height && mask(c, r+1))
                    {
                        for (int d=0; d < depth; ++d)
                        {
                            values[d] += atlas.at<T>(c, r+1, d);
                        }
                        nb++;
                    }
                    if (nb > 0)
                    {
                        for (int d=0; d < depth; ++d)
                        {
                            atlas.at<T>(c, r, d) = static_cast<float>(values[d]) / nb;
                        }
                        tmp(c, r) = true;
                    }
                }
            }
        }

        // horizontally
        for (int r=0; r < height; ++r)
        {
            for (int c=0; c < width; ++c)
            {
                if (mask(c, r) == false)
                {
                    std::vector<T> values(depth, 0);
                    unsigned int nb = 0;
                    if ((c-1) >= 0 && mask(c-1, r))
                    {
                        for (int d=0; d < depth; ++d)
                        {
                            values[d] += atlas.at<T>(c-1, r, d);
                        }
                        nb++;
                    }
                    if ((c+1) < width && mask(c+1, r))
                    {
                        for (int d=0; d < depth; ++d)
                        {
                            values[d] += atlas.at<T>(c+1, r, d);
                        }
                        nb++;
                    }
                    if (nb > 0)
                    {
                        for (int d=0; d < depth; ++d)
                        {
                            atlas.at<T>(c, r, d) = static_cast<float>(values[d]) / nb;
                        }
                        tmp(c, r) = true;
                    }
                }
            }
        }
        mask.copy_from(tmp);
    }
}

template <class T>
T bilinear_interpolation(const vital::image& image,  double u, double v, unsigned int depth)
{
   // bilateral interpolation
   int rounded_u = static_cast<int>(std::round(u));
   int rounded_v = static_cast<int>(std::round(v));
   if (rounded_u >= 0 && rounded_u < image.width()
           && rounded_v >= 0 && rounded_v < image.height())
   {
       // if on image border => no interpolation
       if (u < 0 || v < 0 || u > image.width()-1 || v > image.height()-1)
       {
           return image.at<T>(rounded_u, rounded_v, depth);
       }

       int x1 = static_cast<int>(std::floor(u));
       int y1 = static_cast<int>(std::floor(v));

       double dx = u-x1;
       double dy = v-y1;

       // cast to Vec<int, nb_bands> because of the substractions done in the interpolation

       double Q11, Q21, Q12, Q22;

       Q11 = image.at<T>(x1, y1, depth);
       Q21 = image.at<T>(x1+1, y1, depth);
       Q12 = image.at<T>(x1, y1+1, depth);
       Q22 = image.at<T>(x1+1, y1+1, depth);
       return static_cast<T>(Q11 + (Q21 - Q11) * dx + (Q12 - Q11) * dy + (Q11 + Q22 - Q12 - Q21) * dx * dy);
   }
   return  0;
}

template <class T>
image_container_sptr fuse_texture_atlases(image_container_sptr_list textures,
                                          image_container_sptr_list visibilities,
                                          image_container_sptr_list scores,
                                          int fusion_method)
{
    int width = textures[0]->width();
    int height = textures[0]->height();
    int depth = textures[0]->depth();
    image_of<T> output(width, height, depth);
    for (int i=0; i < height; ++i)
    {
        for (int j=0; j < width; ++j)
        {
            for (int d=0; d < depth; ++d)
            {
                output(j, i, d) = 0;
            }
        }
    }
    unsigned int nb_textures = textures.size();
    std::vector<float> pixel_scores(nb_textures, 0.0);


    for (int v=0; v < height; ++v)
    {
        for (int u=0; u < width; ++u)
        {
            if (scores[0]->get_image().at<float>(u, v) < 0)
            {
                continue;   // score < 0 means that the pixel is empty (outside every triangle)
            }

            for (int id=0; id < nb_textures; ++id)
            {
                pixel_scores[id] = visibilities[id]->get_image().at<unsigned char>(u, v) * scores[id]->get_image().at<float>(u, v);
            }

            if (fusion_method == 0)
            {
                unsigned int image_with_highest_score = std::distance(pixel_scores.begin(),
                                                           std::max_element(pixel_scores.begin(),
                                                                           pixel_scores.end()));
                for (int d=0; d < depth; ++d)
                {
                    output(u, v, d) = textures[image_with_highest_score]->get_image().at<T>(u, v, d);
                }
            }
//            else if(fusion_method == MEAN_WEIGHTING)
//            {
//                // normalize pixel scores
//                double sum_of_scores = std::accumulate(pixel_scores.begin(), pixel_scores.end(), 0.0);
//                if (sum_of_scores > 0)
//                {
//                    // normalization
//                    for (int i=0; i < pixel_scores.size(); ++i)
//                    {
//                        pixel_scores[i] /= sum_of_scores;
//                    }
//                }
//                // compute pixel value
//                for (int i=0; i < pixel_scores.size(); ++i)
//                {
//                    output.at< cv::Vec<unsigned short, nb_bands> >(output_v, output_u) += pixel_scores[i] *
//                            atlases[i]->get_texture().at< cv::Vec<unsigned short, nb_bands> >(v, u);
//                }
//            }
//            else if(fusion_method == MEAN_HIGH_WEIGHTING)
//            {
//                std::vector<float> inital_pixel_scores = pixel_scores;
//                int nb_to_zero = 0;
//                for (int i=0; i < pixel_scores.size(); ++i)
//                {
//                    if (pixel_scores[i] < 0.3)
//                    {
//                        pixel_scores[i] = 0.0;
//                        ++nb_to_zero;
//                    }
//                }
//                if (nb_to_zero == pixel_scores.size())
//                {
//                    unsigned int image_with_highest_score = std::distance(inital_pixel_scores.begin(),
//                                                               std::max_element(inital_pixel_scores.begin(),
//                                                                               inital_pixel_scores.end()));
//                    output.at< cv::Vec<unsigned short, nb_bands> >(output_v, output_u) =
//                            atlases[image_with_highest_score]->get_texture().at< cv::Vec<unsigned short, nb_bands> >(v, u);
//                }
//                else
//                {
//                    // normalize pixel scores
//                    double sum_of_scores = std::accumulate(pixel_scores.begin(), pixel_scores.end(), 0.0);
//                    if (sum_of_scores > 0)
//                    {
//                        // normalization
//                        for (int i=0; i < pixel_scores.size(); ++i)
//                        {
//                            pixel_scores[i] /= sum_of_scores;
//                        }
//                    }
//                    // compute pixel value
//                    for (int i=0; i < pixel_scores.size(); ++i)
//                    {
//                        output.at< cv::Vec<unsigned short, nb_bands> >(output_v, output_u) += pixel_scores[i] *
//                                atlases[i]->get_texture().at< cv::Vec<unsigned short, nb_bands> >(v, u);
//                    }
//                }
//            }
//            else if (fusion_method == EXPONENTIAL_WEIGHTING)
//            {
//                // normalize pixel scores
//                double highest_score = *std::max_element(pixel_scores.begin(), pixel_scores.end());
//                if (highest_score > 0)
//                {
//                    double sum_of_exp = 0.0;
//                    for (int i=0; i < pixel_scores.size(); ++i)
//                    {
//                        if (pixel_scores[i] > 0)
//                            sum_of_exp += std::exp(20*pixel_scores[i]);
//                    }
//                    for (int i=0; i < pixel_scores.size(); ++i)
//                    {
//                        if (pixel_scores[i] > 0)
//                            pixel_scores[i] = std::exp(20*pixel_scores[i]) / sum_of_exp;
//                    }
//                }

//                // compute pixel value
//                for (int i=0; i < pixel_scores.size(); ++i)
//                {
//                    output.at< cv::Vec<unsigned short, nb_bands> >(output_v, output_u) += pixel_scores[i] *
//                            atlases[i]->get_texture().at< cv::Vec<unsigned short, nb_bands> >(v, u);
//                }
//            }
//            else if (fusion_method == TEST_FUSION)
//            {
//                // keep the scores that are higher than a threshold
//                std::vector<unsigned int> score_higher_than_threshold;
//                for (unsigned int i=0; i < pixel_scores.size(); ++i)
//                {
//                    if (pixel_scores[i] > 0.3)
//                    {
//                        score_higher_than_threshold.push_back(i);
//                    }
//                }
//                if (score_higher_than_threshold.size() > 0)
//                {
//                    cv::Vec<unsigned short, nb_bands> out_pixel;

//                    // for each band separately, we average the values
//                    // which are not farther from the median value
//                    // than an asbolute threshold (here 250)
//                    for (int b=0; b < nb_bands; ++b)
//                    {
//                        std::vector<unsigned short> values(score_higher_than_threshold.size());
//                        for (int j=0; j<score_higher_than_threshold.size(); ++j)
//                        {
//                            values[j] = atlases[score_higher_than_threshold[j]]->get_texture().at< cv::Vec<unsigned short, nb_bands> >(v, u)[b];
//                        }
//                        float mean = static_cast<float>(std::accumulate(values.begin(), values.end(), 0)) / values.size();
//                        out_pixel[b] = mean;
//                        // find the median value
////                                std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
////                                float median = values[values.size() / 2];
////                                // average values not too far
////                                float sum = 0.0;
////                                int sum_count = 0;
////                                for (float val : values)
////                                {
////                                    if (abs(val-median) <= 10)
////                                    {
////                                        sum += val;
////                                        ++sum_count;
////                                    }
////                                }
////                                out_pixel[b] = sum / sum_count;
//                    }
//                    output.at< cv::Vec<unsigned short, nb_bands> >(output_v, output_u) = out_pixel;

//                }
//                else
//                {
//                    // if no score is higher than the threshold we use the image with the highest
//                    unsigned int image_with_highest_score = std::distance(pixel_scores.begin(),
//                                                               std::max_element(pixel_scores.begin(),
//                                                                               pixel_scores.end()));
//                    output.at< cv::Vec<unsigned short, nb_bands> >(output_v, output_u) =
//                            atlases[image_with_highest_score]->get_texture().at< cv::Vec<unsigned short, nb_bands> >(v, u);
//                }
//            }
        }
    }

    // no dilation because the input were already dilated
    return image_container_sptr(new simple_image_container(output));
}



}
}
}


#endif // KWIVER_ARROWS_CORE_ATLAS_PROCESSING_H
