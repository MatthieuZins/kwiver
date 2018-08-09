#include "atlas_processing.h"
#include <vital/types/matrix.h>
#include <vital/types/camera_rpc.h>
#include <vital/types/geodesy.h>
#include <Eigen/Dense>

using namespace kwiver::vital;

namespace kwiver {
namespace arrows {
namespace core {


bool is_point_inside_triangle(const vector_2d& p,
                              const vector_2d& a,
                              const vector_2d& b,
                              const vector_2d& c)
{
    vector_2d AB = b - a;
    vector_2d AC = c - a;
    vector_2d AP = p - a;

    double inv_total_area = 1.0 / (AB[0]*AC[1]-AB[1]*AC[0]);
    double area_1 = inv_total_area * (AB[0]*AP[1]-AB[1]*AP[0]);
    double area_2 = inv_total_area * (AP[0]*AC[1]-AP[1]*AC[0]);

    return area_1>=0 && area_2>=0 && (area_1+area_2)<=1;
}


vector_3d barycentric_coordinates(const vector_2d& p,
                                  const vector_2d& a,
                                  const vector_2d& b,
                                  const vector_2d& c)
{
    matrix_3x3d abc;
    abc << a, b, c, 1, 1, 1;
    double det_inv = 1.0 / abc.determinant();

    vector_3d res;
    res(0) = ((b(1)-c(1)) * (p(0)-c(0)) - (b(0) - c(0)) * (p(1) - c(1))) * det_inv;
    res(1) = ((c(1)-a(1)) * (p(0)-c(0)) - (c(0) - a(0)) * (p(1) - c(1))) * det_inv;
    res(2) = 1.0 - res(0) - res(1);
    return res;
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

    int height = static_cast<int>(atlas.height());
    int width = static_cast<int>(atlas.width());
    int depth = static_cast<int>(atlas.depth());

    for (int n=0; n<nb_iter; ++n)
    {
        image_of<bool> tmp;
        tmp.copy_from(mask);

        // vertically
        for (int r=0; r < height; ++r)
        {
            for (int c=0; c < width; ++c)
            {
                if (mask(c, r) == 0)
                {
                    std::vector<unsigned int> values(depth, 0);
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
                            atlas.at<T>(c, r) = static_cast<float>(values[d]) / nb;
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
                if (mask(c, r) == 0)
                {
                    std::vector<unsigned int> values(depth, 0);
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
                            atlas.at<T>(c, r) = static_cast<float>(values[d]) / nb;
                        }
                        tmp(c, r) = true;
                    }
                }
            }
        }

        mask.copy_from(tmp);
    }
}


vital::image_container_sptr generate_triangles_map(const kwiver::arrows::core::uv_parameterization_t &param,
                                                   int exterior_margin)
{

    // can handle separate triangles or adjacent triangles (with weak and strong assignment)

    double bounds[4];
    param.get_bounds(bounds);
    double width = std::ceil(bounds[1]+1 + exterior_margin);     //+3 is a margin (at the bottom and right borders)
    double height = std::ceil(bounds[3]+1 + exterior_margin);

    // status_grid contains the status of each pixel:
    // 0: do not intersect any triangle
    // 1: the pixel center is inside a triangle (the pixel is be strongly assigned to this triangle, it will not changed)
    // 2: the pixel intersects partly with a triangle (the pixel is weakly assigned ti this triangle, it may change depending on the next triangles)
    // id_grid contains the current the triangle id assigned to each pixel

    image_of<unsigned char> status_grid(width, height);
    image_of<int> id_grid(width, height);
    for (int i=0; i < height; ++i)
    {
        for (int j=0; j < width; ++j)
        {
            status_grid(j, i) = 0;
            id_grid(j, i) = -1;
        }
    }

    int number_of_faces = param.face_mapping.size();
    for (int f=0; f < number_of_faces; ++f)
    {
        // get the face uv coordinates
        const vector_2d& a_uv = param.tcoords[param.face_mapping[f][0]];
        const vector_2d& b_uv = param.tcoords[param.face_mapping[f][1]];
        const vector_2d& c_uv = param.tcoords[param.face_mapping[f][2]];

        // get the 2d bounding box
        int u_min = static_cast<int>(std::floor(std::min(a_uv[0], std::min(b_uv[0], c_uv[0]))));
        int u_max = static_cast<int>(std::ceil(std::max(a_uv[0], std::max(b_uv[0], c_uv[0]))));
        int v_min = static_cast<int>(std::floor(std::min(a_uv[1], std::min(b_uv[1], c_uv[1]))));
        int v_max = static_cast<int>(std::ceil(std::max(a_uv[1], std::max(b_uv[1], c_uv[1]))));

        // map pixels to triangles
        for (int v=v_min; v<=v_max; ++v)
        {
            for (int u=u_min; u<=u_max; ++u)
            {
                vector_2d center(u, v);
                if (is_point_inside_triangle(center, a_uv, b_uv, c_uv))
                {
                    id_grid(u, v) = f;
                    status_grid(u, v) = 1;   // strong assignment
                }
                else
                {
                    bool partial_intersection = false;
                    // check partial intersections (done by checking 25 points
                    // regularly placed on the pixel)
                    for (float dy=-0.5; dy <= 0.5; dy+=0.25)
                    {
                        for (float dx=-0.5; dx <= 0.5; dx+=0.25)
                        {
                            if (is_point_inside_triangle(vector_2d(center[0]+dx, center[1]+dy), a_uv, b_uv, c_uv))
                            {
                                partial_intersection = true;
                                break;
                            }
                        }
                    }
                    if (partial_intersection && status_grid(u, v) != 1)
                    {   // if partial intersection and this pixel was not strongly assigned before
                        id_grid(u, v) = f;
                        status_grid(u, v) = 2;
                    }
                }
            }
        }
    }
    return image_container_sptr(new simple_image_container(id_grid));
}

image_container_sptr rasterize(mesh_sptr mesh, const uv_parameterization_t& param,
                               image_container_sptr triangles_id_map, image_container_sptr image,
                               camera_sptr camera, image_container_sptr depthmap)
{
    unsigned int nb_vertices = mesh->num_verts();
    unsigned int nb_faces = mesh->num_faces();

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

    unsigned int width = triangles_id_map->width();
    unsigned int height = triangles_id_map->height();
    unsigned int depth = image->depth();
    image_of<unsigned short> texture(width, height, depth);
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
            double pixel_score = 0.0;
//            pixel_score = rating[face_id];

            // Occlusions
            // compute the real depth of a point
            double point_depth = points_depth[A_id] * bary_coords[0] +
                                 points_depth[B_id] * bary_coords[1] +
                                 points_depth[C_id] * bary_coords[2];

            bool is_occluded = true;
            if (depthmap.get())
            {
                // here we check the 4 nearest pixels of the point on the depthmap
                Eigen::Vector2i p_floor(floor(p_img[0]), floor(p_img[1]));
                double d0 = depthmap->get_image().at<double>(p_floor[0], p_floor[1]);
                double d1 = depthmap->get_image().at<double>(p_floor[0]+1, p_floor[1]);
                double d2 = depthmap->get_image().at<double>(p_floor[0], p_floor[1]+1);
                double d3 = depthmap->get_image().at<double>(p_floor[0]+1, p_floor[1]+1);
                // if the real value is close to one of the for values from the depthmap,
                // we consider the point as non occluded
                if (fabs(point_depth - d0) < 0.5)
                {
                    is_occluded = false;
                }
                else if (fabs(point_depth - d1) < 0.5)
                {
                    is_occluded = false;
                }
                else if (fabs(point_depth - d2) < 0.5)
                {
                    is_occluded = false;
                }
                else if (fabs(point_depth - d3) < 0.5)
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
                    if (min_depth == std::numeric_limits<double>::max() ||
                            point_depth >= min_depth && point_depth <= max_depth)
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
                    texture(u, v, b) = image->get_image().at<unsigned short>(p_img[0], p_img[1], b);
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
    dilate_atlas<unsigned short>(texture, mask, 2);
    dilate_atlas<unsigned char>(visibility, mask, 2);
    dilate_atlas<unsigned char>(shadow, mask, 2);
    dilate_atlas<float>(scores, mask, 2);

//    out_texture_atlas->set_texture(texture);
//    out_texture_atlas->set_visibility(visibility);
//    out_texture_atlas->set_shadow(shadow);
//    out_texture_atlas->set_score(score);
//    out_texture_atlas->set_normals(normals);

    return image_container_sptr(new simple_image_container(texture));
}


}
}
}
