/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Header for match matrix computation
 */

#ifndef KWIVER_ARROWS_CORE_MESH_UV_PARAMETERIZATION_H
#define KWIVER_ARROWS_CORE_MESH_UV_PARAMETERIZATION_H

#include <vital/vital_config.h>
#include <arrows/core/kwiver_algo_core_export.h>

#include <vital/types/mesh.h>


namespace kwiver {
namespace arrows {
namespace core {

// Typedef for a texture coordinates
typedef Eigen::Vector2d tcoord_t;
// Typedef for a vector of texture coordinates
typedef std::vector<tcoord_t> tcoords_t;

struct KWIVER_ALGO_CORE_EXPORT uv_parameterization_t
{
    tcoords_t tcoords;
    std::vector<Eigen::Vector3i> face_mapping;

    void get_bounds(double bounds[4]) const;
};

struct triangle_t
{
    Eigen::Vector2d a;
    Eigen::Vector2d b;
    Eigen::Vector2d c;
    int face_id;
    double height;
    void get_bounds(double bounds[0])
    {
        bounds[0] = std::min(a[0], std::min(b[0], c[0]));
        bounds[1] = std::max(a[0], std::max(b[0], c[0]));

        bounds[2] = std::min(a[1], std::min(b[1], c[1]));
        bounds[3] = std::max(a[1], std::max(b[1], c[1]));
    }
};

KWIVER_ALGO_CORE_EXPORT
uv_parameterization_t parameterize(kwiver::vital::mesh_sptr mesh, double resolution,
                                   unsigned int max_width, unsigned int interior_margin,
                                   unsigned int exterior_margin);

}
}
}

#endif // KWIVER_ARROWS_COREMESH_UV_PARAMETERIZATION_H
