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
 * \brief Header for mesh uv parameterization
 */

#ifndef KWIVER_ARROWS_CORE_MESH_UV_PARAMETERIZATION_H
#define KWIVER_ARROWS_CORE_MESH_UV_PARAMETERIZATION_H

#include <vital/vital_config.h>
#include <arrows/core/kwiver_algo_core_export.h>

#include <vital/types/mesh.h>


namespace kwiver {
namespace arrows {
namespace core {

/**
 * @brief parameterize
 * @param mesh
 * @param resolution [in] resolution used (mesh unit / pixel)
 * @param max_width [in] maximal parameterization width (in pixels)
 * @param interior_margin [in] horizontal or vertical margin between triangles (in pixels)
 * @param exterior_margin [in] horizontal or vertical margin at the borders (in pixels)
 * @return widht and height of the parameterization space
 */
KWIVER_ALGO_CORE_EXPORT
std::pair<unsigned int, unsigned int>
parameterize(kwiver::vital::mesh_sptr mesh, double resolution,
             int max_width, int interior_margin, int exterior_margin);

}
}
}

#endif // KWIVER_ARROWS_COREMESH_UV_PARAMETERIZATION_H
