/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
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
 * \brief compute_mesh_depthmap algorithm definition
 */

#ifndef VITAL_ALGO_COMPUTE_MESH_DEPTHMAP_H
#define VITAL_ALGO_COMPUTE_MESH_DEPTHMAP_H


#include <vital/vital_config.h>
#include <vital/algo/algorithm.h>

#include <vital/types/camera.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>

namespace kwiver {
namespace vital {
namespace algo {

class VITAL_ALGO_EXPORT compute_mesh_depthmap
    : public kwiver::vital::algorithm_def<compute_mesh_depthmap>
{
public:

  /// Return the name of this algorithm
  static std::string static_type_name() { return "compute_mesh_depthmap"; }

  // compute the depthmap
  virtual std::pair<kwiver::vital::image_container_sptr, kwiver::vital::image_container_sptr>
  compute(kwiver::vital::mesh_sptr mesh, kwiver::vital::camera_sptr camera) const = 0;

protected:
  compute_mesh_depthmap();

};

typedef std::shared_ptr<compute_mesh_depthmap> compute_mesh_depthmap_sptr;

} } }
#endif // VITAL_ALGO_COMPUTE_MESH_DEPTHMAP_H
