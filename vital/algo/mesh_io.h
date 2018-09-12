/*ckwg +29
 * Copyright 2013-2015 by Kitware, Inc.
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
 * \brief Interface for mesh_io \link kwiver::vital::algo::algorithm_def algorithm
 *        definition \endlink.
 */

#ifndef VITAL_ALGO_MESH_IO_H
#define VITAL_ALGO_MESH_IO_H

#include <vital/vital_config.h>
#include <string>

#include <vital/algo/algorithm.h>
#include <vital/types/mesh.h>
#include <arrows/core/mesh_uv_parameterization.h>

namespace kwiver {
namespace vital {
namespace algo {

/// An abstract base class for reading and writing images
/**
 * This class represents an abstract interface for reading and writing
 * images.
 */
class VITAL_ALGO_EXPORT mesh_io
  : public kwiver::vital::algorithm_def<mesh_io>
{
public:
  virtual ~mesh_io() = default;

  /// Return the name of this algorithm
  static std::string static_type_name() { return "mesh_io"; }

  /// Load mesh from the file
  /**
   * \throws kwiver::vital::path_not_exists Thrown when the given path does not exist.
   *
   * \throws kwiver::vital::path_not_a_file Thrown when the given path does
   *    not point to a file (i.e. it points to a directory).
   *
   * \param filename the path to the file to load
   * \returns a smart pointer to the loaded mesh
   */
  kwiver::vital::mesh_sptr load(std::string const& filename) const;


  /// Save mesh to file
  /**
   * Image file format is based on file extension.
   *
   * \throws kwiver::vital::path_not_exists Thrown when the expected
   *    containing directory of the given path does not exist.
   *
   * \throws kwiver::vital::path_not_a_directory Thrown when the expected
   *    containing directory of the given path is not actually a
   *    directory.
   *
   * \param filename the path to the file to save
   * \param data the image container refering to the mesh to write
   * \param a uv parameterization (pass nullptr to ignore texture)
   * \param the texture size
   */
  void save(const std::string& filename, kwiver::vital::mesh_sptr data,
            unsigned tex_width=1, unsigned int tex_height=1, bool flip_v_axis=false) const;

protected:
  mesh_io();

private:
  /// Implementation specific load functionality
  /**
   * Concrete implementations of mesh_io class must provide an
   * implementation for this method.
   *
   * \param filename the path to the file to load
   * \returns a smart pointer to the loaded mesh
   */
  virtual kwiver::vital::mesh_sptr load_(std::string const& filename) const = 0;

  /// Implementation specific save functionality.
  /**
   * Concrete implementations of mesh_io class must provide an
   * implementation for this method.
   *
   * \param filename the path to the file to save
   * \param data the image container refering to the mesh to write
   * \param a uv parameterization (pass nullptr to ignore texture)
   * \param the texture size
   */
  virtual void save_(const std::string& filename, kwiver::vital::mesh_sptr data,
                     unsigned tex_width=1, unsigned int tex_height=1, bool flip_v_axis=false) const = 0;
};

/// Shared pointer type for generic mesh_io definition type.
typedef std::shared_ptr<mesh_io> mesh_io_sptr;

} } }
#endif // VITAL_ALGO_MESH_IO_H
