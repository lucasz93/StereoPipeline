// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file IsisInterface.h
///
/// Generic Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_H__
#define __ASP_ISIS_INTERFACE_H__

// VW & ASP
#include <string>
#include <Cube.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <asp/SpiceIO/SpiceUtilities.h>

// ISIS
#include <isis/NaifContext.h>
namespace Isis {
  class Pvl;
  class Camera;
}

namespace asp {
namespace isis {

  /// The IsisInterface abstract base class
  // -------------------------------------------------------

  class IsisInterface {
  public:
    IsisInterface( boost::shared_ptr<Isis::Pvl> &label, boost::shared_ptr<Isis::Cube> &cube, boost::shared_ptr<Isis::Camera> &camera, const Isis::NaifSnapshot& snapshot )
      : m_label(label)
      , m_cube(cube)
      , m_camera(camera)
      , m_working_snapshot(boost::make_shared<Isis::NaifSnapshot>(snapshot))
    {
    }

    virtual ~IsisInterface(); // Can't be declared here since we have
                              // incomplete types from Isis.

    virtual std::string type() = 0;
    boost::shared_ptr<Isis::NaifSnapshot> snapshot() { return m_working_snapshot; }
    
    /// Construct an IsisInterface-derived class of the correct type for the given file.
    static IsisInterface* open( std::string const& filename, const Isis::NaifSnapshot& snapshot );

    // Standard Methods
    //------------------------------------------------------

    // These are the standard camera request; IsisInterface allows for
    // them to be customized for the type of camera so that they are
    // fast and not too full of conditionals.

    virtual vw::Vector2 point_to_pixel ( vw::Vector3 const& point               ) const = 0;
    virtual vw::Vector3 pixel_to_vector( vw::Vector2 const& pix                 ) const = 0;
    virtual vw::Vector3 camera_center  ( vw::Vector2 const& pix = vw::Vector2() ) const = 0;
    virtual vw::Quat    camera_pose    ( vw::Vector2 const& pix = vw::Vector2() ) const = 0;

    // General information
    //------------------------------------------------------
    int         lines         () const;
    int         samples       () const;
    std::string serial_number () const;
    double      ephemeris_time( vw::Vector2 const& pix ) const;
    vw::Vector3 sun_position  ( vw::Vector2 const& pix = vw::Vector2() ) const;
    vw::Vector3 target_radii  () const;
    std::string target_name   () const;

  protected:
    // Standard Variables
    //------------------------------------------------------
    boost::shared_ptr<Isis::Pvl   > m_label;
    boost::shared_ptr<Isis::Camera> m_camera;
    boost::shared_ptr<Isis::Cube  > m_cube;

    // Current CSPICE state.
    boost::shared_ptr<Isis::NaifSnapshot> m_working_snapshot;

    friend std::ostream& operator<<( std::ostream&, IsisInterface* );
  };

  // IOstream interface
  // -------------------------------------------------------
  std::ostream& operator<<( std::ostream& os, IsisInterface* i );

  bool IsisEnv();
}}

#endif//__ASP_ISIS_INTERFACE_H__
