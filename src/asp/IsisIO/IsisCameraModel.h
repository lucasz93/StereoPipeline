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


/// \file IsisCameraModel.h
///
/// This file contains the ISIS camera model.
///
#ifndef __VW_CAMERAMODEL_ISIS_H__
#define __VW_CAMERAMODEL_ISIS_H__

// VW
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Core/Thread.h>

// ASP
#include <asp/Core/Common.h>
#include <asp/IsisIO/IsisInterfaceCache.h>
#include <asp/IsisIO/IsisInterface.h>

// ISIS
#include <isis/NaifContext.h>

namespace vw {
namespace camera {

  // This is largely just a shortened reimplementation of ISIS's
  // Camera.cpp.
  class IsisCameraModel : public CameraModel, public asp::isis::INaifContextListener {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    IsisCameraModel(const std::string &cube_filename)
      : m_cube_filename(cube_filename)
    {
    }

    virtual std::string type() const { return "Isis"; }

    virtual void OnNaifDetach(vw::uint64 thread_id) override {
      vw::FastSharedMutex::WriteLock lock(m_mutex);

      auto it = m_interfaces.find(thread_id);
      if (it == m_interfaces.end())
        throw std::runtime_error("Attempting to interface for unknown thread!");

      asp::isis::IsisInterfaceAllocator::get().release(it->second);
      m_interfaces.erase(it);
    }

    //------------------------------------------------------------------
    // Methods
    //------------------------------------------------------------------

    //  Computes the image of the point 'point' in 3D space on the
    //  image plane.  Returns a pixel location (col, row) where the
    //  point appears in the image.
    virtual Vector2 point_to_pixel(Vector3 const& point) const {
      return get_interface()->point_to_pixel( point ); }

    // Returns a (normalized) pointing vector from the camera center
    //  through the position of the pixel 'pix' on the image plane.
    virtual Vector3 pixel_to_vector (Vector2 const& pix) const {
      return get_interface()->pixel_to_vector( pix ); }


    // Returns the position of the focal point of the camera
    virtual Vector3 camera_center(Vector2 const& pix = Vector2() ) const {
      return get_interface()->camera_center( pix ); }

    // Pose is a rotation which moves a vector in camera coordinates
    // into world coordinates.
    virtual Quat camera_pose(Vector2 const& pix = Vector2() ) const {
      return get_interface()->camera_pose( pix ); }

    // Returns the number of lines is the ISIS cube
    int lines() const {
      return get_interface()->lines(); }

    // Returns the number of samples in the ISIS cube
    int samples() const{
      return get_interface()->samples(); }

    // Returns the serial number of the ISIS cube
    std::string serial_number() const {
      return get_interface()->serial_number(); }

    // Returns the ephemeris time for a pixel
    double ephemeris_time( Vector2 const& pix = Vector2() ) const {
      return get_interface()->ephemeris_time( pix );
    }

    // Sun position in the target frame's inertial frame
    Vector3 sun_position( Vector2 const& pix = Vector2() ) const {
      return get_interface()->sun_position( pix );
    }

    // The three main radii that make up the spheroid. Z is out the polar region
    Vector3 target_radii() const {
      return get_interface()->target_radii();
    }

    // The spheroid name
    std::string target_name() const {
      return get_interface()->target_name();
    }

    // The datum
    vw::cartography::Datum get_datum(bool use_sphere_for_datum) const {
      return m_interface->get_datum(use_sphere_for_datum);
    }
    
  protected:
    friend std::ostream& operator<<( std::ostream&, IsisCameraModel const& );

    //
    // Honestly, this whole method is pretty dodgy. Not happy with its use of
    // mutable members, but removing const the const qualifier would mean
    // modifying every camera in ASP. Not worth it.
    //
    asp::isis::IsisInterface* get_interface() const {
      // Check if an interface for this thread already exists.
      {
        vw::FastSharedMutex::ReadLock lock(m_mutex);

        auto it = m_interfaces.find(vw::Thread::id());
        if (it != m_interfaces.end())
          return it->second.get();
      }

      // Interface doesn't exist. Create one.
      // Very slow - do this outside of any lock.
      //
      // This const_cast is safe - we're only adding the pointer into a list.
      // The non-const pointer is then calls non-const methods at a later time.
      auto interface = asp::isis::IsisInterfaceAllocator::get().allocate(m_cube_filename, const_cast<IsisCameraModel *>(this));

      // Add the interface to the map.
      {
        vw::FastSharedMutex::WriteLock lock(m_mutex);
        m_interfaces[vw::Thread::id()] = interface;
      }

      return interface.get();
    }

  private:
    std::string m_cube_filename;

    mutable vw::FastSharedMutex m_mutex;

    mutable std::map<vw::uint64, boost::shared_ptr<asp::isis::IsisInterface>> m_interfaces;
  };

  // IOstream interface
  // ---------------------------------------------
  inline std::ostream& operator<<( std::ostream& os,
                                   IsisCameraModel const& i ) {
    os << "IsisCameraModel" << i.lines() << "x" << i.samples() << "( "
       << i.get_interface() << " )";
    return os;
  }

}}

#endif  //__VW_CAMERA_ISIS_H__
