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


#include <vw/Core/Exception.h>
#include <vw/Math/Vector.h>
#include <asp/IsisIO/IsisInterface.h>
#include <asp/IsisIO/IsisInterfaceMapFrame.h>
#include <asp/IsisIO/IsisInterfaceFrame.h>
#include <asp/IsisIO/IsisInterfaceMapLineScan.h>
#include <asp/IsisIO/IsisInterfaceLineScan.h>
#include <boost/filesystem.hpp>

#include <iomanip>
#include <ostream>

#include <Cube.h>
#include <Distance.h>
#include <Pvl.h>
#include <Camera.h>
#include <Target.h>
#include <FileName.h>
#include <CameraFactory.h>
#include <SerialNumber.h>
#include <iTime.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Thread safe implementation
  // Maintains a cache of IsisInterface contexts that threads can
  // allocate and use for their lifetime.
  // -------------------------------------------------------

  template<typename T>
  class LruIsisInterface : public IsisInterface {
  public:
    LruIsisInterface( std::string const& file ) : m_file(file) {}

    std::string type() override { return get_context().type(); }
    std::ostream& print( std::ostream& os ) override { return get_context().print(os); }

    // Standard Methods
    //------------------------------------------------------

    // These are the standard camera request; IsisInterface allows for
    // them to be customized for the type of camera so that they are
    // fast and not too full of conditionals.

    vw::Vector2 point_to_pixel ( vw::Vector3 const& point               ) const override { return get_context().point_to_pixel(point); }
    vw::Vector3 pixel_to_vector( vw::Vector2 const& pix                 ) const override { return get_context().pixel_to_vector(pix); }
    vw::Vector3 camera_center  ( vw::Vector2 const& pix = vw::Vector2() ) const override { return get_context().camera_center(pix); }
    vw::Quat    camera_pose    ( vw::Vector2 const& pix = vw::Vector2() ) const override { return get_context().camera_pose(pix); }

    // General information
    //------------------------------------------------------
    int         lines         () const override { return get_context().lines(); }
    int         samples       () const override { return get_context().samples(); }
    std::string serial_number () const override { return get_context().serial_number(); }
    double      ephemeris_time( vw::Vector2 const& pix ) const override { return get_context().ephemeris_time(pix); }
    vw::Vector3 sun_position  ( vw::Vector2 const& pix = vw::Vector2() ) const override { return get_context().sun_position(pix); }
    vw::Vector3 target_radii  () const override { return get_context().target_radii(); }
    std::string target_name   () const override { return get_context().target_name(); }

  protected:
    // Context lifecycle
    //------------------------------------------------------
    friend class IsisInterfaceClient;

    bool allocate_context() override {
      auto thread_id = vw::Thread::id();

      // Prevent recursive allocations.
      {
        vw::Mutex::ReadLock lock(m_mutex);
        if (m_contexts.find(thread_id) != m_contexts.end())
          return false;
      }

      vw::Mutex::WriteLock lock(m_mutex);
      m_contexts[thread_id].reset(new T(m_file));
      return true;
    }

    void free_context() override {
      auto thread_id = vw::Thread::id();
  
      vw::Mutex::WriteLock lock(m_mutex);
      m_contexts.erase(thread_id);
    }

    T&   get_context() const {
      auto thread_id = vw::Thread::id();

      vw::Mutex::ReadLock lock(m_mutex);
      auto it = m_contexts.find(thread_id);
      if (it == m_contexts.end())
        throw std::runtime_error("IsisInterface doesn't exist for this thread. Acquire the IsisInterface object with a IsisInterfaceClient.");

      return *it->second.get();
    }

    // Standard Variables
    //------------------------------------------------------
    std::string                                m_file;
    std::map<vw::uint64, boost::scoped_ptr<T>> m_contexts;
    mutable vw::Mutex                          m_mutex;
  };

IsisInterface* IsisInterface::open( std::string const& filename ) {
  // Opening Labels (This should be done somehow though labels)
  Isis::FileName ifilename( QString::fromStdString(filename) );
  Isis::Pvl label;
  label.read( ifilename.expanded() );

  Isis::Cube tempCube(QString::fromStdString(filename));
  Isis::Camera* camera = Isis::CameraFactory::Create( tempCube );

  IsisInterface* result;

  // Instantiate the correct class type
  switch ( camera->GetCameraType() ) {
  case 0:
    // Framing Camera
    if ( camera->HasProjection() )
      result = new LruIsisInterface<IsisInterfaceMapFrame>( filename );
    else
      result = new LruIsisInterface<IsisInterfaceFrame>( filename );
    break;
  case 2:
    // Linescan Camera
    if ( camera->HasProjection() )
      result = new LruIsisInterface<IsisInterfaceMapLineScan>( filename );
    else
      result = new LruIsisInterface<IsisInterfaceLineScan>( filename );
    break;
  default:
    vw_throw( NoImplErr() << "Don't support Isis Camera Type " << camera->GetCameraType() << " at this moment" );
  }

  return result;
}

IsisInterfaceContext::IsisInterfaceContext( std::string const& file ) {
  // Opening labels and camera
  Isis::FileName ifilename( QString::fromStdString(file) );
  m_label.reset( new Isis::Pvl() );
  m_label->read( ifilename.expanded() );

  // Opening Isis::Camera
  m_cube.reset( new Isis::Cube(QString::fromStdString(file)) );
  m_camera.reset(Isis::CameraFactory::Create( *m_cube ));
}

IsisInterfaceContext::~IsisInterfaceContext() {}

int IsisInterfaceContext::lines() const {
  return m_camera->Lines();
}

int IsisInterfaceContext::samples() const {
  return m_camera->Samples();
}

std::string IsisInterfaceContext::serial_number() const {
  Isis::Pvl copy( *m_label );
  return Isis::SerialNumber::Compose( copy, true ).toStdString();
}

double IsisInterfaceContext::ephemeris_time( vw::Vector2 const& pix ) const {
  m_camera->SetImage( pix[0]+1, pix[1]+1 );
  return m_camera->time().Et();
}

vw::Vector3 IsisInterfaceContext::sun_position( vw::Vector2 const& pix ) const {
  m_camera->SetImage( pix[0]+1, pix[1]+1 );
  Vector3 sun;
  m_camera->sunPosition( &sun[0] );
  return sun * 1000;
}

vw::Vector3 IsisInterfaceContext::target_radii() const {
  Isis::Distance radii[3];
  m_camera->radii(radii);
  return Vector3( radii[0].meters(),
                  radii[1].meters(),
                  radii[2].meters() );
}

std::string IsisInterfaceContext::target_name() const {
  return m_camera->target()->name().toStdString();
}

std::ostream& asp::isis::operator<<( std::ostream& os, IsisInterface* i ) {
  return i->print(os);
}

std::ostream& IsisInterfaceContext::print( std::ostream& os ) {
  os << "IsisInterface" << this->type()
       << "( Serial=" << this->serial_number()
       << std::setprecision(9)
       << ", f=" << m_camera->FocalLength()
       << " mm, pitch=" << m_camera->PixelPitch()
       << " mm/px," << std::setprecision(6)
       << "Center=" << this->camera_center() << " )";
    return os;
}

// Check if ISISROOT and ISISDATA was set
bool asp::isis::IsisEnv() {
  char * isisroot_ptr = getenv("ISISROOT");
  char * isisdata_ptr = getenv("ISISDATA");

  if (isisroot_ptr == NULL || isisdata_ptr == NULL ||
      std::string(isisroot_ptr) == "" ||
      std::string(isisdata_ptr) == "" ||
      !boost::filesystem::exists(std::string(isisroot_ptr) + "/IsisPreferences") )
    return false;
  return true;
}
