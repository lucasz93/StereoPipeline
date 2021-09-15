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
#include <asp/IsisIO/IsisInterfaceSAR.h>
#include <asp/IsisIO/IsisCameraModel.h>
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

class InterfaceDeregisterListener : public vw::ThreadEventListener {
private:
  const vw::camera::IsisCameraModel* m_camera;

public:
  InterfaceDeregisterListener(const vw::camera::IsisCameraModel *cam) : m_camera(cam) {}

  void finish(uint64_t id) override {
    m_camera->release_interface(id);
  }
};

asp::isis::IsisInterface* vw::camera::IsisCameraModel::get_interface() const {
  const auto thread_id = vw::Thread::id();

  // Check if this thread has an active interface.
  {
    vw::FastSharedMutex::ReadLock lock(m_lock);
    auto it = m_active_interfaces.find(thread_id);
    if (it != m_active_interfaces.end()) {
      return it->second.get();
    }
  }

  // Main thread doesn't have a self pointer.
  // Guess we'll just let its interface live forever.
  if (vw::Thread::self())
    vw::Thread::self()->add_listener(boost::make_shared<InterfaceDeregisterListener>(this));

  // No active interfaces - look in the LRU.
  {
    boost::shared_ptr<IsisInterface> interface;

    // Try and acquire an interface.
    {
      vw::FastSharedMutex::WriteLock lock(m_lock);

      if (m_lru_interfaces.size())
      {
        interface = m_lru_interfaces.front();
        auto result = m_active_interfaces.insert(std::make_pair(thread_id, interface));
        m_lru_interfaces.pop_back();
      }
    }

    // Setup and return the interface.
    if (interface.get())
    {
      // NAIF context construction is expensive. Do it outside the lock.
      interface->acquireNaifContext();
      return interface.get();
    }
  }

  // No spare interfaces - create one.
  {
    // Load the interface outside the write lock. Allows other threads to continue using this method without blocking.
    auto interface = boost::shared_ptr<IsisInterface>(IsisInterface::open(m_cube_filename));

    // NAIF context construction is expensive. Do it outside the lock.
    interface->acquireNaifContext();

    vw::FastSharedMutex::WriteLock lock(m_lock);

    auto result = m_active_interfaces.insert(std::make_pair(thread_id, interface));
    return result.first->second.get();
  }
}

void vw::camera::IsisCameraModel::release_interface(uint64_t id) const {
  boost::shared_ptr<IsisInterface> interface;

  // Remove the interface from the active list.
  {
    vw::FastSharedMutex::WriteLock lock(m_lock);

    auto it = m_active_interfaces.find(id);
    if (it == m_active_interfaces.end())
      throw std::runtime_error("Releasing an interface not in the active list?!");

    interface = it->second;
    m_active_interfaces.erase(it);
  }

  // NAIF context destruction is slow. Do it outside the lock.
  interface->releaseNaifContext();

  // Add the interface to the cache.
  {
    vw::FastSharedMutex::WriteLock lock(m_lock);
    
    m_lru_interfaces.push_back(interface);
  }
}

void IsisInterface::acquireNaifContext() {
  if (!m_naif) {
    m_naif_lifecycle.reset(new Isis::NaifContextLifecycle);

    // IsisInterface instances are restricted to use in the thread they were created.
    // That allows us to cache the thread local NAIF context pointer.
    m_naif = Isis::NaifContext::acquire();
  }
}

void IsisInterface::releaseNaifContext() {
  if (!m_naif) {
    m_naif = nullptr;
    m_naif_lifecycle.reset(nullptr);
  }
}

IsisInterface::IsisInterface( boost::shared_ptr<Isis::Pvl> &label, boost::shared_ptr<Isis::Cube> &cube, boost::shared_ptr<Isis::Camera> &camera )
  : m_label(label)
  , m_cube(cube)
  , m_camera(camera)
{
  acquireNaifContext();

  // Set the datum
  // TODO(oalexan1): This is fragile. Need to find the right internal ISIS
  // function to use to convert ECEF to lon-lat-height and vice-versa.
  bool use_sphere_for_datum = false;
  m_datum = this->get_datum(use_sphere_for_datum);
}

IsisInterface::~IsisInterface() {}

IsisInterface* IsisInterface::open(std::string const& filename, const Isis::NaifSnapshot& snapshot) {
  // Opening Labels (This should be done somehow though labels)
  Isis::FileName ifilename( QString::fromStdString(filename) );
  boost::shared_ptr<Isis::Pvl> label(new Isis::Pvl(ifilename.expanded()));

  boost::shared_ptr<Isis::Cube> tempCube(new Isis::Cube(QString::fromStdString(filename)));
  boost::shared_ptr<Isis::Camera> camera(Isis::CameraFactory::Create( *tempCube ));

  IsisInterface* result;

  // Instantiate the correct class type
  switch (camera->GetCameraType()) {
  case 0:
    // Framing camera
    if (camera->HasProjection())
      result = new IsisInterfaceMapFrame(label, tempCube, camera, snapshot);
    else
      result = new IsisInterfaceFrame(label, tempCube, camera, snapshot);
    break;
  case 2:
    // Linescan camera
    if (camera->HasProjection())
      result = new IsisInterfaceMapLineScan(label, tempCube, camera, snapshot);
    else
      result = new IsisInterfaceLineScan(label, tempCube, camera, snapshot);
    break;
  case 3:
    // SAR camera (such as MiniRF)
    // The same interface handles both projected and unprojected images,
    // since the ISIS functions take care of the details.
    // TODO(oalexan1): that cam2map-ed images are handled correctly.
    result = new IsisInterfaceSAR(filename);
    break;
  default:
    // LRO WAC comes here
    vw_throw(NoImplErr() << "Don't support Isis camera type "
             << camera->GetCameraType() << " at this moment. "
             << "Consider using CSM cameras with these images.");
  }

  return result;
}

int IsisInterface::lines() const {
  return m_camera->Lines();
}

int IsisInterface::samples() const {
  return m_camera->Samples();
}

std::string IsisInterface::serial_number() const {
  Isis::Pvl copy(*m_label);
  return Isis::SerialNumber::Compose(copy, true).toStdString();
}

double IsisInterface::ephemeris_time(vw::Vector2 const& pix) const {
  m_camera->SetImage(pix[0]+1, pix[1]+1, m_naif);
  return m_camera->time().Et();
}

vw::Vector3 IsisInterface::sun_position(vw::Vector2 const& pix) const {
  m_camera->SetImage(pix[0]+1, pix[1]+1, m_naif);
  Vector3 sun;
  m_camera->sunPosition(&sun[0]);
  return sun * 1000;
}

vw::Vector3 IsisInterface::target_radii() const {
  Isis::Distance radii[3];
  m_camera->radii(radii);
  return Vector3(radii[0].meters(),
                  radii[1].meters(),
                  radii[2].meters());
}

std::string IsisInterface::target_name() const {
  return m_camera->target()->name().toStdString();
}

// Manufacture a datum
vw::cartography::Datum IsisInterface::get_datum(bool use_sphere_for_datum) const {
      
  vw::Vector3 radii = this->target_radii();
  double radius1 = (radii[0] + radii[1]) / 2; // average the x and y axes (semi-major) 
  double radius2 = radius1;
  if (!use_sphere_for_datum) {
    radius2 = radii[2]; // the z radius (semi-minor axis)
  }
  
  vw::cartography::Datum datum("D_" + this->target_name(), this->target_name(),
                               "Reference Meridian", radius1, radius2, 0);
  return datum;
}

std::ostream& asp::isis::operator<<(std::ostream& os, IsisInterface* i) {
  os << "IsisInterface" << i->type()
       << "(Serial=" << i->serial_number()
       << std::setprecision(9)
       << ", f=" << i->m_camera->FocalLength()
       << " mm, pitch=" << i->m_camera->PixelPitch()
       << " mm/px," << std::setprecision(6)
       << "Center=" << i->camera_center() << ")";
    return os;
}

// Check if ISISROOT and ISISDATA was set
bool asp::isis::IsisEnv() {
  char * isisroot_ptr = getenv("ISISROOT");
  char * isisdata_ptr = getenv("ISISDATA");

  if (isisroot_ptr == NULL || isisdata_ptr == NULL ||
      std::string(isisroot_ptr) == "" ||
      std::string(isisdata_ptr) == "" ||
      !boost::filesystem::exists(std::string(isisroot_ptr) + "/IsisPreferences"))
    return false;
  return true;
}
