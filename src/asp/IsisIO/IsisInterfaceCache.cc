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


#include <asp/IsisIO/IsisInterfaceCache.h>
#include <asp/IsisIO/IsisInterface.h>

#include <stdexcept>
#include <boost/make_shared.hpp>

using namespace asp;
using namespace asp::isis;

///
/// Singleton. All threads need to read+write from the same cache.
///
static IsisInterfaceCache g_cache;
IsisInterfaceCache& IsisInterfaceCache::get() { return g_cache; }

///
/// Allocators are thread specific.
///
static thread_local IsisInterfaceAllocator* tls_allocator = nullptr;
IsisInterfaceAllocator& IsisInterfaceAllocator::get() { 
  if (!tls_allocator)
    IsisInterfaceCache::get().installAllocatorForCurrentThread();
  
  return *tls_allocator;
}

///
/// Create or reuse an IsisInterface.
/// Takes the owner
///
IsisInterfaceAllocator::IsisInterfacePtr IsisInterfaceAllocator::allocate(const std::string &cube_filename, INaifContextListener *owner) {
  IsisInterfacePtr interface;
  
  // Check if we have unused interfaces for this filename.
  auto it = m_unallocated.find(cube_filename);
  if (it != m_unallocated.end() && it->second.size()) {
    interface = it->second.back();
    it->second.pop_back();
  }
  // No unused interfaces. Open a fresh one.
  else {
    interface.reset(IsisInterface::open(cube_filename));
  }

  m_allocated.push_back(interface);
  m_listeners[interface.get()] = owner;

  return interface;
}

///
///
///
void IsisInterfaceAllocator::release(IsisInterfacePtr interface) {
  const auto& filename = interface->filename();

  // Remove the interface from the allocated list.
  // Ensure we're in a good state.
  auto allocated_it = std::find_if(m_allocated.begin(), m_allocated.end(), [interface](const IsisInterfacePtr& ptr){ return ptr.get() == interface.get(); });
  if (allocated_it == m_allocated.end())
    throw std::logic_error("Releasing unknown interface. Possible cross-thread contamination?");
  m_allocated.erase(allocated_it);

  // Owner doesn't need to be notified anymore.
  m_listeners.erase(m_listeners.find(interface.get()));

  // Add the interface back to the cache.
  m_unallocated[filename].push_back(interface);
}

///
/// Hooks up the NAIF context to the current thread.
///
void IsisInterfaceAllocator::attach() {
  Isis::NaifContext::attach(m_naif);
}

///
/// Detaches the NAIF context from the current thread.
///
void IsisInterfaceAllocator::detach() {
  const auto thread_id = vw::Thread::id();
  auto listeners = m_listeners;

  for (auto &l : listeners) {
    l.second->OnNaifDetach(thread_id);
  }

  m_naif = Isis::NaifContext::detach();
}

///
/// Assigns a thread-local allocator.
/// Installs a listener that deallocates the allocator on thread shutdown.
///
void IsisInterfaceCache::installAllocatorForCurrentThread() {
  class DetachAllocatorListener : public vw::ThreadEventListener {
  public:
    void finish(uint64_t id) override { IsisInterfaceCache::get().removeAllocatorForCurrentThread(); }
  };

  if (tls_allocator)
    throw std::logic_error("Thread already has an IsisInterfaceAllocator");

  // Main thread doesn't have a self pointer.
  // Guess we'll just let its allocator live forever.
  if (vw::Thread::self())
    vw::Thread::self()->add_listener(boost::make_shared<DetachAllocatorListener>());

  // Get the allocator for this thread.
  {
    vw::Mutex::WriteLock lock(m_mutex);

    // Recycle existing allocators.
    if (m_allocators.size()) {
      tls_allocator = m_allocators.back();
      m_allocators.pop_back();

      // Make the allocator ready for this thread.
      tls_allocator->attach();
    }
    // No spare allocators. Make new one.
    else {
      tls_allocator = new IsisInterfaceAllocator();
      // No need to attach. Done in the allocator constructor.
    }
  }
}

///
/// Assigns a thread-local allocator.
/// Installs a listener that deallocates the allocator on thread shutdown.
///
void IsisInterfaceCache::removeAllocatorForCurrentThread() {
  if (!tls_allocator)
    throw std::logic_error("Thread doesn't have an IsisInterfaceAllocator");

  // Unhook the allocate from the thread.
  tls_allocator->detach();

  // Place the allocator back into the cache.
  {
    vw::Mutex::WriteLock lock(m_mutex);
    m_allocators.push_back(tls_allocator);
  }

  tls_allocator = nullptr;
}
