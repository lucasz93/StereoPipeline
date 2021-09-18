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


/// \file IsisInterfaceCache.h
///
/// Stores unused NaifContexts and the associated IsisInterfaces.
///
#ifndef __ASP_ISIS_INTERFACE_CACHE_H__
#define __ASP_ISIS_INTERFACE_CACHE_H__

// VW
#include <vw/Core/Thread.h>

// ISIS
#include <isis/NaifContext.h>

namespace asp {
namespace isis {

  class IsisInterface;
  class IsisCameraModel;

  ///
  /// Some objects need to remove references to their interfaces
  /// when the interface is detached from the current thread.
  ///
  class INaifContextListener {
  public:
    virtual void OnNaifDetach(vw::uint64 thread_id) = 0;
  };

  ///
  /// Where a single thread gets their IsisInterfaces.
  ///
  class IsisInterfaceAllocator {
    typedef boost::shared_ptr<IsisInterface> IsisInterfacePtr;
    friend class IsisInterfaceCache;

  public:
    static IsisInterfaceAllocator& get();

    IsisInterfacePtr allocate(const std::string &cube_filename, INaifContextListener *owner);
    void release(IsisInterfacePtr interface);

  private:
    void attach();
    void detach();

    // Maps a filename to a list of unallocated interfaces.
    std::map<std::string, std::vector<IsisInterfacePtr>> m_unallocated;

    // Used to verify inputs to release().
    std::vector<IsisInterfacePtr> m_allocated;

    // Only valid when the allocator is detached.
    boost::shared_ptr<Isis::NaifContext::Internal> m_naif;

    // Ensures a NAIF context is created when the allocator is created.
    Isis::NaifContextReference m_naif_reference;

    // Objects that need to know when to remove their interfaces.
    std::map<IsisInterface *, INaifContextListener *> m_listeners;
  };

  ///
  /// Where all threads can get their IsisInterfaceAllocators.
  /// 
  class IsisInterfaceCache {
  public:
    static IsisInterfaceCache& get();

    void installAllocatorForCurrentThread();
    void removeAllocatorForCurrentThread();

  private:
    vw::Mutex m_mutex;
    std::vector<IsisInterfaceAllocator *> m_allocators;
  };

}}

#endif//__ASP_ISIS_INTERFACE_CACHE_H__
