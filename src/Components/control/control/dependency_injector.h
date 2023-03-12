#ifndef _DEPENDENCY_INJECTOR_H_
#define _DEPENDENCY_INJECTOR_H_

#include "vehicle_state_provider.h"

class DependencyInjector {
 public:
  DependencyInjector() = default;

  ~DependencyInjector() = default;

  VehicleStateProvider* vehicle_state() { return &vehicle_state_; }

 private:
  VehicleStateProvider vehicle_state_;
};

#endif