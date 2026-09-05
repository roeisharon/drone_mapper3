// The only definitions of the staff-declared registration constructors
// (common/include/Common/MappingAlgorithmRegistration.h and MissionControlRegistration.h
// declare them without a body). Per the spec, these .cpp implementations belong to the
// Simulator project only; the executable exports the symbols (ENABLE_EXPORTS) so that
// dlopen'd plugins resolve them at load time.

#include "Registrar.h"

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

#include <utility>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator::AlgorithmRegistrar::instance().add(std::move(factory));
}

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator::MissionControlRegistrar::instance().add(std::move(factory));
}

} // namespace common
