// Test-binary-only definitions of the staff registration constructors. In the product these are
// defined by the Simulator (which owns the registrar); a test binary that compiles a plugin's
// sources directly still triggers the plugin's REGISTER_* global, so the symbols must exist —
// as no-ops, because tests construct the classes directly and need no registry.

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory) {}

MissionControlRegistration::MissionControlRegistration(MissionControlFactory) {}

} // namespace common
