#pragma once

#include <stdexcept>

namespace mission_control_322526864_323020347 {

// The ONE exception family allowed to escape runMission (everything else is converted to an
// error entry). runMission catches the BASE class on the way out only to honor the save-always
// invariant and then rethrows; the simulation run catches it and scores the run -1.
//
// Members of the family:
//   - FatalCollisionError — common-issues Mandatory row 5: the movement driver (which holds the
//     real map) threw AND the commanded destination was already KNOWN Occupied in the output map.
//     A valid algorithm never commands that move (row 1), so the mission is unsalvageable.
//   - FatalMissionError thrown directly — the Phase-8a optional rows' escalation paths, where
//     "gracefully try again; throw after N tries" ran out of tries: N consecutive invalid
//     commands (row 3), N consecutive NOOPs (row 4), N consecutive empty lidar scans (row 6),
//     N consecutive movement-driver failures (row 7), or a GPS reading that is out of bounds
//     while the internal estimate is too (row 11).
// Distinct from the row-9 path (an Error STATUS returned from step()), where the mission logs
// the error and continues.
struct FatalMissionError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct FatalCollisionError : FatalMissionError {
    using FatalMissionError::FatalMissionError;
};

} // namespace mission_control_322526864_323020347
