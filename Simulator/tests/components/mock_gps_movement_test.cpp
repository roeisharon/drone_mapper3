#include <gtest/gtest.h>

#include <Common/Units.h>

#include "map/Map3DImpl.h"
#include "mocks/MockGPS.h"
#include "mocks/MockMovement.h"

#include <cmath>
#include <stdexcept>
#include <string>

// These tests verify the real MockGPS + MockMovement behaviour. Ported from ex2, where they shared
// the SimulationRun fixture; the suite keeps the ex2 name (so --gtest_filter=SimulationRun.* still
// covers the GPS/movement mocks) with a local fixture providing the same `gps`/`movement` members.
//
// ONE semantic change vs ex2 (common-issues Mandatory row 5): a blocked advance()/elevate() no
// longer returns MovementResult{false, "DRONE_HITS_OBSTACLE"} — it THROWS std::runtime_error whose
// message starts with "DRONE_HITS_OBSTACLE", and the drone stays put. rotate() never throws, and
// the plain 1-arg constructor still disables collision checking entirely.
using namespace simulator;
using namespace common;

namespace {

// Fixture for the SimulationRun suite: a real concrete MockGPS at the world origin, heading
// 0 deg / 0 deg, 1 cm GPS resolution, and a MockMovement built with the plain (no hidden map)
// constructor, so moves always succeed. GTest reconstructs the fixture before every test, so each
// starts from this clean state. Kept in the anonymous namespace so a same-named fixture in
// another translation unit cannot collide at link time.
class SimulationRun : public ::testing::Test {
protected:
    MockGPS gps{{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
                {0.0 * deg, 0.0 * deg},
                1.0 * cm};

    // MockMovement mutates the gps above via advance/elevate/rotate. Declared after gps so member
    // initialisation order matches (movement holds a reference to gps).
    MockMovement movement{gps};
};

} // namespace

// advance ----------------------------------------------------------------------

// Heading 0°: cos(0)=1, sin(0)=0 — the whole distance goes to X; Y and Z stay.
TEST_F(SimulationRun, AdvanceAt0DegreesUpdatesXOnly) {
    gps.setHeading({0.0 * deg, 0.0 * deg});
    const auto result = movement.advance(10.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 0.0);
}

// Heading 90°: sin(90°)=1 — the whole distance goes to Y; X stays.
TEST_F(SimulationRun, AdvanceAt90DegreesUpdatesYOnly) {
    gps.setHeading({90.0 * deg, 0.0 * deg});
    movement.advance(10.0 * cm);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 0.0,  1e-9);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 10.0, 1e-9);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 0.0);
}

// Heading 180°: cos(180°)=-1 — advancing moves in the -X direction.
TEST_F(SimulationRun, AdvanceAt180DegreesDecrementsX) {
    gps.setHeading({180.0 * deg, 0.0 * deg});
    movement.advance(10.0 * cm);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), -10.0, 1e-9);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm),   0.0, 1e-9);
}

// Heading 270°: sin(270°)=-1 — advancing moves in the -Y direction.
TEST_F(SimulationRun, AdvanceAt270DegreesDecrementsY) {
    gps.setHeading({270.0 * deg, 0.0 * deg});
    movement.advance(10.0 * cm);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm),  0.0,  1e-9);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), -10.0, 1e-9);
}

// Heading 45°: equal X and Y components, each distance/√2.
TEST_F(SimulationRun, AdvanceDiagonalAt45Degrees) {
    gps.setHeading({45.0 * deg, 0.0 * deg});
    movement.advance(10.0 * cm);
    const double expected = 10.0 / std::sqrt(2.0);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), expected, 1e-9);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), expected, 1e-9);
}

// advance() is a horizontal move; altitude must be untouched.
TEST_F(SimulationRun, AdvanceDoesNotChangeZ) {
    gps.setHeading({45.0 * deg, 0.0 * deg});
    gps.setPosition({0.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]});
    movement.advance(10.0 * cm);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 50.0);
}

// advance() always succeeds in the mock (no hidden map injected), with an empty message.
TEST_F(SimulationRun, AdvanceReturnsSuccess) {
    const auto result = movement.advance(5.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.message.empty());
}

// elevate ----------------------------------------------------------------------

// Positive distance increases Z; X and Y stay.
TEST_F(SimulationRun, ElevatePositiveDistanceIncrementsZ) {
    movement.elevate(20.0 * cm);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 0.0);
}

// Negative distance decreases Z — elevate() supports descent via the sign.
TEST_F(SimulationRun, ElevateNegativeDistanceDecrementsZ) {
    gps.setPosition({0.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]});
    movement.elevate(-15.0 * cm);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 35.0);
}

// elevate() is purely vertical; X and Y stay.
TEST_F(SimulationRun, ElevateDoesNotChangeXOrY) {
    gps.setPosition({10.0 * x_extent[cm], 20.0 * y_extent[cm], 0.0 * z_extent[cm]});
    movement.elevate(5.0 * cm);
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 20.0);
}

// elevate() always succeeds in the mock (no hidden map injected).
TEST_F(SimulationRun, ElevateReturnsSuccess) {
    const auto result = movement.elevate(5.0 * cm);
    EXPECT_TRUE(result.success);
}

// rotate (wraps the heading into [0,360)) --------------------------------------

// Left rotation adds the angle to the heading.
TEST_F(SimulationRun, RotateLeftAddsAngle) {
    gps.setHeading({10.0 * deg, 0.0 * deg});
    movement.rotate(common::types::RotationDirection::Left, 30.0 * deg);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 40.0, 1e-9);
}

// Right rotation subtracts the angle from the heading.
TEST_F(SimulationRun, RotateRightSubtractsAngle) {
    gps.setHeading({40.0 * deg, 0.0 * deg});
    movement.rotate(common::types::RotationDirection::Right, 30.0 * deg);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 10.0, 1e-9);
}

// Rotating past 360° wraps back into [0,360): 350° + 20° = 10°.
TEST_F(SimulationRun, RotateWrapsPast360) {
    gps.setHeading({350.0 * deg, 0.0 * deg});
    movement.rotate(common::types::RotationDirection::Left, 20.0 * deg);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 10.0, 1e-9);
}

// Rotating below 0° wraps to the positive equivalent: 10° - 30° = 340°.
TEST_F(SimulationRun, RotateWrapsNegativeToPositive) {
    gps.setHeading({10.0 * deg, 0.0 * deg});
    movement.rotate(common::types::RotationDirection::Right, 30.0 * deg);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 340.0, 1e-9);
}

// rotate() changes only the horizontal heading; altitude is preserved.
TEST_F(SimulationRun, RotatePreservesAltitude) {
    gps.setHeading({0.0 * deg, 15.0 * deg});
    movement.rotate(common::types::RotationDirection::Left, 30.0 * deg);
    EXPECT_NEAR(gps.heading().altitude.numerical_value_in(deg), 15.0, 1e-9);
}

// rotate() always succeeds — it never throws, even with a hidden map injected.
TEST_F(SimulationRun, RotateReturnsSuccess) {
    const auto result = movement.rotate(common::types::RotationDirection::Left, 10.0 * deg);
    EXPECT_TRUE(result.success);
}

// MockGPS get/set --------------------------------------------------------------

// The fixture's GPS starts at the world origin.
TEST_F(SimulationRun, GpsInitialPositionIsOrigin) {
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 0.0);
}

// The fixture's GPS starts at heading 0° horizontal / 0° altitude.
TEST_F(SimulationRun, GpsInitialHeadingIsZero) {
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 0.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.numerical_value_in(deg), 0.0);
}

// setPosition() is reflected by position().
TEST_F(SimulationRun, SetPositionUpdatesPosition) {
    gps.setPosition({1.0 * x_extent[cm], 2.0 * y_extent[cm], 3.0 * z_extent[cm]});
    EXPECT_DOUBLE_EQ(gps.position().x.numerical_value_in(cm), 1.0);
    EXPECT_DOUBLE_EQ(gps.position().y.numerical_value_in(cm), 2.0);
    EXPECT_DOUBLE_EQ(gps.position().z.numerical_value_in(cm), 3.0);
}

// setHeading() is reflected by heading().
TEST_F(SimulationRun, SetHeadingUpdatesHeading) {
    gps.setHeading({90.0 * deg, 5.0 * deg});
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.numerical_value_in(deg), 5.0);
}

// movement collision against the hidden ground-truth map (drone size) ----------

namespace {

// A 1 cm/voxel hidden map, all Unmapped, sized side_cm per axis. Built with the ex3 config-only
// Map3DImpl constructor (every voxel starts Unmapped); the caller marks walls with setOcc().
Map3DImpl makeHiddenMap(double side_cm = 10.0) {
    common::types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{};
    cfg.boundaries = {
        0.0 * x_extent[cm], side_cm * x_extent[cm],
        0.0 * y_extent[cm], side_cm * y_extent[cm],
        0.0 * z_extent[cm], side_cm * z_extent[cm],
    };
    return Map3DImpl(cfg);
}

// Marks grid cell (ix,iy,iz) Occupied (addresses the cell centre so atVoxel is unambiguous).
void setOcc(Map3DImpl& m, int ix, int iy, int iz) {
    m.set({(static_cast<double>(ix) + 0.5) * x_extent[cm],
           (static_cast<double>(iy) + 0.5) * y_extent[cm],
           (static_cast<double>(iz) + 0.5) * z_extent[cm]},
          common::types::VoxelOccupancy::Occupied);
}

// World position at the centre of grid cell (ix,iy,iz) for a 1 cm/voxel map.
Position3D cellCentre(int ix, int iy, int iz) {
    return {(static_cast<double>(ix) + 0.5) * x_extent[cm],
            (static_cast<double>(iy) + 0.5) * y_extent[cm],
            (static_cast<double>(iz) + 0.5) * z_extent[cm]};
}

} // namespace

// A large drone (radius 1 cm) advancing toward a wall one voxel ahead is refused: the footprint
// would overlap the Occupied voxel, so advance() THROWS (ex3 row-5 semantics) and the position
// does not change.
TEST_F(SimulationRun, AdvanceRefusedWhenLargeDroneFootprintHitsWall) {
    Map3DImpl hidden = makeHiddenMap();
    for (int y = 0; y <= 4; ++y)
        for (int z = 0; z <= 4; ++z)
            setOcc(hidden, 3, y, z); // wall plane at x = 3

    MockGPS g{cellCentre(1, 2, 2), {0.0 * deg, 0.0 * deg}, 1.0 * cm}; // facing +X
    MockMovement mv{g, hidden, 1.0 * cm};

    // dest = centre of cell (2,2,2), adjacent to the wall
    EXPECT_THROW((void)mv.advance(1.0 * cm), std::runtime_error);
    EXPECT_DOUBLE_EQ(g.position().x.numerical_value_in(cm), 1.5) << "drone must not have moved";
    EXPECT_DOUBLE_EQ(g.position().y.numerical_value_in(cm), 2.5);
    EXPECT_DOUBLE_EQ(g.position().z.numerical_value_in(cm), 2.5);
}

// The blocked-move exception carries the machine-readable collision marker: its message starts
// with "DRONE_HITS_OBSTACLE", which the drone controller uses to classify the failure.
TEST_F(SimulationRun, BlockedMoveExceptionMessageStartsWithDroneHitsObstacle) {
    Map3DImpl hidden = makeHiddenMap();
    for (int y = 0; y <= 4; ++y)
        for (int z = 0; z <= 4; ++z)
            setOcc(hidden, 3, y, z);

    MockGPS g{cellCentre(1, 2, 2), {0.0 * deg, 0.0 * deg}, 1.0 * cm};
    MockMovement mv{g, hidden, 1.0 * cm};

    try {
        (void)mv.advance(1.0 * cm);
        FAIL() << "advance into a wall must throw std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string what = e.what();
        EXPECT_TRUE(what.starts_with("DRONE_HITS_OBSTACLE"))
            << "exception message must start with DRONE_HITS_OBSTACLE, got: " << what;
    }
}

// A small drone (radius 0.5 cm) makes the same move: its footprint only touches the wall face,
// which still fits, so the advance succeeds and the position updates.
TEST_F(SimulationRun, AdvanceAllowedForSmallDroneInSameGap) {
    Map3DImpl hidden = makeHiddenMap();
    for (int y = 0; y <= 4; ++y)
        for (int z = 0; z <= 4; ++z)
            setOcc(hidden, 3, y, z);

    MockGPS g{cellCentre(1, 2, 2), {0.0 * deg, 0.0 * deg}, 1.0 * cm};
    MockMovement mv{g, hidden, 0.5 * cm};

    const auto result = mv.advance(1.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(g.position().x.numerical_value_in(cm), 2.5);
}

// With no obstacle ahead, even the large drone advances normally (collision check is not a blanket
// veto — it only fires on real geometry).
TEST_F(SimulationRun, AdvanceSucceedsInOpenSpaceForLargeDrone) {
    Map3DImpl hidden = makeHiddenMap(); // all Unmapped, no walls

    MockGPS g{cellCentre(1, 2, 2), {0.0 * deg, 0.0 * deg}, 1.0 * cm};
    MockMovement mv{g, hidden, 1.0 * cm};

    const auto result = mv.advance(1.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(g.position().x.numerical_value_in(cm), 2.5);
}

// elevate() is collision-checked too: a large drone climbing into a ceiling one voxel above
// THROWS, and the altitude does not change.
TEST_F(SimulationRun, ElevateRefusedWhenLargeDroneFootprintHitsCeiling) {
    Map3DImpl hidden = makeHiddenMap();
    for (int x = 0; x <= 4; ++x)
        for (int y = 0; y <= 4; ++y)
            setOcc(hidden, x, y, 3); // ceiling plane at z = 3

    MockGPS g{cellCentre(2, 2, 1), {0.0 * deg, 0.0 * deg}, 1.0 * cm};
    MockMovement mv{g, hidden, 1.0 * cm};

    // dest = centre of cell (2,2,2), under the ceiling
    EXPECT_THROW((void)mv.elevate(1.0 * cm), std::runtime_error);
    EXPECT_DOUBLE_EQ(g.position().z.numerical_value_in(cm), 1.5) << "drone must not have moved";
    EXPECT_DOUBLE_EQ(g.position().x.numerical_value_in(cm), 2.5);
    EXPECT_DOUBLE_EQ(g.position().y.numerical_value_in(cm), 2.5);
}

// The same climb succeeds for a small drone whose footprint clears the ceiling.
TEST_F(SimulationRun, ElevateAllowedForSmallDroneUnderSameCeiling) {
    Map3DImpl hidden = makeHiddenMap();
    for (int x = 0; x <= 4; ++x)
        for (int y = 0; y <= 4; ++y)
            setOcc(hidden, x, y, 3);

    MockGPS g{cellCentre(2, 2, 1), {0.0 * deg, 0.0 * deg}, 1.0 * cm};
    MockMovement mv{g, hidden, 0.5 * cm};

    const auto result = mv.elevate(1.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(g.position().z.numerical_value_in(cm), 2.5);
}

// Back-compat: the plain (gps-only) constructor performs no collision checking, so a move that the
// collision-aware ctor would refuse still succeeds. Guards the always-succeeds path the fixture
// relies on.
TEST_F(SimulationRun, PlainCtorMovementSkipsCollisionChecking) {
    MockGPS g{cellCentre(1, 2, 2), {0.0 * deg, 0.0 * deg}, 1.0 * cm};
    MockMovement mv{g}; // no hidden map injected

    const auto result = mv.advance(1.0 * cm);
    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(g.position().x.numerical_value_in(cm), 2.5);
}
