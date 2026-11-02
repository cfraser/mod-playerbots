/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <utility>

#include "JumpTrajectory.h"
#include "gtest/gtest.h"

using namespace JumpTrajectory;

namespace
{
constexpr float RUN_SPEED = 7.0f;
constexpr float RADIUS = DEFAULT_COLLISION_RADIUS;
constexpr uint32 RAYS_PER_CHORD = 5;  // along the body's four leading edges and through its front

// Terrain from a height function, plus optional flat platforms, wall planes, boxes, sloped model floors, a ceiling
// plane and a liquid surface. Platforms and slopes are found only within the search distance, like VMAP floors; terrain
// is found from anywhere above it, like the heightmap. Rays meet planes, boxes and slopes but not terrain or platforms,
// as in the core.
struct FakeWorld : CollisionProbe
{
    struct Platform
    {
        float minX, maxX, z;
    };

    // The points p with Dot(normal, p) == offset.
    struct Plane
    {
        Point normal;
        float offset;
    };

    struct Box
    {
        Point min, max;
    };

    std::function<float(float, float)> terrain = [](float, float) { return 0.0f; };
    std::vector<Platform> platforms;
    std::vector<Plane> planes;
    std::vector<Box> boxes;
    std::vector<Plane> slopes;
    float wallX = 1e9f;
    float ceilingZ = 1e9f;
    float liquidZ = NO_HEIGHT;
    float liquidFromX = -1e9f;
    float voidZ = -1e9f;
    float voidSlope = 0.0f;  // the map's lowest point rises this much per yard of x
    mutable uint32 rays = 0;

    static float Along(Plane const& plane, Point const& point)
    {
        return plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z - plane.offset;
    }

    // Where the ray enters the box, or leaves it when it starts inside, as VMAP's two-sided triangles are hit.
    static float BoxHit(Box const& box, Point const& from, Point const& to)
    {
        float const start[3] = {from.x, from.y, from.z};
        float const delta[3] = {to.x - from.x, to.y - from.y, to.z - from.z};
        float const low[3] = {box.min.x, box.min.y, box.min.z};
        float const high[3] = {box.max.x, box.max.y, box.max.z};
        float enter = -1e9f;
        float leave = 1e9f;
        for (uint32 axis = 0; axis < 3; ++axis)
        {
            if (std::abs(delta[axis]) < 1e-9f)
            {
                if (start[axis] < low[axis] || start[axis] > high[axis])
                    return 2.0f;

                continue;
            }

            float near = (low[axis] - start[axis]) / delta[axis];
            float far = (high[axis] - start[axis]) / delta[axis];
            if (near > far)
                std::swap(near, far);

            enter = std::max(enter, near);
            leave = std::min(leave, far);
        }

        if (enter > leave || leave < 0.0f)
            return 2.0f;

        return enter >= 0.0f ? enter : leave;
    }

    bool RayHit(Point const& from, Point const& to, Point& hit) const override
    {
        ++rays;
        float best = 2.0f;
        if ((from.x - wallX) * (to.x - wallX) < 0.0f)
            best = (wallX - from.x) / (to.x - from.x);
        if ((from.z - ceilingZ) * (to.z - ceilingZ) < 0.0f)
            best = std::min(best, (ceilingZ - from.z) / (to.z - from.z));
        for (std::vector<Plane> const* list : {&planes, &slopes})
            for (Plane const& plane : *list)
            {
                float const a = Along(plane, from);
                float const b = Along(plane, to);
                if (a * b < 0.0f)
                    best = std::min(best, a / (a - b));
            }
        for (Box const& box : boxes)
            best = std::min(best, BoxHit(box, from, to));
        if (best > 1.0f)
            return false;

        hit = {from.x + (to.x - from.x) * best, from.y + (to.y - from.y) * best, from.z + (to.z - from.z) * best};
        return true;
    }

    float TerrainZ(float x, float y) const override { return terrain(x, y); }

    float FloorZ(float x, float y, float zFrom, float maxSearch) const override
    {
        float best = NO_HEIGHT;
        float const ground = terrain(x, y);
        if (ground > NO_HEIGHT && ground <= zFrom)
            best = ground;
        for (Platform const& platform : platforms)
            if (x >= platform.minX && x <= platform.maxX && platform.z <= zFrom && zFrom - platform.z <= maxSearch)
                best = std::max(best, platform.z);
        for (Plane const& slope : slopes)
        {
            float const z = (slope.offset - slope.normal.x * x - slope.normal.y * y) / slope.normal.z;
            if (z <= zFrom && zFrom - z <= maxSearch)
                best = std::max(best, z);
        }
        return best;
    }

    // Like the core, no water below the bed.
    float LiquidZ(float x, float y, float z) const override
    {
        return x >= liquidFromX && z <= liquidZ && z >= terrain(x, y) ? liquidZ : NO_HEIGHT;
    }

    float MinZ(float x, float) const override { return voidZ + voidSlope * x; }
};

Launch RunningJump(Point from)
{
    Launch launch;
    launch.position = from;
    launch.velocity.x = RUN_SPEED;
    launch.velocity.z = JUMP_VELOCITY;
    return launch;
}

Launch Fall(Point from)
{
    Launch launch;
    launch.position = from;
    return launch;
}

// Terrain rising this steeply toward -x from flat ground at x = 5.
std::function<float(float, float)> SlopeFromFive(float gradient)
{
    return [gradient](float x, float) { return std::max(0.0f, gradient * (5.0f - x)); };
}

// Every point of the segments that start and end on the terrain stays near it: a slide keeps to its slope.
void ExpectSlideKeepsToTheTerrain(Trajectory const& trajectory, FakeWorld const& world)
{
    auto clearance = [&](Point const& point) { return point.z - world.terrain(point.x, point.y); };
    uint32 onSlope = 0;
    for (Segment const& segment : trajectory.segments)
    {
        if (clearance(segment.start) > 0.25f || clearance(segment.end) > 0.25f)
            continue;

        ++onSlope;
        for (uint32 ms = 0; ms <= segment.durationMs; ms += 10)
            EXPECT_NEAR(clearance(PositionAt(segment, ms)), 0.0f, 0.25f);
    }

    EXPECT_GE(onSlope, 1u);
}

uint32 TotalMs(Trajectory const& trajectory)
{
    uint32 total = 0;
    for (Segment const& segment : trajectory.segments)
        total += segment.durationMs;
    return total;
}

constexpr float FLAT_FLIGHT_SECONDS = 2.0f * JUMP_VELOCITY / GRAVITY;
}  // namespace

TEST(JumpTrajectory, FlatJumpIsOneExactArc)
{
    FakeWorld world;
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 1u);
    EXPECT_NEAR(TotalMs(trajectory), FLAT_FLIGHT_SECONDS * 1000.0f, 2.0f);
    EXPECT_NEAR(trajectory.landing.x, RUN_SPEED * FLAT_FLIGHT_SECONDS, 0.02f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
    EXPECT_EQ(trajectory.segments[0].kind, SegmentKind::Parabolic);
}

TEST(JumpTrajectory, InclineLandsEarly)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return 0.3f * x; };
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, RUN_SPEED * FLAT_FLIGHT_SECONDS - 0.5f);
    EXPECT_NEAR(trajectory.landing.z, 0.3f * trajectory.landing.x, 0.05f);
}

TEST(JumpTrajectory, DeclineLandsLate)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return -0.3f * x; };
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, RUN_SPEED * FLAT_FLIGHT_SECONDS + 0.5f);
    EXPECT_NEAR(trajectory.landing.z, -0.3f * trajectory.landing.x, 0.05f);
}

TEST(JumpTrajectory, WallStopsHorizontalTravelThenFallsStraightDown)
{
    FakeWorld world;
    world.platforms.push_back({-10.0f, 2.0f, 5.0f});
    world.wallX = 4.0f;
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 5.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    Segment const& fall = trajectory.segments[1];
    EXPECT_FLOAT_EQ(fall.start.x, fall.end.x);
    // The body's front stops short of the wall.
    EXPECT_LT(trajectory.landing.x, 4.0f - RADIUS);
    EXPECT_GT(trajectory.landing.x, 4.0f - RADIUS - 2.0f * CONTACT_GAP);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, CeilingBonkKillsLiftButKeepsMomentum)
{
    FakeWorld world;
    world.ceilingZ = 3.0f;
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    EXPECT_FLOAT_EQ(trajectory.segments[1].startVz, 0.0f);
    EXPECT_LT(trajectory.segments[0].end.z + 2.03f, 3.0f);
    EXPECT_GT(trajectory.landing.x, trajectory.segments[0].end.x);
}

TEST(JumpTrajectory, VerticalJumpSplitsAtTheApex)
{
    FakeWorld world;
    Launch launch;
    launch.velocity.z = JUMP_VELOCITY;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    EXPECT_NEAR(trajectory.segments[0].end.z, JUMP_VELOCITY * JUMP_VELOCITY / (2.0f * GRAVITY), 0.01f);
    EXPECT_FLOAT_EQ(trajectory.landing.x, 0.0f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, FallFromTheGroundLandsInPlace)
{
    FakeWorld world;
    Launch launch;
    launch.position = {1.0f, 2.0f, -0.2f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_TRUE(trajectory.segments.empty());
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, LiquidCatchesTheFall)
{
    FakeWorld world;
    world.terrain = [](float, float) { return -20.0f; };
    world.platforms.push_back({-10.0f, 1.0f, 10.0f});
    world.liquidZ = 0.0f;
    world.liquidFromX = 1.0f;
    Launch const launch = RunningJump({0.0f, 0.0f, 10.0f});
    Trajectory trajectory = Sweep(launch, world);

    // A client swims, and stops falling, once it sinks to swim depth.
    ASSERT_EQ(trajectory.outcome, Outcome::Liquid);
    EXPECT_FLOAT_EQ(trajectory.landing.z, -SWIM_DEPTH * launch.collisionHeight);
}

// Long falls take steps of many yards; one that passes the surface and ends below the bed still lands in the water.
TEST(JumpTrajectory, LiquidCatchesALongFall)
{
    FakeWorld world;
    world.liquidZ = 5.0f;
    for (uint32 height = 60; height <= 250; ++height)
    {
        Launch launch;
        launch.position = {0.0f, 0.0f, static_cast<float>(height)};
        Trajectory trajectory = Sweep(launch, world);

        ASSERT_EQ(trajectory.outcome, Outcome::Liquid) << "falling from " << height;
        EXPECT_FLOAT_EQ(trajectory.landing.z, 5.0f - SWIM_DEPTH * launch.collisionHeight);
    }
}

// A client launched under the surface rises through the water and swims on from the apex.
TEST(JumpTrajectory, LaunchUnderWaterSwimsOnFromTheApex)
{
    FakeWorld world;
    world.terrain = [](float, float) { return -50.0f; };
    world.liquidZ = 0.0f;
    Launch launch;
    launch.position = {0.0f, 0.0f, -10.0f};
    launch.velocity = {5.0f, 0.0f, 7.0f};
    Trajectory trajectory = Sweep(launch, world);

    float const apexTime = 7.0f / GRAVITY;
    ASSERT_EQ(trajectory.outcome, Outcome::Liquid);
    EXPECT_NEAR(trajectory.landing.x, 5.0f * apexTime, 0.001f);
    EXPECT_NEAR(trajectory.landing.z, -10.0f + 0.5f * 7.0f * apexTime, 0.001f);
}

// A fall that starts under the surface, such as a dismount while swimming, ends where it starts.
TEST(JumpTrajectory, FallUnderWaterSwimsAtOnce)
{
    FakeWorld world;
    world.terrain = [](float, float) { return -50.0f; };
    world.liquidZ = 0.0f;
    Launch launch;
    launch.position = {0.0f, 0.0f, -10.0f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Liquid);
    EXPECT_TRUE(trajectory.segments.empty());
    EXPECT_FLOAT_EQ(trajectory.landing.z, -10.0f);
}

// Water too shallow to swim in does not catch a fall, short or long.
TEST(JumpTrajectory, ShallowWaterLandsOnTheBed)
{
    FakeWorld world;
    world.liquidZ = 1.0f;
    for (uint32 height = 2; height <= 200; ++height)
    {
        Launch launch;
        launch.position = {0.0f, 0.0f, static_cast<float>(height)};
        Trajectory trajectory = Sweep(launch, world);

        ASSERT_EQ(trajectory.outcome, Outcome::Landed) << "falling from " << height;
        EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
    }
}

// Water under a floor does not catch a fall that lands on the floor.
TEST(JumpTrajectory, FloorAboveTheWaterCatchesTheFall)
{
    FakeWorld world;
    world.terrain = [](float, float) { return -20.0f; };
    world.platforms.push_back({-10.0f, 10.0f, 3.0f});
    world.liquidZ = 0.0f;
    Launch launch;
    launch.position = {0.0f, 0.0f, 40.0f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 3.0f);
}

TEST(JumpTrajectory, RefusesADropPastTheCap)
{
    FakeWorld world;
    world.platforms.push_back({-10.0f, 1.0f, 10.0f});
    Launch launch = RunningJump({0.0f, 0.0f, 10.0f});
    launch.minZ = 4.0f;

    EXPECT_EQ(Sweep(launch, world).outcome, Outcome::DropCap);
}

TEST(JumpTrajectory, RefusesWhenThereIsNoFloor)
{
    FakeWorld world;
    world.terrain = [](float, float) { return NO_HEIGHT; };
    Launch launch = RunningJump({0.0f, 0.0f, 10.0f});
    launch.minZ = -500.0f;
    EXPECT_EQ(Sweep(launch, world).outcome, Outcome::DropCap);

    launch.minZ = NO_HEIGHT;
    EXPECT_EQ(Sweep(launch, world).outcome, Outcome::StepCap);
}

// A client falls on until it passes below the map, where the server kills it.
TEST(JumpTrajectory, FallsIntoTheVoid)
{
    FakeWorld world;
    world.terrain = [](float, float) { return NO_HEIGHT; };
    world.voidZ = -500.0f;
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 10.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Void);
    EXPECT_FALSE(trajectory.Landed());
    EXPECT_TRUE(trajectory.Ends());
    EXPECT_LT(trajectory.landing.z, -500.0f - VOID_DEPTH);
}

// Outland's lowest point can rise over a yard per yard, so the fall ends below it where the bot ends, not where it
// crossed.
TEST(JumpTrajectory, VoidFallEndsBelowASlopingLowestPoint)
{
    FakeWorld world;
    world.terrain = [](float, float) { return NO_HEIGHT; };
    world.voidZ = -300.0f;
    world.voidSlope = 1.33f;
    Launch launch = RunningJump({0.0f, 0.0f, 10.0f});
    launch.velocity.x = 20.0f;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Void);
    EXPECT_LT(trajectory.landing.z, world.MinZ(trajectory.landing.x, trajectory.landing.y) - VOID_DEPTH);
}

// A deliberate drop is refused at its cap before it reaches the void.
TEST(JumpTrajectory, DropCapComesBeforeTheVoid)
{
    FakeWorld world;
    world.terrain = [](float, float) { return NO_HEIGHT; };
    world.voidZ = -500.0f;
    Launch launch = RunningJump({0.0f, 0.0f, 10.0f});
    launch.minZ = 4.0f;
    EXPECT_EQ(Sweep(launch, world).outcome, Outcome::DropCap);
}

// Floors and liquid above the map's lowest point still catch the fall.
TEST(JumpTrajectory, FloorAboveTheVoidCatchesTheFall)
{
    FakeWorld world;
    world.voidZ = -1.0f;
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 10.0f}), world);
    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);

    world.terrain = [](float, float) { return -50.0f; };
    world.liquidZ = 0.0f;
    world.voidZ = -10.0f;
    EXPECT_EQ(Sweep(RunningJump({0.0f, 0.0f, 10.0f}), world).outcome, Outcome::Liquid);
}

TEST(JumpTrajectory, LongFallReachesTerminalVelocityOnAStraightLine)
{
    FakeWorld world;
    Launch launch;
    launch.position = {0.0f, 0.0f, 300.0f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    EXPECT_EQ(trajectory.segments[0].kind, SegmentKind::Parabolic);
    EXPECT_EQ(trajectory.segments[1].kind, SegmentKind::Linear);
    EXPECT_NEAR(trajectory.segments[0].durationMs, TERMINAL_VELOCITY / GRAVITY * 1000.0f, 2.0f);
    float const terminalDrop = TERMINAL_VELOCITY * TERMINAL_VELOCITY / (2.0f * GRAVITY);
    float const expectedMs = (TERMINAL_VELOCITY / GRAVITY + (300.0f - terminalDrop) / TERMINAL_VELOCITY) * 1000.0f;
    EXPECT_NEAR(TotalMs(trajectory), expectedMs, 5.0f);
}

TEST(JumpTrajectory, SlowFallGlidesFarWithFewRays)
{
    FakeWorld world;
    world.platforms.push_back({-10.0f, 1.0f, 40.0f});
    Launch launch = RunningJump({0.0f, 0.0f, 40.0f});
    launch.terminalVelocity = SAFE_FALL_TERMINAL_VELOCITY;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, 40.0f);
    EXPECT_EQ(trajectory.segments.back().kind, SegmentKind::Linear);
    EXPECT_LT(world.rays, 32u * RAYS_PER_CHORD);
}

TEST(JumpTrajectory, HoverLandsAboveTheFloor)
{
    FakeWorld world;
    Launch launch = RunningJump({0.0f, 0.0f, 1.0f});
    launch.landingOffset = 1.0f;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 1.0f);
}

// A hop that reaches a ledge with its feet more than a step height below the top meets the ledge as a wall.
TEST(JumpTrajectory, LedgeAboveTheStepHeightIsAWall)
{
    FakeWorld world;
    world.platforms.push_back({1.0f, 10.0f, 0.9f});
    Launch launch = RunningJump({0.0f, 0.0f, 0.0f});
    launch.velocity.z = 2.0f;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 1.0f);
    EXPECT_GT(trajectory.landing.x, 0.95f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, LedgeBelowTheStepHeightIsLandedOn)
{
    FakeWorld world;
    world.platforms.push_back({1.0f, 10.0f, 0.4f});
    Launch launch = RunningJump({0.0f, 0.0f, 0.0f});
    launch.velocity.z = 2.0f;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GE(trajectory.landing.x, 1.0f);
    EXPECT_LT(trajectory.landing.x, 1.05f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.4f);
}

TEST(JumpTrajectory, CeilingStopsAStandingJump)
{
    FakeWorld world;
    world.ceilingZ = 2.83f;
    Launch launch;
    launch.velocity.z = JUMP_VELOCITY;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    float const headAtStop = trajectory.segments[0].end.z + launch.collisionHeight;
    EXPECT_LT(headAtStop, world.ceilingZ);
    EXPECT_GT(headAtStop, world.ceilingZ - 2.0f * CONTACT_GAP);
    EXPECT_FLOAT_EQ(trajectory.segments[1].startVz, 0.0f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

// A knockup rises over a yard per chord, so the body's middle reaches the ceiling in the same chord as the head.
TEST(JumpTrajectory, CeilingStopsAFastKnockup)
{
    FakeWorld world;
    world.ceilingZ = 4.8f;
    Launch launch;
    launch.velocity = {0.11f, 0.0f, std::sqrt(2.0f * GRAVITY * 20.0f)};  // a knockup's minimum drift
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    EXPECT_LT(trajectory.segments[0].end.z + launch.collisionHeight, world.ceilingZ);
    EXPECT_FLOAT_EQ(trajectory.segments[1].startVz, 0.0f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

// At terminal velocity a chord is half a second long; the stop still comes at the cliff, not at the chord's start.
TEST(JumpTrajectory, CliffStopsAFastFallWhereItRises)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return x < 5.0f ? 0.0f : 300.0f; };
    Launch launch;
    launch.position = {0.0f, 0.0f, 200.0f};
    launch.velocity = {RUN_SPEED, 0.0f, -TERMINAL_VELOCITY};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 5.0f - RADIUS);
    EXPECT_GT(trajectory.landing.x, 5.0f - RADIUS - 0.05f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, WaterWalkerLandsOnTheSurface)
{
    FakeWorld world;
    world.terrain = [](float, float) { return -20.0f; };
    world.platforms.push_back({-10.0f, 1.0f, 10.0f});
    world.liquidZ = 0.0f;
    world.liquidFromX = 1.0f;
    Launch launch = RunningJump({0.0f, 0.0f, 10.0f});
    launch.waterWalking = true;
    Trajectory trajectory = Sweep(launch, world);

    // A floor, so it takes fall damage.
    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, WaterWalkerJumpsOffTheSurface)
{
    FakeWorld world;
    world.terrain = [](float, float) { return -20.0f; };
    world.liquidZ = 0.0f;
    Launch launch = RunningJump({0.0f, 0.0f, 0.0f});
    launch.waterWalking = true;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_NEAR(trajectory.landing.x, RUN_SPEED * FLAT_FLIGHT_SECONDS, 0.02f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);

    // Standing on it, it does not fall in.
    launch.velocity = {};
    trajectory = Sweep(launch, world);
    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_TRUE(trajectory.segments.empty());
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

// A walkable hill is landed on where the arc meets it, however fast the body comes in.
TEST(JumpTrajectory, FastArcLandsOnAWalkableHill)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return std::max(0.0f, x - 3.0f); };  // 45 degrees
    Launch launch;
    launch.velocity = {20.0f, 0.0f, 3.0f};
    Trajectory trajectory = Sweep(launch, world);

    // 3t - g t^2 / 2 = 20t - 3
    float const time = (-17.0f + std::sqrt(17.0f * 17.0f + 4.0f * 0.5f * GRAVITY * 3.0f)) / GRAVITY;
    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_NEAR(trajectory.landing.x, 20.0f * time, 0.05f);
    EXPECT_NEAR(trajectory.landing.z, 20.0f * time - 3.0f, 0.05f);
}

// The body's top meets an overhang that its middle passes under.
TEST(JumpTrajectory, OverhangAtHeadHeightIsAWall)
{
    FakeWorld world;
    world.boxes.push_back({{3.0f, -10.0f, 3.0f}, {10.0f, 10.0f, 6.0f}});
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 3.0f - RADIUS);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

// A steep fall stops with the body's side at the wall, not its middle.
TEST(JumpTrajectory, SteepFallStopsBesideTheWall)
{
    FakeWorld world;
    world.wallX = 1.0f;
    Launch launch;
    launch.position = {0.0f, 0.0f, 50.0f};
    launch.velocity.x = 0.5f;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 1.0f - RADIUS);
    EXPECT_GT(trajectory.landing.x, 1.0f - RADIUS - 2.0f * CONTACT_GAP);
}

// The client keeps the motion along the wall and drops the part into it.
TEST(JumpTrajectory, SlidesAlongAnAngledWall)
{
    FakeWorld world;
    float const half = std::sqrt(0.5f);
    world.planes.push_back({{half, -half, 0.0f}, 3.0f * half});  // x - y = 3
    Launch launch;
    launch.velocity = {10.0f, 0.0f, 5.0f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_EQ(trajectory.segments.size(), 2u);
    Segment const& slide = trajectory.segments[1];
    float const seconds = slide.durationMs / 1000.0f;
    EXPECT_NEAR((slide.end.x - slide.start.x) / seconds, 5.0f, 0.05f);
    EXPECT_NEAR((slide.end.y - slide.start.y) / seconds, 5.0f, 0.05f);
    EXPECT_LT(trajectory.landing.x - trajectory.landing.y, 3.0f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, SlidesAlongACliff)
{
    FakeWorld world;
    world.terrain = [](float x, float y) { return x - y < 3.0f ? 0.0f : 300.0f; };
    Launch launch;
    launch.position = {0.0f, 0.0f, 1.0f};
    launch.velocity = {10.0f, 0.0f, 5.0f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.y, 1.0f);
    EXPECT_LT(trajectory.landing.x - trajectory.landing.y, 3.0f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectory, CornerStopsTheSlide)
{
    FakeWorld world;
    world.wallX = 3.0f;
    world.planes.push_back({{0.0f, 1.0f, 0.0f}, 1.0f});  // y = 1
    Launch launch;
    launch.velocity = {10.0f, 5.0f, 5.0f};
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 3.0f - RADIUS);
    EXPECT_LT(trajectory.landing.y, 1.0f - RADIUS);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

// Past the end of the wall, a held key gets the launch's speed and heading back; a knockback keeps the slide.
TEST(JumpTrajectory, HeldKeyTurnsBackPastTheWall)
{
    FakeWorld world;
    world.boxes.push_back({{2.0f, -10.0f, -1.0f}, {3.0f, 1.0f, 10.0f}});
    Launch launch = RunningJump({0.0f, 0.0f, 0.0f});
    launch.velocity.x = 6.0f;
    launch.velocity.y = 3.0f;
    Trajectory const slid = Sweep(launch, world);

    ASSERT_EQ(slid.outcome, Outcome::Landed);
    EXPECT_LT(slid.landing.x, 2.0f - RADIUS);
    EXPECT_GT(slid.landing.y, 2.0f);

    launch.moveKeyHeld = true;
    Trajectory const held = Sweep(launch, world);

    ASSERT_EQ(held.outcome, Outcome::Landed);
    EXPECT_GT(held.landing.x, 3.0f);
    EXPECT_GT(held.landing.y, 1.0f + RADIUS);
    ASSERT_GE(held.segments.size(), 3u);
    Segment const& last = held.segments.back();
    EXPECT_NEAR((last.end.x - last.start.x) / (last.end.y - last.start.y), 2.0f, 0.01f);
}

TEST(JumpTrajectory, PostNarrowerThanTheBodyIsAWall)
{
    FakeWorld world;
    world.boxes.push_back({{3.0f, -0.05f, -1.0f}, {3.1f, 0.05f, 10.0f}});
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 3.0f - RADIUS);
}

// A bot thrown with its front already against a wall does not pass through it.
TEST(JumpTrajectory, LaunchAgainstAWallStaysOnItsSide)
{
    FakeWorld world;
    world.wallX = 0.2f;
    Launch launch;
    launch.position = {0.0f, 0.0f, 5.0f};
    launch.velocity.x = RUN_SPEED;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 0.2f);
}

// Too steep to stand on: the client lets the body fall freely and pushes it out sideways along the slope.
TEST(JumpTrajectory, SteepSlopeSlidesToItsFoot)
{
    FakeWorld world;
    world.terrain = SlopeFromFive(2.0f);  // 63 degrees
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 20.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, 4.8f);
    EXPECT_LT(trajectory.landing.x, 5.4f);
    EXPECT_NEAR(trajectory.landing.z, 0.0f, 0.001f);
    EXPECT_NEAR(TotalMs(trajectory), std::sqrt(2.0f * 20.0f / GRAVITY) * 1000.0f, 50.0f);
    ExpectSlideKeepsToTheTerrain(trajectory, world);
}

// Chords merge into segments, a few per second, that each keep to the slope.
TEST(JumpTrajectory, LongSlideKeepsToTheSlope)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return std::max(0.0f, 2.0f * (25.0f - x)); };
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 60.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_NEAR(trajectory.landing.x, 25.0f, 0.3f);
    ExpectSlideKeepsToTheTerrain(trajectory, world);
    EXPECT_LE(trajectory.segments.size(), TotalMs(trajectory) / 200u + 3u);
}

// Off a slope that steepens the body lifts clear and falls back onto it, still within the slide's segments.
TEST(JumpTrajectory, SteepeningSlopeSlidesInFewSegments)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return x < 5.0f ? 30.0f - x * x : 5.0f; };
    Trajectory trajectory = Sweep(Fall({1.0f, 0.0f, 35.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_NEAR(trajectory.landing.x, 5.0f, 0.3f);
    EXPECT_LE(trajectory.segments.size(), TotalMs(trajectory) / 200u + 3u);
}

TEST(JumpTrajectory, SlopeJustUnderTheLimitIsStoodOn)
{
    FakeWorld world;
    world.terrain = SlopeFromFive(std::tan(48.0f * 3.14159265f / 180.0f));
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 20.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.x, 2.0f);
}

TEST(JumpTrajectory, SlopeJustOverTheLimitSlides)
{
    FakeWorld world;
    world.terrain = SlopeFromFive(std::tan(52.0f * 3.14159265f / 180.0f));
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 20.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, 4.5f);
}

// The floor beside a ledge's edge is far below: the body stands on the edge.
TEST(JumpTrajectory, LedgeEdgeIsStoodOn)
{
    FakeWorld world;
    world.platforms.push_back({-10.0f, 1.0f, 5.0f});
    Trajectory trajectory = Sweep(Fall({0.95f, 0.0f, 8.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.x, 0.95f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 5.0f);
}

// Below a step's nose the next step is in reach, but the step it lands on is flat.
TEST(JumpTrajectory, StepNoseIsStoodOn)
{
    FakeWorld world;
    world.platforms.push_back({-10.0f, 1.0f, 0.4f});
    Trajectory trajectory = Sweep(Fall({0.95f, 0.0f, 5.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.x, 0.95f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.4f);
}

// No floor beside it on either side tells nothing of its slope: the body stands on it.
TEST(JumpTrajectory, NarrowPostIsStoodOn)
{
    FakeWorld world;
    world.platforms.push_back({0.9f, 1.0f, 5.0f});
    Trajectory trajectory = Sweep(Fall({0.95f, 0.0f, 8.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 5.0f);
}

// Slid to the bottom of a steep valley, the body is wedged between its sides and stops there.
TEST(JumpTrajectory, SteepValleyWedgesTheSlide)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return 3.0f * std::abs(x); };
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 20.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(std::abs(trajectory.landing.x), 0.3f);
    EXPECT_NEAR(trajectory.landing.z, world.terrain(trajectory.landing.x, 0.0f), 0.01f);
}

// One side gentle enough to slide down, the other a cliff: the body is wedged where they meet.
TEST(JumpTrajectory, ValleyUnderACliffWedgesTheSlide)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return x > 0.0f ? 1.5f * x : -6.0f * x; };
    Trajectory trajectory = Sweep(Fall({1.5f, 0.0f, 10.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(std::abs(trajectory.landing.x), 0.3f);
}

// The heightmap's cliff rises above where the floor is searched from: the slide still stops at it.
TEST(JumpTrajectory, SlideIntoATerrainCliffStopsAtIt)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return x > 0.0f ? 2.0f * x : -50.0f * x; };
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 10.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, -0.05f);
    EXPECT_LT(trajectory.landing.x, 0.3f);
}

// A jump up a steep hillside comes back down it: the slope takes the motion into it, as a wall does.
TEST(JumpTrajectory, JumpUpASteepHillSlidesBack)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return std::max(0.0f, 1.43f * (x - 3.0f)); };  // 55 degrees
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_LT(trajectory.landing.x, 3.1f);
    EXPECT_NEAR(trajectory.landing.z, world.terrain(trajectory.landing.x, 0.0f), 0.001f);
    for (size_t i = 1; i < trajectory.segments.size(); ++i)
        EXPECT_LE(trajectory.segments[i].end.z, trajectory.segments[i].start.z);
}

TEST(JumpTrajectory, SlopeIntoAWallPinsTheBody)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return 2.0f * x; };
    world.wallX = 1.0f;
    Trajectory trajectory = Sweep(Fall({3.0f, 0.0f, 20.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, 1.0f + RADIUS);
    EXPECT_LT(trajectory.landing.x, 1.0f + RADIUS + 2.0f * CONTACT_GAP);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 2.0f * trajectory.landing.x);
}

// Past the slope's foot the floor falls away. The push was not speed, so the body drops straight down from there.
TEST(JumpTrajectory, SlideOffAnEdgeFallsStraightDown)
{
    FakeWorld world;
    world.terrain = [](float x, float) { return x < 5.0f ? 2.0f * (5.0f - x) : -20.0f; };
    Trajectory trajectory = Sweep(Fall({2.0f, 0.0f, 20.0f}), world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.landing.z, -20.0f);
    EXPECT_GT(trajectory.landing.x, 5.0f);
    EXPECT_LT(trajectory.landing.x, 5.5f);
    Segment const& last = trajectory.segments.back();
    EXPECT_FLOAT_EQ(last.end.x, last.start.x);
    EXPECT_GT(last.start.z, -1.0f);
    EXPECT_NEAR(TotalMs(trajectory), std::sqrt(2.0f * 40.0f / GRAVITY) * 1000.0f, 50.0f);
}

// Across a steep model slope the body keeps its speed along it. Its box rides above the slope, so its rays do not meet
// the slope at every chord.
TEST(JumpTrajectory, RunAcrossASteepModelSlope)
{
    FakeWorld world;
    float const gradient = 2.75f;  // 70 degrees
    float const length = std::sqrt(gradient * gradient + 1.0f);
    world.slopes.push_back({{gradient / length, 0.0f, 1.0f / length}, 5.0f * gradient / length});  // z = 2.75 (5 - x)
    Launch launch = Fall({2.0f, 0.0f, 20.0f});
    launch.velocity.y = RUN_SPEED;
    Trajectory trajectory = Sweep(launch, world);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_GT(trajectory.landing.x, 4.8f);
    EXPECT_NEAR(trajectory.landing.y, RUN_SPEED * TotalMs(trajectory) / 1000.0f, 0.05f);
    EXPECT_LT(world.rays, 180u);
}

// Jumps, knockbacks and falls over random hills, many too steep to stand on, all come down on the ground.
TEST(JumpTrajectory, RandomHillsAlwaysLandOnTheGround)
{
    std::mt19937 random(1);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    for (uint32 i = 0; i < 2000; ++i)
    {
        float const heightX = unit(random) * 6.0f;
        float const frequencyX = 0.2f + unit(random) * 1.5f;
        float const phaseX = unit(random) * 6.0f;
        float const heightY = unit(random) * 6.0f;
        float const frequencyY = 0.2f + unit(random) * 1.5f;
        float const phaseY = unit(random) * 6.0f;
        float const ridge = unit(random) * 3.0f;
        float const ridgeFrequency = 0.3f + unit(random) * 0.7f;
        FakeWorld world;
        world.terrain = [=](float x, float y)
        {
            return heightX * std::sin(frequencyX * x + phaseX) + heightY * std::sin(frequencyY * y + phaseY) +
                   ridge * std::abs(std::sin(ridgeFrequency * (x + y)));
        };

        Launch launch;
        launch.position = {unit(random) * 10.0f, unit(random) * 10.0f, 0.0f};
        launch.position.z = world.terrain(launch.position.x, launch.position.y) + unit(random) * 40.0f;
        float const heading = unit(random) * 6.2831853f;
        float const speed = unit(random) < 0.3f ? 0.0f : unit(random) * 25.0f;
        launch.velocity = {speed * std::cos(heading), speed * std::sin(heading), unit(random) * 20.0f - 5.0f};
        launch.moveKeyHeld = unit(random) < 0.5f;
        Trajectory const trajectory = Sweep(launch, world);

        ASSERT_EQ(trajectory.outcome, Outcome::Landed) << "launch " << i;
        ASSERT_NEAR(trajectory.landing.z, world.terrain(trajectory.landing.x, trajectory.landing.y), 0.05f)
            << "launch " << i;
        ASSERT_LE(trajectory.segments.size(), TotalMs(trajectory) / 100u + 6u) << "launch " << i;
    }
}

TEST(JumpTrajectorySegment, AmplitudeAndVelocityMatchTheSplineMath)
{
    EXPECT_FLOAT_EQ(ParabolicAmplitude(1000), GRAVITY / 8.0f);
    EXPECT_FLOAT_EQ(ParabolicAmplitude(500), GRAVITY / 32.0f);

    Segment segment{{0.0f, 0.0f, 0.0f}, {3.0f, 4.0f, 0.0f}, 500, 0.0f, SegmentKind::Parabolic};
    EXPECT_FLOAT_EQ(SegmentVelocity(segment), 10.0f);
    segment.durationMs = 0;
    EXPECT_FLOAT_EQ(SegmentVelocity(segment), 0.0f);
}

TEST(JumpTrajectorySegment, SplineFollowsGravityExactly)
{
    FakeWorld world;
    Trajectory trajectory = Sweep(RunningJump({0.0f, 0.0f, 0.0f}), world);
    ASSERT_EQ(trajectory.segments.size(), 1u);
    Segment const& segment = trajectory.segments[0];

    for (uint32 ms = 0; ms <= segment.durationMs; ms += 100)
    {
        float const t = ms / 1000.0f;
        EXPECT_NEAR(PositionAt(segment, ms).z, JUMP_VELOCITY * t - 0.5f * GRAVITY * t * t, 0.01f) << ms;
        EXPECT_NEAR(VerticalSpeedAt(segment, ms), JUMP_VELOCITY - GRAVITY * t, 0.02f) << ms;
    }
}

TEST(JumpTrajectorySegment, MidArcRelaunchContinuesTheSameArc)
{
    Segment segment{{0.0f, 0.0f, 10.0f}, {6.0f, 2.0f, 4.0f}, 900, 3.0f, SegmentKind::Parabolic};
    for (uint32 cut : {0u, 150u, 450u, 899u})
    {
        Segment const rest = Remainder(segment, cut);
        EXPECT_EQ(rest.durationMs, segment.durationMs - cut);
        for (uint32 ms = 0; ms <= rest.durationMs; ms += 50)
        {
            Point const expected = PositionAt(segment, cut + ms);
            Point const actual = PositionAt(rest, ms);
            EXPECT_NEAR(actual.x, expected.x, 0.001f);
            EXPECT_NEAR(actual.y, expected.y, 0.001f);
            EXPECT_NEAR(actual.z, expected.z, 0.001f);
        }
    }
}

namespace
{
Launch Walk(Point from)
{
    Launch launch;
    launch.position = from;
    launch.velocity.x = RUN_SPEED;
    return launch;
}

constexpr float WALK_LIMIT = 10.0f;
}  // namespace

TEST(JumpTrajectoryWalkOff, WalksToTheEdgeThenFalls)
{
    FakeWorld world;
    world.platforms.push_back({-100.0f, 3.0f, 10.0f});
    Trajectory trajectory = WalkOff(Walk({0.0f, 0.0f, 10.0f}), world, WALK_LIMIT);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    ASSERT_GE(trajectory.segments.size(), 2u);
    Segment const& walk = trajectory.segments.front();
    EXPECT_EQ(walk.kind, SegmentKind::Linear);
    EXPECT_FLOAT_EQ(walk.end.x, 3.5f);
    EXPECT_FLOAT_EQ(walk.end.z, 10.0f);
    EXPECT_EQ(walk.durationMs, 500u);
    EXPECT_FLOAT_EQ(trajectory.takeoff.x, 3.5f);
    EXPECT_FLOAT_EQ(trajectory.takeoff.z, 10.0f);

    // Leaves the edge at run speed with no vertical speed.
    float const fallSeconds = std::sqrt(2.0f * 10.0f / GRAVITY);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
    EXPECT_NEAR(trajectory.landing.x, 3.5f + RUN_SPEED * fallSeconds, 0.05f);
}

TEST(JumpTrajectoryWalkOff, FollowsStepsDownToTheEdge)
{
    FakeWorld world;
    world.platforms.push_back({-100.0f, 1.0f, 10.0f});
    world.platforms.push_back({1.0f, 2.0f, 9.6f});
    world.platforms.push_back({2.0f, 3.0f, 9.2f});
    Trajectory trajectory = WalkOff(Walk({0.0f, 0.0f, 10.0f}), world, WALK_LIMIT);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.takeoff.x, 3.5f);
    EXPECT_FLOAT_EQ(trajectory.takeoff.z, 9.2f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

// The fall counts from the edge, above where the walk began.
TEST(JumpTrajectoryWalkOff, ClimbsToTheEdgeThenFalls)
{
    FakeWorld world;
    world.platforms.push_back({-100.0f, 1.0f, 10.0f});
    world.platforms.push_back({1.0f, 2.0f, 10.4f});
    world.platforms.push_back({2.0f, 3.0f, 10.8f});
    Trajectory trajectory = WalkOff(Walk({0.0f, 0.0f, 10.0f}), world, WALK_LIMIT);

    ASSERT_EQ(trajectory.outcome, Outcome::Landed);
    EXPECT_FLOAT_EQ(trajectory.takeoff.x, 3.5f);
    EXPECT_FLOAT_EQ(trajectory.takeoff.z, 10.8f);
    EXPECT_FLOAT_EQ(trajectory.landing.z, 0.0f);
}

TEST(JumpTrajectoryWalkOff, NoEdgeInReachRefuses)
{
    FakeWorld world;
    Trajectory trajectory = WalkOff(Walk({0.0f, 0.0f, 0.0f}), world, WALK_LIMIT);

    EXPECT_EQ(trajectory.outcome, Outcome::StepCap);
    EXPECT_TRUE(trajectory.segments.empty());
}

TEST(JumpTrajectoryWalkOff, WallBeforeTheEdgeRefuses)
{
    FakeWorld world;
    world.platforms.push_back({-100.0f, 3.0f, 10.0f});
    world.wallX = 2.2f;
    Trajectory trajectory = WalkOff(Walk({0.0f, 0.0f, 10.0f}), world, WALK_LIMIT);

    EXPECT_EQ(trajectory.outcome, Outcome::StepCap);
    EXPECT_TRUE(trajectory.segments.empty());
}

TEST(JumpTrajectoryWalkOff, StandingStillRefuses)
{
    FakeWorld world;
    Launch launch = Walk({0.0f, 0.0f, 10.0f});
    launch.velocity.x = 0.0f;
    world.platforms.push_back({-100.0f, 3.0f, 10.0f});

    EXPECT_EQ(WalkOff(launch, world, WALK_LIMIT).outcome, Outcome::StepCap);
}
