/*
 * This file is part of the mod-playerbots module for AzerothCore. See AUTHORS file for Copyright
 * information; released under GNU GPL v2 license, redistribute/modify under version 2 of the License,
 * or (at your option) any later version.
 */

#include "JumpTrajectory.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace JumpTrajectory
{
namespace
{
constexpr float EPSILON = 0.0001f;
constexpr uint32 CONTACT_ITERATIONS = 8;          // halvings that place a contact along a chord
constexpr float WALKABLE_NORMAL_Z = 0.64278764f;  // cos 50 degrees: steeper facets are the client's walls and ceilings
constexpr float SURFACE_PROBE_SPACING = 0.1f;     // between a ray and the rays beside it that find the surface it hit
constexpr float TERRAIN_GRADIENT_SPACING = 0.25f;
constexpr uint32 MAX_STALLS = 3;  // wall contacts in a row without progress before the body stops moving sideways

enum class Surface : uint8
{
    None,
    Wall,
    Ceiling
};

struct Contact
{
    Surface surface = Surface::None;
    float fraction = 1.0f;  // of the chord, CONTACT_GAP short of the surface
    Point normal;           // horizontal on a wall
};

struct Body
{
    float radius;
    float lowerEdge;  // the box's bottom above the feet
    float height;
};

Point operator+(Point const& a, Point const& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Point operator-(Point const& a, Point const& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Point operator*(Point const& a, float scale) { return {a.x * scale, a.y * scale, a.z * scale}; }
float Dot(Point const& a, Point const& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float Length(Point const& a) { return std::sqrt(Dot(a, a)); }
Point Flat(Point const& a) { return {a.x, a.y, 0.0f}; }

Point Cross(Point const& a, Point const& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Unit length, or zero when too short to have a direction.
Point Direction(Point const& a)
{
    float const length = Length(a);
    return length < EPSILON ? Point{} : a * (1.0f / length);
}

Point Lerp(Point const& from, Point const& to, float fraction) { return from + (to - from) * fraction; }

// Where along a chord `touches` starts to hold, given that it holds at the end: the last fraction clear of it and the
// first touching it.
template <typename Touches>
std::pair<float, float> FirstContact(Touches const& touches)
{
    float clear = 0.0f;
    float touching = 1.0f;
    for (uint32 i = 0; i < CONTACT_ITERATIONS; ++i)
    {
        float const middle = (clear + touching) * 0.5f;
        if (touches(middle))
            touching = middle;
        else
            clear = middle;
    }

    return {clear, touching};
}

float Seconds(uint32 ms) { return static_cast<float>(ms) / 1000.0f; }
uint32 Milliseconds(float seconds) { return static_cast<uint32>(seconds * 1000.0f + 0.5f); }

// The height below which the bot swims in `liquid`, NO_HEIGHT when it is too shallow to swim in. Shallower liquid
// lands the bot on its bed, as the client and core decide.
float SwimHeight(CollisionProbe const& probe, Point const& end, float liquid, float collisionHeight)
{
    float const swimDepth = SWIM_DEPTH * collisionHeight;
    float const bed = probe.FloorZ(end.x, end.y, liquid, swimDepth);
    return bed <= NO_HEIGHT || liquid - bed > swimDepth ? liquid - swimDepth : NO_HEIGHT;
}

// The surface a ray hit, from where two rays beside it hit; false at an edge, where one of them misses on both sides.
bool SurfaceNormal(CollisionProbe const& probe, Point const& from, Point const& to, Point const& hit, Point& normal)
{
    Point const direction = Direction(to - from);
    Point side = Direction(Cross(direction, {0.0f, 0.0f, 1.0f}));
    if (Length(side) < EPSILON)
        side = {1.0f, 0.0f, 0.0f};

    Point const offsets[2] = {side * SURFACE_PROBE_SPACING, Cross(side, direction) * SURFACE_PROBE_SPACING};
    Point beside[2];
    for (uint32 i = 0; i < 2; ++i)
        if (!probe.RayHit(from + offsets[i], to + offsets[i], beside[i]) &&
            !probe.RayHit(from - offsets[i], to - offsets[i], beside[i]))
            return false;

    normal = Direction(Cross(beside[0] - hit, beside[1] - hit));
    if (Dot(normal, direction) > 0.0f)
        normal = normal * -1.0f;

    return Length(normal) > EPSILON;
}

// A wall or a ceiling, as the client tells them apart by the facet's slope. A floor met by the body's edge, such as a
// ledge's top, stops it moving that way.
Contact Classify(Point const& normal, Point const& forward)
{
    Contact contact;
    if (normal.z < -WALKABLE_NORMAL_Z || Length(forward) < EPSILON)
    {
        contact.surface = Surface::Ceiling;
        contact.normal = normal;
        return contact;
    }

    contact.surface = Surface::Wall;
    contact.normal = normal.z > WALKABLE_NORMAL_Z ? forward * -1.0f : Direction(Flat(normal));
    return contact;
}

// Stops CONTACT_GAP short of the surface, measured along its normal: a steep fall comes that close to a wall too.
float GapFraction(Point const& chord, Contact const& contact, float rayFraction)
{
    float const approach = -Dot(chord, contact.normal);
    return approach > EPSILON ? std::max(0.0f, rayFraction - CONTACT_GAP / approach) : 0.0f;
}

// The first static or dynamic geometry the body's box meets along the chord: rays along its leading edges and through
// its front, the head's middle while rising, and its front half after it turns, where a wall can already be.
Contact RayContact(CollisionProbe const& probe, Body const& body, Point const& from, Point const& to, bool rising,
                   bool turned)
{
    Point const chord = to - from;
    Point const forward = Direction(Flat(chord));
    bool const horizontal = Length(forward) > EPSILON;

    Point offsets[6];
    uint32 count = 0;
    if (horizontal)
    {
        Point const front = forward * body.radius;
        Point const side = Point{-forward.y, forward.x, 0.0f} * body.radius;
        for (float const height : {body.lowerEdge, body.height})
        {
            offsets[count++] = front + side + Point{0.0f, 0.0f, height};
            offsets[count++] = front - side + Point{0.0f, 0.0f, height};
        }

        offsets[count++] = front + Point{0.0f, 0.0f, body.height * 0.5f};  // a post narrower than the body
    }

    if (rising)
        offsets[count++] = {0.0f, 0.0f, body.height};

    Point rayFrom;
    Point rayTo;
    Point hit;
    float rayFraction = 2.0f;
    if (horizontal && turned)
    {
        Point const middle = from + Point{0.0f, 0.0f, body.height * 0.5f};
        Point const front = middle + forward * body.radius;
        if (probe.RayHit(middle, front, hit))
        {
            rayFrom = middle;
            rayTo = front;
            rayFraction = 0.0f;
        }
    }

    float const length = Length(chord);
    for (uint32 i = 0; i < count && rayFraction > 0.0f; ++i)
    {
        Point const start = from + offsets[i];
        Point const end = to + offsets[i];
        Point candidate;
        if (!probe.RayHit(start, end, candidate))
            continue;

        float const fraction = length < EPSILON ? 0.0f : Length(candidate - start) / length;
        if (fraction < rayFraction)
        {
            rayFraction = fraction;
            rayFrom = start;
            rayTo = end;
            hit = candidate;
        }
    }

    if (rayFraction > 1.0f)
        return {};

    Point normal;
    if (!SurfaceNormal(probe, rayFrom, rayTo, hit, normal))
        normal = horizontal ? forward * -1.0f : Point{0.0f, 0.0f, -1.0f};

    Contact contact = Classify(normal, forward);
    contact.fraction = GapFraction(chord, contact, rayFraction);
    return contact;
}

// Terrain rising into the box's front, where it starts to: the heightmap is not in the rays.
Contact TerrainContact(CollisionProbe const& probe, Body const& body, Point const& from, Point const& to)
{
    Point const forward = Direction(Flat(to - from));
    auto frontAt = [&](float t) { return Lerp(from, to, t) + forward * body.radius; };
    auto intoTerrain = [&](float t)
    {
        Point const front = frontAt(t);
        float const terrain = probe.TerrainZ(front.x, front.y);
        return terrain > NO_HEIGHT && terrain > std::max(from.z, front.z) + body.lowerEdge;
    };
    if (!intoTerrain(1.0f))
        return {};

    auto const [clear, touching] = FirstContact(intoTerrain);
    Point const front = frontAt(touching);
    float const spacing = TERRAIN_GRADIENT_SPACING;
    float const east = probe.TerrainZ(front.x + spacing, front.y);
    float const west = probe.TerrainZ(front.x - spacing, front.y);
    float const north = probe.TerrainZ(front.x, front.y + spacing);
    float const south = probe.TerrainZ(front.x, front.y - spacing);
    Point normal = forward * -1.0f;
    if (std::min({east, west, north, south}) > NO_HEIGHT)
        normal = Direction({(west - east) / (2.0f * spacing), (south - north) / (2.0f * spacing), 1.0f});

    Contact contact = Classify(normal, forward);
    contact.fraction = clear;
    return contact;
}
}  // namespace

Trajectory Sweep(Launch const& launch, CollisionProbe const& probe)
{
    Trajectory trajectory;
    float const halfHeight = launch.collisionHeight * 0.5f;
    float const terminal = launch.terminalVelocity;
    Body const body{launch.collisionRadius, launch.collisionRadius * STEP_HEIGHT_PER_RADIUS, launch.collisionHeight};

    Point position = launch.position;
    float vx = launch.velocity.x;
    float vy = launch.velocity.y;
    float vz = std::max(launch.velocity.z, -terminal);

    // A takeoff a hair below the floor it stands on would land at once.
    float const takeoffFloor =
        probe.FloorZ(position.x, position.y, position.z + halfHeight, halfHeight + FLOOR_SEARCH_MARGIN);
    float const standZ = takeoffFloor > NO_HEIGHT ? takeoffFloor + launch.landingOffset : NO_HEIGHT;
    if (standZ > position.z && standZ - position.z <= TAKEOFF_SNAP)
        position.z = standZ;

    trajectory.takeoff = position;

    // Not rising off the floor it stands on: it lands where it is.
    if (takeoffFloor > NO_HEIGHT && vz <= 0.0f && position.z <= standZ)
    {
        trajectory.outcome = Outcome::Landed;
        trajectory.landing = {position.x, position.y, standZ};
        return trajectory;
    }

    // Under the heightmap (caves, building interiors) the terrain is not a wall.
    float const takeoffTerrain = probe.TerrainZ(position.x, position.y);
    bool const underTerrain = takeoffTerrain > NO_HEIGHT && position.z < takeoffTerrain - TAKEOFF_SNAP;

    Point segmentStart = position;
    float segmentStartVz = vz;
    float segmentTime = 0.0f;
    SegmentKind segmentKind = SegmentKind::Parabolic;

    auto openSegment = [&]()
    {
        segmentStart = position;
        segmentStartVz = vz;
        segmentTime = 0.0f;
        segmentKind = vz <= -terminal + EPSILON ? SegmentKind::Linear : SegmentKind::Parabolic;
    };
    auto closeSegment = [&]()
    {
        uint32 const durationMs = Milliseconds(segmentTime);
        if (durationMs > 0)
            trajectory.segments.push_back({segmentStart, position, durationMs, segmentStartVz, segmentKind});
    };
    openSegment();

    // The floor under a point of the chord, searched from the body's middle at the chord's higher end.
    auto floorAt = [&](Point const& point)
    {
        float const searchFrom = std::max(position.z, point.z) + halfHeight;
        return probe.FloorZ(point.x, point.y, searchFrom, searchFrom - point.z + FLOOR_SEARCH_MARGIN);
    };
    auto onFloor = [&](Point const& point)
    {
        float const floor = floorAt(point);
        return floor > NO_HEIGHT && point.z <= floor + launch.landingOffset;
    };

    // The first thing the body meets on its way to `end`.
    auto contactAlong = [&](Point const& end, bool rising, bool turned)
    {
        Contact contact = RayContact(probe, body, position, end, rising, turned);
        if (underTerrain || Length(Flat(end - position)) < EPSILON)
            return contact;

        Contact const terrain = TerrainContact(probe, body, position, end);
        if (terrain.surface != Surface::None &&
            (contact.surface == Surface::None || terrain.fraction < contact.fraction))
            contact = terrain;

        return contact;
    };

    float const heldSpeedSquared = launch.velocity.x * launch.velocity.x + launch.velocity.y * launch.velocity.y;
    bool keyHeld = launch.moveKeyHeld && heldSpeedSquared > EPSILON;
    bool turned = true;  // the body's front is new: at takeoff, and after a wall turns it
    uint32 stalls = 0;
    for (uint32 step = 0; step < MAX_STEPS; ++step)
    {
        bool const capped = vz <= -terminal + EPSILON;
        auto arc = [&](float t, float speedX, float speedY)
        {
            return Point{position.x + speedX * t, position.y + speedY * t,
                         capped ? position.z - terminal * t : position.z + vz * t - 0.5f * GRAVITY * t * t};
        };

        // The client gives a held key its speed and heading back once the wall no longer blocks them.
        if (keyHeld && (vx != launch.velocity.x || vy != launch.velocity.y) &&
            contactAlong(arc(Seconds(STEP_MS), launch.velocity.x, launch.velocity.y), false, true).surface !=
                Surface::Wall)
        {
            closeSegment();
            vx = launch.velocity.x;
            vy = launch.velocity.y;
            openSegment();
            turned = true;
            stalls = 0;
        }

        bool const horizontal = vx * vx + vy * vy > EPSILON;
        float dt = Seconds(capped ? CAPPED_STEP_MS : STEP_MS);
        bool reachesTerminal = false;
        bool reachesApex = false;
        if (!capped)
        {
            float const toTerminal = (vz + terminal) / GRAVITY;
            if (toTerminal < dt)
            {
                dt = toTerminal;
                reachesTerminal = true;
            }

            // Straight up and back down is a spline with no length; split it at the apex.
            if (!horizontal && vz > 0.0f && vz / GRAVITY < dt)
            {
                dt = vz / GRAVITY;
                reachesApex = true;
                reachesTerminal = false;
            }
        }

        auto arcAt = [&](float t) { return arc(t, vx, vy); };
        Point const end = arcAt(dt);

        Contact contact = horizontal || vz > 0.0f ? contactAlong(end, vz > 0.0f, turned) : Contact{};
        turned = false;
        float chordTime = dt * contact.fraction;
        Point chordEnd = contact.surface == Surface::None ? end : arcAt(chordTime);

        auto finish = [&](Point const& landing, float time, Outcome outcome)
        {
            position = landing;
            segmentTime += time;
            closeSegment();
            trajectory.outcome = outcome;
            trajectory.landing = landing;
            return trajectory;
        };

        // Lands where the chord sinks to a level surface, or at its end when it rises onto it.
        auto land = [&](float surface, Outcome outcome)
        {
            float const startClearance = position.z - surface;
            float crossing = vz > 0.0f ? 1.0f : 0.0f;
            if (startClearance > 0.0f)
                crossing = startClearance / (startClearance - (chordEnd.z - surface));

            Point landing = Lerp(position, chordEnd, crossing);
            landing.z = surface;
            return finish(landing, chordTime * crossing, outcome);
        };

        float const floor = floorAt(chordEnd);
        bool const reachesFloor = floor > NO_HEIGHT && chordEnd.z <= floor + launch.landingOffset;

        // A long chord can pass through liquid and end below its bed, so ask at the floor: the core finds no liquid
        // below the bed.
        float const chordEndVz = capped ? -terminal : vz - GRAVITY * chordTime;
        float const liquid =
            chordEndVz < 0.0f ? probe.LiquidZ(chordEnd.x, chordEnd.y, reachesFloor ? floor : chordEnd.z) : NO_HEIGHT;
        if (liquid > NO_HEIGHT)
        {
            // A water walker comes down on the surface as on a floor.
            float const surface = liquid + launch.landingOffset;
            if (launch.waterWalking && chordEnd.z <= surface && position.z >= surface - TAKEOFF_SNAP)
                return land(surface, Outcome::Landed);

            // Past the apex a client swims, and stops falling, once it sinks to swim depth.
            float const swimZ = SwimHeight(probe, chordEnd, liquid, launch.collisionHeight);
            if (swimZ > NO_HEIGHT && chordEnd.z <= swimZ)
            {
                if (position.z > swimZ)
                    return land(swimZ, Outcome::Liquid);

                // Launched under the surface: it swims on from the apex.
                float const apexTime = capped || vz <= 0.0f ? 0.0f : vz / GRAVITY;
                return finish(arcAt(apexTime), apexTime, Outcome::Liquid);
            }
        }

        if (reachesFloor)
        {
            auto const [clear, touching] = FirstContact([&](float t) { return onFloor(arcAt(chordTime * t)); });
            Point landing = arcAt(chordTime * touching);
            float const landingFloor = floorAt(landing) + launch.landingOffset;
            if (!horizontal || landingFloor - landing.z <= body.lowerEdge)
            {
                landing.z = landingFloor;
                return finish(landing, chordTime * touching, Outcome::Landed);
            }

            // A ledge above the step height meets the body's side, as a wall. No ray sees that side, so nothing would
            // tell a held key when the ledge ends.
            chordTime *= clear;
            chordEnd = arcAt(chordTime);
            contact.surface = Surface::Wall;
            contact.normal = Direction({-vx, -vy, 0.0f});
            keyHeld = false;
        }
        else if (chordEnd.z < launch.minZ)
        {
            trajectory.outcome = Outcome::DropCap;
            return trajectory;
        }
        else
        {
            // Ends where the lowest point was sampled: it can slope steeply between there and the crossing.
            if (chordEnd.z < probe.MinZ(chordEnd.x, chordEnd.y) - VOID_DEPTH)
                return finish(chordEnd, chordTime, Outcome::Void);
        }

        position = chordEnd;
        segmentTime += chordTime;
        vz = capped ? -terminal : vz - GRAVITY * chordTime;

        if (contact.surface != Surface::None || reachesTerminal || reachesApex)
        {
            closeSegment();
            if (contact.surface == Surface::Wall)
            {
                // The client keeps the motion along the wall and drops the part into it.
                float const into = vx * contact.normal.x + vy * contact.normal.y;
                if (into < 0.0f)
                {
                    vx -= into * contact.normal.x;
                    vy -= into * contact.normal.y;
                }

                stalls = chordTime > 0.0f ? 0 : stalls + 1;
                if (stalls >= MAX_STALLS || vx * vx + vy * vy <= EPSILON)
                {
                    vx = 0.0f;
                    vy = 0.0f;
                }

                turned = true;
            }
            else if (contact.surface == Surface::Ceiling)
                vz = std::min(vz, 0.0f);
            else
                vz = reachesTerminal ? -terminal : 0.0f;

            openSegment();
        }
    }

    trajectory.outcome = Outcome::StepCap;
    return trajectory;
}

Trajectory WalkOff(Launch const& launch, CollisionProbe const& probe, float maxWalk)
{
    float const speed = std::hypot(launch.velocity.x, launch.velocity.y);
    if (speed < EPSILON)
        return {};

    float const halfHeight = launch.collisionHeight * 0.5f;
    float const stepX = launch.velocity.x / speed * WALK_STEP;
    float const stepY = launch.velocity.y / speed * WALK_STEP;
    Point position = launch.position;
    uint32 const steps = static_cast<uint32>(maxWalk / WALK_STEP);
    for (uint32 step = 1; step <= steps; ++step)
    {
        Point const next{position.x + stepX, position.y + stepY, position.z};
        Point hit;
        if (probe.RayHit({position.x, position.y, position.z + halfHeight}, {next.x, next.y, next.z + halfHeight}, hit))
            return {};

        // Searched as the sweep searches at takeoff, so the floor the walk steps off is one the fall leaves behind.
        float const floor = probe.FloorZ(next.x, next.y, next.z + halfHeight, halfHeight + FLOOR_SEARCH_MARGIN);
        if (floor > NO_HEIGHT && floor + launch.landingOffset >= next.z - FLOOR_SEARCH_MARGIN)
        {
            position = {next.x, next.y, floor + launch.landingOffset};
            continue;
        }

        // The floor drops away: the walk carries the bot this one step past the edge, and it falls from there.
        Launch fall = launch;
        fall.position = next;
        fall.velocity.z = 0.0f;
        Trajectory trajectory = Sweep(fall, probe);
        float const walked = static_cast<float>(step) * WALK_STEP;
        trajectory.segments.insert(trajectory.segments.begin(),
                                   {launch.position, next, Milliseconds(walked / speed), 0.0f, SegmentKind::Linear});
        return trajectory;
    }

    return {};
}

float ParabolicAmplitude(uint32 durationMs)
{
    float const duration = Seconds(durationMs);
    return GRAVITY * duration * duration / 8.0f;
}

float SegmentVelocity(Segment const& segment)
{
    if (!segment.durationMs)
        return 0.0f;

    float const dx = segment.end.x - segment.start.x;
    float const dy = segment.end.y - segment.start.y;
    float const dz = segment.end.z - segment.start.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) / Seconds(segment.durationMs);
}

Point PositionAt(Segment const& segment, uint32 elapsedMs)
{
    if (!segment.durationMs)
        return segment.end;

    elapsedMs = std::min(elapsedMs, segment.durationMs);
    Point position =
        Lerp(segment.start, segment.end, static_cast<float>(elapsedMs) / static_cast<float>(segment.durationMs));
    if (segment.kind == SegmentKind::Parabolic)
    {
        float const elapsed = Seconds(elapsedMs);
        position.z += 0.5f * GRAVITY * elapsed * (Seconds(segment.durationMs) - elapsed);
    }

    return position;
}

float VerticalSpeedAt(Segment const& segment, uint32 elapsedMs)
{
    if (!segment.durationMs)
        return segment.startVz;

    float const duration = Seconds(segment.durationMs);
    float const elapsed = Seconds(std::min(elapsedMs, segment.durationMs));
    float speed = (segment.end.z - segment.start.z) / duration;
    if (segment.kind == SegmentKind::Parabolic)
        speed += 0.5f * GRAVITY * (duration - 2.0f * elapsed);

    return speed;
}

Segment Remainder(Segment const& segment, uint32 elapsedMs)
{
    elapsedMs = std::min(elapsedMs, segment.durationMs);
    return {PositionAt(segment, elapsedMs), segment.end, segment.durationMs - elapsedMs,
            VerticalSpeedAt(segment, elapsedMs), segment.kind};
}
}  // namespace JumpTrajectory
