// Wire protocol shared with the MuJoCo simulator.
// Mirrors mujoco_sim/star_nav_mujoco/protocol.py; keep both in sync.
#pragma once

#include <cstdint>

namespace mujoco_bridge {

constexpr uint16_t kProtocolVersion = 1;

constexpr char kTopicState[] = "state";
constexpr char kTopicDepth[] = "depth";
constexpr char kTopicLidar[] = "lidar";
constexpr char kTopicWorld[] = "world";
constexpr char kTopicCmd[] = "cmd";

#pragma pack(push, 1)

struct StateMsg {
  uint16_t version;
  uint16_t pad0;
  double t;
  double pos[3];
  double quat[4];   // w, x, y, z
  double vel[3];    // world frame
  double omega[3];  // body frame
  uint32_t contacts;
  uint32_t pad1;
};

// Followed by width * height uint16 depth values in millimetres.
struct DepthHeader {
  uint16_t version;
  uint16_t width;
  uint16_t height;
  uint16_t pad0;
  double t;
  double pos[3];   // optical frame in world
  double quat[4];  // w, x, y, z
  float fx, fy, cx, cy;
};

// Followed by count * 3 float32 x, y, z in the sensor frame (x forward, z up).
struct LidarHeader {
  uint16_t version;
  uint16_t beams;
  uint32_t count;
  double t;
  double pos[3];   // sensor origin in world
  double quat[4];  // w, x, y, z
};

// Followed by `count` ObstacleMsg records.
struct WorldHeader {
  uint16_t version;
  uint16_t pad0;
  uint32_t count;
};

enum ObstacleKind : uint8_t { kPillar = 0, kRing = 1 };

struct ObstacleMsg {
  uint8_t kind;
  uint8_t pad[3];
  float x, y, z;
  float a, b;  // pillar: radius, height; ring: semi-axes
  float yaw;
};

struct CmdMsg {
  uint16_t version;
  uint16_t pad0;
  uint32_t pad1;
  double t;
  double pos[3];
  double vel[3];
  double acc[3];
  double yaw;
  double yaw_rate;
};

#pragma pack(pop)

static_assert(sizeof(StateMsg) == 124, "StateMsg layout");
static_assert(sizeof(DepthHeader) == 88, "DepthHeader layout");
static_assert(sizeof(LidarHeader) == 72, "LidarHeader layout");
static_assert(sizeof(WorldHeader) == 8, "WorldHeader layout");
static_assert(sizeof(ObstacleMsg) == 28, "ObstacleMsg layout");
static_assert(sizeof(CmdMsg) == 104, "CmdMsg layout");

}  // namespace mujoco_bridge
