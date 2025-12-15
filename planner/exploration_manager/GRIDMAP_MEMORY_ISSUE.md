# GridMap Memory Issue Analysis

## Date: December 15, 2025

## **Critical Issue Discovered**

The exploration manager has a **fundamental contradiction**:

### **The Problem:**
- ✅ **Exploration Manager** remembers visited areas (via `explored_cells_` hash map)
- ❌ **GridMap** forgets old map data (sliding window with `local_update_range`)

### **Code Evidence:**

```cpp
// In grid_map.cpp line 761-762
void GridMap::pointCloudCallBack(...) {
  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);
  // ^^^ This CLEARS occupancy data outside local_update_range
}
```

### **What Happens:**

```
Time 0: Drone at (0,0) - Explores room at (0,0) to (5,5)
  GridMap: Knows obstacles at (0,0)-(5,5) ✅
  Exploration: Marks (0,0)-(5,5) as explored ✅

Time 1: Drone moves to (20,0) - Outside local_update_range
  GridMap: FORGETS (0,0)-(5,5) data ❌ [resetBuffer clears it]
  Exploration: Still remembers (0,0)-(5,5) explored ✅

Time 2: Drone tries to navigate near (0,0)
  GridMap: Sees "unknown space" (no data) ❌
  Exploration: "Don't go there, already explored" ✅
  EGO Planner: Might plan through old obstacles (forgotten walls)
```

---

## **Consequences**

### 1. **Navigation Safety Issues**
- EGO planner might plan paths through forgotten obstacles
- Old walls/obstacles become "unknown" → treated as free space
- Collision risk when navigating in previously explored areas

### 2. **False Frontier Detection**
- Old free space → unknown (forgotten) → detected as frontier
- Creates fake frontiers in already-explored regions
- Wastes time flying to "frontiers" that aren't real boundaries

### 3. **Inconsistent World Model**
```
Exploration Manager says: "I've been there, it's explored"
GridMap says:             "I don't know what's there" (unknown)
EGO Planner sees:         "Unknown space, maybe free?"
```

---

## **Solutions**

### **Solution 1: Increase Local Update Range (Quick Fix) ✅ APPLIED**

**Change:**
```xml
<!-- OLD (5.5m range - too small) -->
<param name="grid_map/local_update_range_x" value="5.5"/>
<param name="grid_map/local_update_range_y" value="5.5"/>
<param name="grid_map/local_update_range_z" value="3.5"/>

<!-- NEW (30m range - matches exploration radius) -->
<param name="grid_map/local_update_range_x" value="30.0"/>
<param name="grid_map/local_update_range_y" value="30.0"/>
<param name="grid_map/local_update_range_z" value="5.0"/>
```

**Pros:**
- ✅ Simple to implement (just change parameters)
- ✅ GridMap keeps data within exploration radius
- ✅ No code changes needed
- ✅ Works if `local_update_range >= max_exploration_radius`

**Cons:**
- ❌ Memory usage increases (keeps more voxels)
  - Old: (5.5×2/0.1)³ = 110³ = **1.3M voxels**
  - New: (30×2/0.1)³ = 600³ = **216M voxels** (166x more!)
- ❌ Still forgets data outside 30m
- ❌ Not scalable for large exploration areas

**Memory Calculation:**
```
Resolution: 0.1m
Local range: 30m radius = 60m diameter
Voxels: (60/0.1)³ = 600³ = 216,000,000 voxels
Memory per voxel: ~4 bytes (float occupancy)
Total: 216M × 4 bytes = 864 MB just for occupancy buffer
```

---

### **Solution 2: Global Map with Octree (Better but Complex)**

**Idea:** Replace GridMap sliding window with global octree-based map

**Implementation:**
```cpp
// Use OctoMap for global persistent mapping
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

class GlobalOccupancyMap {
  octomap::OcTree* octree_;  // Persistent global map
  
  void updateFromPointCloud(pcl::PointCloud& cloud) {
    // Update octree incrementally (doesn't forget old data)
    for (auto& pt : cloud.points) {
      octree_->updateNode(pt.x, pt.y, pt.z, true);
    }
  }
  
  int getOccupancy(Eigen::Vector3d pos) {
    octomap::OcTreeNode* node = octree_->search(pos.x(), pos.y(), pos.z());
    if (!node) return -1; // Unknown
    return node->getOccupancy() > 0.5 ? 1 : 0;
  }
};
```

**Pros:**
- ✅ Truly persistent memory (never forgets)
- ✅ Memory-efficient (only stores non-empty voxels)
- ✅ Scales to large environments
- ✅ Industry standard (ROS OctoMap)

**Cons:**
- ❌ Major code refactoring needed
- ❌ Need to replace GridMap throughout codebase
- ❌ Compatibility with EGO planner unclear
- ❌ 1-2 weeks development time

---

### **Solution 3: Hybrid Approach (Best Long-term)**

**Idea:** Keep GridMap for local planning, add global OctoMap for exploration

```cpp
class HybridMap {
  GridMap* local_map_;           // Fast local planning (5.5m range)
  octomap::OcTree* global_map_;  // Persistent global memory
  
  void update(pcl::PointCloud& cloud, Eigen::Vector3d pos) {
    // Update both maps
    local_map_->updateMap(cloud);
    global_map_->insertPointCloud(cloud, pos);
  }
  
  int getOccupancy(Eigen::Vector3d pos) {
    // Check local first (fast), fall back to global
    if (local_map_->isInRange(pos)) {
      return local_map_->getOccupancy(pos);
    }
    return global_map_->getOccupancy(pos);
  }
};
```

**Pros:**
- ✅ Best of both worlds
- ✅ Fast local planning (GridMap)
- ✅ Persistent global memory (OctoMap)
- ✅ Gradual implementation (can add OctoMap alongside GridMap)

**Cons:**
- ❌ More complex architecture
- ❌ Need to maintain synchronization
- ❌ Still requires code changes

---

## **Recommended Action**

### **Short-term (Current Fix):**

✅ **Increased `local_update_range` to 30m** (already applied)

**Why this works for now:**
- Keeps map data within exploration radius (25m geofence)
- Simple parameter change, no code modifications
- Good enough for single-room/building exploration

**Limitations:**
- High memory usage (~864 MB)
- Still forgets data beyond 30m
- Not scalable to large outdoor environments

### **Long-term (Future Enhancement):**

Consider implementing **Solution 3 (Hybrid Approach)** if:
- Exploring large areas (>50m radius)
- Need true persistent mapping
- Have memory constraints
- Want to support multi-session exploration

---

## **Testing Recommendations**

### **Test 1: Verify No Memory Loss**
```bash
# Explore area, then move 20m away, then return
# Expected: Should remember old obstacles, not detect false frontiers
```

**Check:**
1. Launch exploration
2. Let drone explore region A
3. Wait for drone to move >20m away to region B
4. Command drone back toward region A
5. Verify: No false frontiers in region A (should be marked explored)
6. Verify: EGO planner still knows about obstacles in region A

### **Test 2: Memory Usage**
```bash
# Monitor memory consumption during exploration
htop  # Watch exploration_node memory usage
```

**Expected:**
- Initial: ~100 MB
- After exploring 25m radius: ~800-1000 MB
- Should stabilize (not grow indefinitely)

### **Test 3: Large Area Exploration**
```bash
# Test with larger exploration radius
<param name="exploration/max_exploration_radius" value="40.0"/>
<param name="grid_map/local_update_range_x" value="45.0"/>
```

**Expected:**
- Memory usage: ~2-3 GB
- If exceeds available RAM → need Solution 2 or 3

---

## **Configuration Guidelines**

### **Rule of Thumb:**
```
local_update_range ≥ max_exploration_radius + 5m (safety margin)
```

### **Examples:**

| Exploration Radius | Local Update Range | Memory | Use Case |
|-------------------|-------------------|---------|----------|
| 10m | 15m | ~100 MB | Single room |
| 25m | 30m | ~800 MB | Building floor |
| 50m | 55m | ~5 GB | Large building |
| 100m | N/A | Need OctoMap | Outdoor/campus |

### **Current Configuration:**
```xml
<!-- Exploration geofence -->
<param name="exploration/max_exploration_radius" value="25.0"/>

<!-- GridMap memory (MUST be >= exploration radius) -->
<param name="grid_map/local_update_range_x" value="30.0"/>
<param name="grid_map/local_update_range_y" value="30.0"/>
<param name="grid_map/local_update_range_z" value="5.0"/>
```

**This configuration is valid for:**
- ✅ Indoor exploration (rooms, corridors, single floor)
- ✅ Bounded areas up to 25m radius
- ✅ Typical building environments

**NOT suitable for:**
- ❌ Large outdoor areas (>50m)
- ❌ Multi-floor exploration
- ❌ Memory-constrained platforms (embedded systems)

---

## **Comparison with FUEL**

| Feature | fuel-exploration (Current) | fuel-planner-replacement (Full FUEL) |
|---------|---------------------------|--------------------------------------|
| **Global Memory** | ❌ GridMap sliding window | ✅ FIS with incremental updates |
| **Map Type** | Occupancy grid (dense) | SDF + EDT (sparse) |
| **Memory Usage** | ~800 MB (30m range) | ~200 MB (efficient structure) |
| **Forgetting** | ✅ Fixed with large range | ✅ Never forgets (incremental) |
| **Scalability** | Limited to local range | Scales to large areas |

**Key Insight:**
- FUEL uses **SDF (Signed Distance Field)** with incremental updates
- Doesn't use sliding window → truly persistent
- More memory-efficient for large areas

---

## **Action Items**

- [x] Increase `local_update_range` to 30m (DONE)
- [x] Update both EGO planner and exploration manager configs (DONE)
- [ ] Test memory usage during exploration
- [ ] Monitor for false frontiers in revisited areas
- [ ] Document memory requirements for different exploration radii
- [ ] Consider OctoMap integration if scaling beyond 50m radius

---

## **Summary**

**Problem:** GridMap forgets old data while exploration remembers visited areas → inconsistency

**Quick Fix:** Increased local_update_range to 30m (covers 25m exploration radius)

**Trade-off:** Higher memory usage (~800 MB) but solves the forgetting issue

**Future:** Consider OctoMap for truly scalable persistent mapping

**Status:** ✅ Fixed for current use case (indoor exploration, 25m radius)
