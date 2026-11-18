#include "plan_env/grid_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);
  node_.param("grid_map/enable_pointcloud_raycasting", mp_.enable_pointcloud_raycasting_, false);
  node_.param("grid_map/max_hits_per_update", mp_.max_hits_per_update_, 1);
  mp_.max_hits_per_update_ = max(1, mp_.max_hits_per_update_);
  std::vector<double> lidar_offset;
  node_.param("grid_map/lidar_offset", lidar_offset, std::vector<double>(3, 0.0));
  if (lidar_offset.size() != 3)
  {
    ROS_ERROR("grid_map/lidar_offset needs 3 values, ignoring it");
    lidar_offset.assign(3, 0.0);
  }
  mp_.lidar_offset_ = Eigen::Vector3d(lidar_offset[0], lidar_offset[1], lidar_offset[2]);

  node_.param("grid_map/obstacle_min_height", mp_.obstacle_min_height_, -1.0e9);
  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("grid_map/visualization_min_height", mp_.visualization_min_height_, -1.0e9);
  node_.param("grid_map/slice_min_z", mp_.slice_min_z_, 0.0);
  node_.param("grid_map/slice_max_z", mp_.slice_max_z_, 0.0);
  node_.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node_.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  double sliding_x, sliding_y;
  node_.param("grid_map/sliding_map", mp_.sliding_map_, false);
  node_.param("grid_map/sliding_size_x", sliding_x, x_size);
  node_.param("grid_map/sliding_size_y", sliding_y, y_size);
  node_.param("grid_map/sliding_threshold", mp_.sliding_threshold_, 1.0);

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // Stored voxels: the whole map, or with sliding_map a window (x, y) that
  // follows the vehicle; z always covers the map height.
  Eigen::Vector3d stored_size = mp_.map_size_;
  if (mp_.sliding_map_)
    stored_size.head<2>() << sliding_x, sliding_y;
  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(stored_size(i) / mp_.resolution_);

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  // The window starts at the map origin, or centred on (0, 0) when sliding.
  Eigen::Vector3i window_min = Eigen::Vector3i::Zero();
  if (mp_.sliding_map_)
  {
    posToIndex(Eigen::Vector3d(0.0, 0.0, mp_.map_origin_(2)), window_min);
    window_min.head<2>() -= mp_.map_voxel_num_.head<2>() / 2;
    window_min(2) = 0;
    ROS_INFO("grid_map: sliding map %.1f x %.1f m, follows the vehicle every %.1f m", sliding_x, sliding_y,
             mp_.sliding_threshold_);
  }
  setWindow(window_min);

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_fusion/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);

  // Latched publishers so RViz can connect later and still see the last map.
  // Also helps prevent the visualization from flickering when updates are intermittent.
  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10, true);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10, true);
  depth_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/depth_cloud", 10, true);
  if (mp_.slice_max_z_ > mp_.slice_min_z_)
  {
    slice_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("grid_map/slice", 2);
    slice_timer_ = node_.createTimer(ros::Duration(0.2), &GridMap::sliceCallback, this);
  }

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.timed_updates_ = 0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = md_.window_min_;
  md_.local_bound_max_ = md_.window_min_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  const int idx_ctns = addressIfInMap(id);
  if (idx_ctns < 0)
    return INVALID_IDX;

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v+=skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u+=skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);

  // A ray normally stops at the first voxel another ray has already cleared in
  // this update. Rays only share that voxel, not the rest of the path, so the
  // voxels behind it are skipped. Depth pixels are dense enough to cover them,
  // but lidar beams are ~1.5 deg apart (a voxel at 4 m): with a steady vehicle
  // the same voxels are skipped every scan and an obstacle that moved away is
  // never cleared. Point clouds therefore trace every ray to the vehicle.
  const bool full_rays = !md_.flag_use_depth_fusion;
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        // A ground robot sees the floor it walks on: such returns clear the
        // ray but are not obstacles.
        vox_idx = setCacheOccupancy(pt_w, pt_w(2) < mp_.obstacle_min_height_ ? 0 : 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    // Trace in map grid coordinates: the map origin (ground_height in z) is
    // not a multiple of the resolution, and world-aligned cells would put the
    // ray in a neighbouring voxel near cell boundaries.
    raycaster.setInput((pt_w - mp_.map_origin_) / mp_.resolution_,
                       (md_.camera_pos_ - mp_.map_origin_) / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_ + mp_.map_origin_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          if (!full_rays)
            break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    // One update per voxel and scan: a hit if at least as many rays ended in
    // it as passed through it. A surface hit by k rays of the same scan gets
    // k hits (at most max_hits_per_update), as in ROG-Map, so a dense lidar
    // confirms an obstacle in one or two scans while a single stray return
    // still counts once.
    const int hits = md_.count_hit_[idx_ctns];
    const int misses = md_.count_hit_and_miss_[idx_ctns] - hits;
    double log_odds_update =
        hits >= misses ? mp_.prob_hit_log_ * min(hits, mp_.max_hits_per_update_) : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    const double current = md_.occupancy_buffer_[idx_ctns];
    if (log_odds_update >= 0 && current >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && current <= mp_.clamp_min_log_)
    {
      setLogOdds(idx_ctns, idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    const double base = in_local ? current : mp_.clamp_min_log_;

    setLogOdds(idx_ctns, idx,
               std::min(std::max(base + log_odds_update, mp_.clamp_min_log_), mp_.clamp_max_log_));
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  // Forget a shell of voxels just outside the updated region (upstream
  // behaviour). The inflated map no longer needs rebuilding here: setLogOdds
  // keeps it in step with every occupancy change.
  const int vec_margin = 5;

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  const double unknown = mp_.clamp_min_log_ - mp_.unknown_flag_;
  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
      for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
      {
        if (x >= min_cut(0) && x <= max_cut(0) && y >= min_cut(1) && y <= max_cut(1) && z >= min_cut(2) &&
            z <= max_cut(2))
        {
          z = max_cut(2);  // skip the inside of the box
          continue;
        }
        Eigen::Vector3i id(x, y, z);
        setLogOdds(toAddress(id), id, unknown);
      }
}

int GridMap::inflationStep() const { return ceil(mp_.obstacles_inflation_ / mp_.resolution_); }

void GridMap::changeInflation(const Eigen::Vector3i &id, int delta)
{
  const int step = inflationStep();
  Eigen::Vector3i nb;
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
      {
        nb = id + Eigen::Vector3i(x, y, z);
        if (!isInMap(nb))
          continue;
        uint16_t &count = md_.occupancy_buffer_inflate_[toAddress(nb)];
        if (delta > 0)
          ++count;
        else if (count > 0)
          --count;
      }
}

void GridMap::rebuildInflation(const Eigen::Vector3i &min_id, const Eigen::Vector3i &max_id)
{
  // Recomputes the counters inside [min_id, max_id] from the occupied voxels
  // around it.
  Eigen::Vector3i lo = min_id, hi = max_id;
  boundIndex(lo);
  boundIndex(hi);
  for (int x = lo(0); x <= hi(0); ++x)
    for (int y = lo(1); y <= hi(1); ++y)
      for (int z = lo(2); z <= hi(2); ++z)
        md_.occupancy_buffer_inflate_[toAddress(Eigen::Vector3i(x, y, z))] = 0;

  const int step = inflationStep();
  Eigen::Vector3i src_lo = lo - Eigen::Vector3i::Constant(step), src_hi = hi + Eigen::Vector3i::Constant(step);
  boundIndex(src_lo);
  boundIndex(src_hi);
  for (int x = src_lo(0); x <= src_hi(0); ++x)
    for (int y = src_lo(1); y <= src_hi(1); ++y)
      for (int z = src_lo(2); z <= src_hi(2); ++z)
      {
        const Eigen::Vector3i src(x, y, z);
        if (md_.occupancy_buffer_[toAddress(src)] <= mp_.min_occupancy_log_)
          continue;
        for (int i = max(lo(0), x - step); i <= min(hi(0), x + step); ++i)
          for (int j = max(lo(1), y - step); j <= min(hi(1), y + step); ++j)
            for (int k = max(lo(2), z - step); k <= min(hi(2), z + step); ++k)
              ++md_.occupancy_buffer_inflate_[toAddress(Eigen::Vector3i(i, j, k))];
      }
}

void GridMap::setWindow(const Eigen::Vector3i &window_min)
{
  md_.window_min_ = window_min;
  const Eigen::Vector3i &n = mp_.map_voxel_num_;
  const int stride[3] = {n(1) * n(2), n(2), 1};
  std::vector<int> *ring[3] = {&md_.ring_x_, &md_.ring_y_, &md_.ring_z_};
  for (int i = 0; i < 3; ++i)
  {
    ring[i]->resize(n(i));
    for (int k = 0; k < n(i); ++k)
    {
      int r = (window_min(i) + k) % n(i);
      if (r < 0)
        r += n(i);
      (*ring[i])[k] = r * stride[i];
    }
  }
  mp_.map_min_boundary_ = mp_.map_origin_ + window_min.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = mp_.map_min_boundary_ + mp_.map_voxel_num_.cast<double>() * mp_.resolution_;
}

namespace
{
// The parts of box a = [a_min, a_max] outside box b, as up to three boxes.
std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>> boxDifference(Eigen::Vector3i a_min, Eigen::Vector3i a_max,
                                                                       const Eigen::Vector3i &b_min,
                                                                       const Eigen::Vector3i &b_max)
{
  std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>> boxes;
  for (int i = 0; i < 3; ++i)
  {
    if (a_min(i) < b_min(i))
    {
      Eigen::Vector3i hi = a_max;
      hi(i) = min(a_max(i), b_min(i) - 1);
      boxes.emplace_back(a_min, hi);
      a_min(i) = hi(i) + 1;
    }
    if (a_max(i) > b_max(i))
    {
      Eigen::Vector3i lo = a_min;
      lo(i) = max(a_min(i), b_max(i) + 1);
      boxes.emplace_back(lo, a_max);
      a_max(i) = lo(i) - 1;
    }
    if (a_min(i) > a_max(i))
      break;  // nothing of a left inside b
  }
  return boxes;
}
}  // namespace

void GridMap::slideWindow(const Eigen::Vector3d &center)
{
  if (!mp_.sliding_map_)
    return;

  Eigen::Vector3i center_id;
  posToIndex(center, center_id);
  Eigen::Vector3i target = md_.window_min_;
  target.head<2>() = center_id.head<2>() - mp_.map_voxel_num_.head<2>() / 2;
  const Eigen::Vector3i shift = target - md_.window_min_;
  if (abs(shift(0)) * mp_.resolution_ < mp_.sliding_threshold_ &&
      abs(shift(1)) * mp_.resolution_ < mp_.sliding_threshold_)
    return;

  const Eigen::Vector3i one = Eigen::Vector3i::Ones();
  const Eigen::Vector3i old_min = md_.window_min_, old_max = old_min + mp_.map_voxel_num_ - one;
  const Eigen::Vector3i new_min = target, new_max = target + mp_.map_voxel_num_ - one;
  const double unknown = mp_.clamp_min_log_ - mp_.unknown_flag_;

  // Voxels that leave: take back their inflation, then forget them. Their
  // memory is reused by the voxels that enter.
  for (const auto &box : boxDifference(old_min, old_max, new_min, new_max))
    for (int x = box.first(0); x <= box.second(0); ++x)
      for (int y = box.first(1); y <= box.second(1); ++y)
        for (int z = box.first(2); z <= box.second(2); ++z)
        {
          const Eigen::Vector3i id(x, y, z);
          const int adr = toAddress(id);
          if (md_.occupancy_buffer_[adr] > mp_.min_occupancy_log_)
            changeInflation(id, -1);
          md_.occupancy_buffer_[adr] = unknown;
          md_.count_hit_[adr] = md_.count_hit_and_miss_[adr] = 0;
        }

  setWindow(target);

  // Voxels that enter are unknown; their inflation comes only from occupied
  // voxels that were already stored.
  for (const auto &box : boxDifference(new_min, new_max, old_min, old_max))
    rebuildInflation(box.first, box.second);
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMapInflate(true);
  publishMap();
  publishDepth();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (md_.last_occ_update_time_.toSec() < 1.0 ) md_.last_occ_update_time_ = ros::Time::now();
  
  if (!md_.occ_need_update_)
  {
    if ( md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ )
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
        ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  const ros::WallTime t0 = ros::WallTime::now();
  slideWindow(md_.camera_pos_);

  // Depth-fusion path: project depth image into point set for raycasting.
  // Pointcloud-only path: cloudCallback already populates md_.proj_points_ and md_.proj_points_cnt.
  if (md_.flag_use_depth_fusion)
  {
    projectDepthImage();
  }
  raycastProcess();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  if (mp_.show_occ_time_)
  {
    const double t = (ros::WallTime::now() - t0).toSec();
    md_.fuse_time_ += t;
    md_.max_fuse_time_ = max(md_.max_fuse_time_, t);
    md_.timed_updates_ += 1;
    ROS_INFO_THROTTLE(5.0, "grid_map update: %.2f ms (mean %.2f, max %.2f over %d updates)", 1e3 * t,
                      1e3 * md_.fuse_time_ / md_.timed_updates_, 1e3 * md_.max_fuse_time_, md_.timed_updates_);
  }

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  slideWindow(md_.camera_pos_);
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  // Cast rays from the sensor, not the body centre: a few centimetres move
  // the rays to neighbouring voxels, which then never get cleared.
  const geometry_msgs::Quaternion &q = odom->pose.pose.orientation;
  md_.camera_pos_ += Eigen::Quaterniond(q.w, q.x, q.y, q.z) * mp_.lidar_offset_;

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  // Ray casting: the points go to the probabilistic map, which
  // updateOccupancyCallback() updates and inflates. Returns beyond
  // max_ray_length are kept too: raycastProcess() shortens them to a
  // free-only ray, so the space in front of a distant return is cleared, the
  // same way a far depth pixel clears it.
  if (mp_.enable_pointcloud_raycasting_)
  {
    md_.proj_points_cnt = 0;
    for (const auto &pt : latest_cloud.points)
    {
      const Eigen::Vector3d p3d(pt.x, pt.y, pt.z);
      if ((p3d - md_.camera_pos_).norm() <= mp_.min_ray_length_)
        continue;
      // The buffer is sized for a depth image; a lidar scan can be larger.
      if (md_.proj_points_cnt >= (int)md_.proj_points_.size())
        md_.proj_points_.resize(2 * md_.proj_points_.size());
      md_.proj_points_[md_.proj_points_cnt++] = p3d;
    }
    md_.occ_need_update_ = true;
    return;
  }

  // Otherwise (upstream): the latest cloud replaces the local map.
  slideWindow(md_.camera_pos_);
  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);
  
  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    /* point inside update range */
    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {
      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_ || pos(2) < mp_.visualization_min_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  // Avoid publishing empty clouds: RViz will otherwise show "no data" and appear to flicker.
  if (cloud.points.empty())
    return;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

// The voxels between slice_min_z and slice_max_z around the sensor (the local
// update range), flattened: occupied (100) if any of them is or if the voxel
// at mid-height is in the inflated map (where the planner cannot fly, also
// near obstacles just above or below the band), free (0) if at least half of
// them are observed, otherwise unknown (-1).
void GridMap::publishSlice()
{
  if (slice_pub_.getNumSubscribers() <= 0 || !md_.has_odom_)
    return;

  Eigen::Vector3i lo, hi;
  posToIndex(Eigen::Vector3d(md_.camera_pos_(0) - mp_.local_update_range_(0),
                             md_.camera_pos_(1) - mp_.local_update_range_(1), mp_.slice_min_z_), lo);
  posToIndex(Eigen::Vector3d(md_.camera_pos_(0) + mp_.local_update_range_(0),
                             md_.camera_pos_(1) + mp_.local_update_range_(1), mp_.slice_max_z_), hi);
  const Eigen::Vector3i window_max = md_.window_min_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  lo = lo.cwiseMax(md_.window_min_);
  hi = hi.cwiseMin(window_max);
  if ((hi.array() < lo.array()).any())
    return;

  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = mp_.frame_id_;
  msg.header.stamp = ros::Time::now();
  msg.info.resolution = mp_.resolution_;
  msg.info.width = hi(0) - lo(0) + 1;
  msg.info.height = hi(1) - lo(1) + 1;
  msg.info.origin.position.x = mp_.map_origin_(0) + lo(0) * mp_.resolution_;
  msg.info.origin.position.y = mp_.map_origin_(1) + lo(1) * mp_.resolution_;
  msg.info.origin.position.z = mp_.slice_min_z_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(msg.info.width * msg.info.height, -1);

  const int layers = hi(2) - lo(2) + 1;
  Eigen::Vector3i mid;
  posToIndex(Eigen::Vector3d(0.0, 0.0, 0.5 * (mp_.slice_min_z_ + mp_.slice_max_z_)), mid);
  const bool check_inflation = mid(2) >= lo(2) && mid(2) <= hi(2);
  for (int y = lo(1); y <= hi(1); ++y)
    for (int x = lo(0); x <= hi(0); ++x)
    {
      int known = 0;
      bool occupied = check_inflation && md_.occupancy_buffer_inflate_[toAddress(x, y, mid(2))] > 0;
      for (int z = lo(2); z <= hi(2) && !occupied; ++z)
      {
        const double log_odds = md_.occupancy_buffer_[toAddress(x, y, z)];
        if (log_odds < mp_.clamp_min_log_ - 1e-3)
          continue;
        ++known;
        occupied = log_odds > mp_.min_occupancy_log_;
      }
      int8_t &cell = msg.data[(y - lo(1)) * msg.info.width + (x - lo(0))];
      if (occupied)
        cell = 100;
      else if (2 * known >= layers)
        cell = 0;
    }
  slice_pub_.publish(msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_ || pos(2) < mp_.visualization_min_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  // Avoid publishing empty clouds: keep last valid visualization latched.
  if (cloud.points.empty())
    return;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation()
{
  // In pointcloud-only mode (exploration), treat cloud reception as a valid map observation.
  return md_.has_first_depth_ || (mp_.enable_pointcloud_raycasting_ && md_.has_cloud_);
}

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}

void GridMap::publishDepth()
{
  if (depth_cloud_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Debug: print camera transform
  static int debug_counter = 0;
  if (debug_counter++ % 100 == 0) {
    ROS_INFO("[GridMap] Camera position: [%.2f, %.2f, %.2f]", 
             md_.camera_pos_(0), md_.camera_pos_(1), md_.camera_pos_(2));
    ROS_INFO("[GridMap] Camera rotation:\n  [%.3f, %.3f, %.3f]\n  [%.3f, %.3f, %.3f]\n  [%.3f, %.3f, %.3f]",
             md_.camera_r_m_(0,0), md_.camera_r_m_(0,1), md_.camera_r_m_(0,2),
             md_.camera_r_m_(1,0), md_.camera_r_m_(1,1), md_.camera_r_m_(1,2),
             md_.camera_r_m_(2,0), md_.camera_r_m_(2,1), md_.camera_r_m_(2,2));
    ROS_INFO("[GridMap] cam2body:\n  [%.3f, %.3f, %.3f, %.3f]\n  [%.3f, %.3f, %.3f, %.3f]\n  [%.3f, %.3f, %.3f, %.3f]",
             md_.cam2body_(0,0), md_.cam2body_(0,1), md_.cam2body_(0,2), md_.cam2body_(0,3),
             md_.cam2body_(1,0), md_.cam2body_(1,1), md_.cam2body_(1,2), md_.cam2body_(1,3),
             md_.cam2body_(2,0), md_.cam2body_(2,1), md_.cam2body_(2,2), md_.cam2body_(2,3));
    ROS_INFO("[GridMap] Publishing depth cloud with %d projected points", md_.proj_points_cnt);
  }

  // Publish the projected points from depth image
  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    Eigen::Vector3d pos = md_.proj_points_[i];
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  if (!cloud.points.empty())
  {
    pcl::toROSMsg(cloud, cloud_msg);
    depth_cloud_pub_.publish(cloud_msg);
  }
}
