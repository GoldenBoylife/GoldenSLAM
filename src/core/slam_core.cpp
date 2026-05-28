// All heavy algorithm headers are ONLY included here, not in the .hpp
#include <omp.h>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "so3_math.h"
#include "common_lib.h"
#include "use-ikfom.hpp"
#include "IKFoM_toolkit/esekfom/esekfom.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "IMU_Processing.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "core/slam_core.hpp"

#define INIT_TIME        (0.1)
#define LASER_POINT_COV  (0.001)

// ─────────────────────────────────────────────────────────────────────────────
// SlamCoreImpl  –  all algorithm state lives here
// ─────────────────────────────────────────────────────────────────────────────
class SlamCoreImpl
{
public:
    // singleton pointer for esekfom raw-function callback
    static SlamCoreImpl* s_instance;
    static void h_share_model_static(state_ikfom& s,
                                     esekfom::dyn_share_datastruct<double>& ekfom_data)
    {
        s_instance->h_share_model(s, ekfom_data);
    }

    SlamCoreImpl()
        : p_imu_(std::make_shared<ImuProcess>())
        , featsFromMap_(new PointCloudXYZI())
        , feats_undistort_(new PointCloudXYZI())
        , feats_down_body_(new PointCloudXYZI())
        , feats_down_world_(new PointCloudXYZI())
        , normvec_(new PointCloudXYZI(100000, 1))
        , laserCloudOri_(new PointCloudXYZI(100000, 1))
        , corr_normvect_(new PointCloudXYZI(100000, 1))
        , _featsArray_(new PointCloudXYZI())
        , pcl_wait_pub_(new PointCloudXYZI())
        , XAxisPoint_body_(LIDAR_SP_LEN, 0.0f, 0.0f)
        , XAxisPoint_world_(LIDAR_SP_LEN, 0.0f, 0.0f)
        , position_last_(Zero3d)
        , Lidar_T_wrt_IMU_(Zero3d)
        , Lidar_R_wrt_IMU_(Eye3d)
    {
        s_instance = this;
        memset(point_selected_surf_, true,    sizeof(point_selected_surf_));
        memset(res_last_,            -1000.0f, sizeof(res_last_));
        fill(epsi_, epsi_ + 23, 0.001);
    }

    ~SlamCoreImpl()
    {
        if (fp_) { fclose(fp_); fp_ = nullptr; }
        if (fout_pre_.is_open()) fout_pre_.close();
        if (fout_out_.is_open()) fout_out_.close();
    }

    // ── public interface methods ───────────────────────────────────────────────
    void init(const SlamParams& p)
    {
        params_                   = p;
        filter_size_surf_min_     = p.filter_size_surf_min;
        filter_size_map_min_      = p.filter_size_map_min;
        cube_len_                 = p.cube_len;
        DET_RANGE_                = p.det_range;
        extrinsic_est_en_         = p.extrinsic_est_en;
        NUM_MAX_ITERATIONS_       = p.num_max_iterations;
        time_sync_en_             = p.time_sync_en;
        time_diff_lidar_to_imu_   = p.time_diff_lidar_to_imu;
        runtime_pos_log_          = p.runtime_pos_log;
        pcd_save_en_              = p.pcd_save_en;
        map_file_path_            = p.map_file_path;
        root_dir_                 = p.root_dir;

        downSizeFilterSurf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);
        downSizeFilterMap_.setLeafSize(filter_size_map_min_,   filter_size_map_min_,  filter_size_map_min_);

        Lidar_T_wrt_IMU_ << VEC_FROM_ARRAY(p.extrinT);
        Lidar_R_wrt_IMU_ << MAT_FROM_ARRAY(p.extrinR);

        p_imu_->set_extrinsic(Lidar_T_wrt_IMU_, Lidar_R_wrt_IMU_);
        p_imu_->set_gyr_cov(V3D(p.gyr_cov,   p.gyr_cov,   p.gyr_cov));
        p_imu_->set_acc_cov(V3D(p.acc_cov,   p.acc_cov,   p.acc_cov));
        p_imu_->set_gyr_bias_cov(V3D(p.b_gyr_cov, p.b_gyr_cov, p.b_gyr_cov));
        p_imu_->set_acc_bias_cov(V3D(p.b_acc_cov, p.b_acc_cov, p.b_acc_cov));

        kf_.init_dyn_share(get_f, df_dx, df_dw,
                           SlamCoreImpl::h_share_model_static,
                           NUM_MAX_ITERATIONS_, epsi_);

        std::string pos_log_dir = root_dir_ + "/Log/pos_log.txt";
        fp_ = fopen(pos_log_dir.c_str(), "w");
        fout_pre_.open(root_dir_ + "/Log/mat_pre.txt", std::ios::out);
        fout_out_.open(root_dir_ + "/Log/mat_out.txt", std::ios::out);
    }

    void pushLidar(PointCloudXYZI::Ptr cloud, double timestamp)
    {
        std::lock_guard<std::mutex> lk(mtx_buffer_);
        scan_count_++;

        if (!is_first_lidar_ && timestamp < last_timestamp_lidar_)
        {
            std::cerr << "lidar loop back, clear buffer\n";
            lidar_buffer_.clear();
            time_buffer_.clear();
        }
        if (is_first_lidar_) is_first_lidar_ = false;

        last_timestamp_lidar_ = timestamp;

        if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 &&
            last_timestamp_imu_ > 0 && !imu_buffer_.empty() && !lidar_buffer_.empty())
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf\n",
                   last_timestamp_imu_, last_timestamp_lidar_);
        }
        if (time_sync_en_ && !timediff_set_flg_ &&
            abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 && !imu_buffer_.empty())
        {
            timediff_set_flg_       = true;
            timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
            printf("Self sync IMU and LiDAR, time diff is %.10lf\n", timediff_lidar_wrt_imu_);
        }

        lidar_buffer_.push_back(cloud);
        time_buffer_.push_back(timestamp);
    }

    void pushImu(const sensor_msgs::msg::Imu::SharedPtr msg_in)
    {
        publish_count_++;
        sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

        msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu_);

        if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_)
        {
            msg->header.stamp =
                rclcpp::Time(timediff_lidar_wrt_imu_ + get_time_sec(msg_in->header.stamp));
        }

        double timestamp = get_time_sec(msg->header.stamp);

        std::lock_guard<std::mutex> lk(mtx_buffer_);
        if (timestamp < last_timestamp_imu_)
        {
            std::cerr << "imu loop back, clear buffer\n";
            imu_buffer_.clear();
        }
        last_timestamp_imu_ = timestamp;
        imu_buffer_.push_back(msg);
    }

    void spinFrontendOnce()
    {
        if (!syncPackages(Measures_))
        {
            updateSnapshot(false);
            return;
        }

        if (flg_first_scan_)
        {
            first_lidar_time_          = Measures_.lidar_beg_time;
            p_imu_->first_lidar_time   = first_lidar_time_;
            flg_first_scan_            = false;
            updateSnapshot(false);
            return;
        }

        double t0, t1, t3, t5;
        match_time_         = 0;
        kdtree_search_time_ = 0.0;
        solve_time_         = 0.0;
        t0 = omp_get_wtime();

        p_imu_->Process(Measures_, kf_, feats_undistort_);
        state_point_ = kf_.get_x();
        pos_lid_     = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

        if (feats_undistort_->empty() || feats_undistort_ == nullptr)
        {
            updateSnapshot(false);
            return;
        }

        flg_EKF_inited_ = (Measures_.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

        lasermapFovSegment();

        downSizeFilterSurf_.setInputCloud(feats_undistort_);
        downSizeFilterSurf_.filter(*feats_down_body_);
        t1 = omp_get_wtime();
        feats_down_size_ = feats_down_body_->points.size();

        if (ikdtree_.Root_Node == nullptr)
        {
            if (feats_down_size_ > 5)
            {
                ikdtree_.set_downsample_param(filter_size_map_min_);
                feats_down_world_->resize(feats_down_size_);
                for (int i = 0; i < feats_down_size_; i++)
                    pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
                ikdtree_.Build(feats_down_world_->points);
            }
            updateSnapshot(false);
            return;
        }

        int featsFromMapNum = ikdtree_.validnum();
        kdtree_size_st_ = ikdtree_.size();
        (void)featsFromMapNum;

        if (feats_down_size_ < 5)
        {
            updateSnapshot(false);
            return;
        }

        normvec_->resize(feats_down_size_);
        feats_down_world_->resize(feats_down_size_);

        V3D ext_euler = SO3ToEuler(state_point_.offset_R_L_I);
        fout_pre_ << std::setw(20) << Measures_.lidar_beg_time - first_lidar_time_
                  << " " << euler_cur_.transpose()
                  << " " << state_point_.pos.transpose()
                  << " " << ext_euler.transpose()
                  << " " << state_point_.offset_T_L_I.transpose()
                  << " " << state_point_.vel.transpose()
                  << " " << state_point_.bg.transpose()
                  << " " << state_point_.ba.transpose()
                  << " " << state_point_.grav << "\n";

        pointSearchInd_surf_.resize(feats_down_size_);
        Nearest_Points_.resize(feats_down_size_);

        double t_update_start = omp_get_wtime();
        double solve_H_time   = 0;
        kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

        state_point_ = kf_.get_x();
        euler_cur_   = SO3ToEuler(state_point_.rot);
        pos_lid_     = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

        double t_update_end = omp_get_wtime();

        // accumulate map cloud
        {
            int sz = feats_undistort_->points.size();
            PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(sz, 1));
            for (int i = 0; i < sz; i++)
                rgbPointBodyToWorld(&feats_undistort_->points[i], &laserCloudWorld->points[i]);
            *pcl_wait_pub_ += *laserCloudWorld;
        }

        t3 = omp_get_wtime();
        mapIncremental();
        t5 = omp_get_wtime();

        if (runtime_pos_log_)
        {
            frame_num_++;
            kdtree_size_end_         = ikdtree_.size();
            aver_time_consu_         = aver_time_consu_        * (frame_num_ - 1) / frame_num_ + (t5 - t0) / frame_num_;
            aver_time_icp_           = aver_time_icp_          * (frame_num_ - 1) / frame_num_ + (t_update_end - t_update_start) / frame_num_;
            aver_time_match_         = aver_time_match_        * (frame_num_ - 1) / frame_num_ + match_time_ / frame_num_;
            aver_time_incre_         = aver_time_incre_        * (frame_num_ - 1) / frame_num_ + kdtree_incremental_time_ / frame_num_;
            aver_time_solve_         = aver_time_solve_        * (frame_num_ - 1) / frame_num_ + (solve_time_ + solve_H_time) / frame_num_;
            aver_time_const_H_time_  = aver_time_const_H_time_ * (frame_num_ - 1) / frame_num_ + solve_time_ / frame_num_;

            printf("[ mapping ]: IMU+Map: %0.6f match: %0.6f solve: %0.6f ICP: %0.6f incre: %0.6f total: %0.6f icp: %0.6f H: %0.6f\n",
                t1 - t0, aver_time_match_, aver_time_solve_, t3 - t1, t5 - t3,
                aver_time_consu_, aver_time_icp_, aver_time_const_H_time_);

            ext_euler = SO3ToEuler(state_point_.offset_R_L_I);
            fout_out_ << std::setw(20) << Measures_.lidar_beg_time - first_lidar_time_
                      << " " << euler_cur_.transpose()
                      << " " << state_point_.pos.transpose()
                      << " " << ext_euler.transpose()
                      << " " << state_point_.offset_T_L_I.transpose()
                      << " " << state_point_.vel.transpose()
                      << " " << state_point_.bg.transpose()
                      << " " << state_point_.ba.transpose()
                      << " " << state_point_.grav
                      << " " << feats_undistort_->points.size() << "\n";

            dumpLioStateToLog();
        }

        updateSnapshot(true);
    }

    SlamSnapshot getSnapshot() const
    {
        std::lock_guard<std::mutex> lk(snapshot_mutex_);
        return latest_snapshot_;
    }

    bool saveMap() const
    {
        if (!pcd_save_en_) return false;
        auto snap = getSnapshot();
        if (!snap.valid || !snap.pcl_wait_pub || snap.pcl_wait_pub->empty()) return false;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(map_file_path_, *snap.pcl_wait_pub);
        return true;
    }

private:
    // ── sync packages ──────────────────────────────────────────────────────────
    bool syncPackages(MeasureGroup& meas)
    {
        if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;

        if (!lidar_pushed_)
        {
            meas.lidar          = lidar_buffer_.front();
            meas.lidar_beg_time = time_buffer_.front();

            if (meas.lidar->points.size() <= 1)
            {
                lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_)
            {
                lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
            }
            else
            {
                scan_num_++;
                lidar_end_time_ = meas.lidar_beg_time +
                                  meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime_ +=
                    (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
            }
            meas.lidar_end_time = lidar_end_time_;
            lidar_pushed_       = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) return false;

        double imu_time = get_time_sec(imu_buffer_.front()->header.stamp);
        meas.imu.clear();
        while (!imu_buffer_.empty() && imu_time < lidar_end_time_)
        {
            imu_time = get_time_sec(imu_buffer_.front()->header.stamp);
            if (imu_time > lidar_end_time_) break;
            meas.imu.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        lidar_pushed_ = false;
        return true;
    }

    // ── geometry helpers ───────────────────────────────────────────────────────
    void pointBodyToWorld(PointType const* pi, PointType* po) const
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) + state_point_.pos);
        po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    void rgbPointBodyToWorld(PointType const* pi, PointType* po) const
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) + state_point_.pos);
        po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    // ── lasermap fov segment ───────────────────────────────────────────────────
    void lasermapFovSegment()
    {
        cub_needrm_.clear();
        kdtree_delete_counter_ = 0;
        kdtree_delete_time_    = 0.0;

        {
            PointType pb, pw;
            pb.x = XAxisPoint_body_.x(); pb.y = XAxisPoint_body_.y(); pb.z = XAxisPoint_body_.z();
            pb.intensity = 0;
            pointBodyToWorld(&pb, &pw);
            XAxisPoint_world_.x() = pw.x; XAxisPoint_world_.y() = pw.y; XAxisPoint_world_.z() = pw.z;
        }

        V3D pos_LiD = pos_lid_;
        if (!Localmap_Initialized_)
        {
            for (int i = 0; i < 3; i++)
            {
                LocalMap_Points_.vertex_min[i] = pos_LiD(i) - cube_len_ / 2.0;
                LocalMap_Points_.vertex_max[i] = pos_LiD(i) + cube_len_ / 2.0;
            }
            Localmap_Initialized_ = true;
            return;
        }

        constexpr float MOV_THRESHOLD = 1.5f;
        float dist_to_map_edge[3][2];
        bool  need_move = false;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points_.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE_ ||
                dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE_)
                need_move = true;
        }
        if (!need_move) return;

        BoxPointType New_LocalMap_Points = LocalMap_Points_;
        BoxPointType tmp_boxpoints;
        float mov_dist = max((cube_len_ - 2.0 * MOV_THRESHOLD * DET_RANGE_) * 0.5 * 0.9,
                            double(DET_RANGE_ * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = LocalMap_Points_;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE_)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points_.vertex_max[i] - mov_dist;
                cub_needrm_.push_back(tmp_boxpoints);
            }
            else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE_)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points_.vertex_min[i] + mov_dist;
                cub_needrm_.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points_ = New_LocalMap_Points;

        {
            PointVector points_history;
            ikdtree_.acquire_removed_points(points_history);
        }
        double delete_begin = omp_get_wtime();
        if (!cub_needrm_.empty())
            kdtree_delete_counter_ = ikdtree_.Delete_Point_Boxes(cub_needrm_);
        kdtree_delete_time_ = omp_get_wtime() - delete_begin;
    }

    // ── map incremental ────────────────────────────────────────────────────────
    void mapIncremental()
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size_);
        PointNoNeedDownsample.reserve(feats_down_size_);

        for (int i = 0; i < feats_down_size_; i++)
        {
            pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));

            if (!Nearest_Points_[i].empty() && flg_EKF_inited_)
            {
                const PointVector& points_near = Nearest_Points_[i];
                bool need_add = true;
                PointType mid_point;
                mid_point.x = floor(feats_down_world_->points[i].x / filter_size_map_min_) * filter_size_map_min_ + 0.5f * filter_size_map_min_;
                mid_point.y = floor(feats_down_world_->points[i].y / filter_size_map_min_) * filter_size_map_min_ + 0.5f * filter_size_map_min_;
                mid_point.z = floor(feats_down_world_->points[i].z / filter_size_map_min_) * filter_size_map_min_ + 0.5f * filter_size_map_min_;
                float dist = calc_dist(feats_down_world_->points[i], mid_point);

                if (fabs(points_near[0].x - mid_point.x) > 0.5f * filter_size_map_min_ &&
                    fabs(points_near[0].y - mid_point.y) > 0.5f * filter_size_map_min_ &&
                    fabs(points_near[0].z - mid_point.z) > 0.5f * filter_size_map_min_)
                {
                    PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
                {
                    if ((int)points_near.size() < NUM_MATCH_POINTS) break;
                    if (calc_dist(points_near[readd_i], mid_point) < dist) { need_add = false; break; }
                }
                if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
            }
            else
            {
                PointToAdd.push_back(feats_down_world_->points[i]);
            }
        }

        double st_time = omp_get_wtime();
        add_point_size_ = ikdtree_.Add_Points(PointToAdd, true);
        ikdtree_.Add_Points(PointNoNeedDownsample, false);
        add_point_size_ = PointToAdd.size() + PointNoNeedDownsample.size();
        kdtree_incremental_time_ = omp_get_wtime() - st_time;
    }

    // ── h_share_model ──────────────────────────────────────────────────────────
    void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
    {
        double match_start = omp_get_wtime();
        laserCloudOri_->clear();
        corr_normvect_->clear();
        total_residual_ = 0.0;

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size_; i++)
        {
            PointType& point_body  = feats_down_body_->points[i];
            PointType& point_world = feats_down_world_->points[i];

            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0); point_world.y = p_global(1);
            point_world.z = p_global(2); point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto& points_near = Nearest_Points_[i];

            if (ekfom_data.converge)
            {
                ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf_[i] = (points_near.size() >= (size_t)NUM_MATCH_POINTS) &&
                                           pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5.0f;
            }

            if (!point_selected_surf_[i]) continue;

            VF(4) pabcd;
            point_selected_surf_[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0)*point_world.x + pabcd(1)*point_world.y + pabcd(2)*point_world.z + pabcd(3);
                float s   = 1.0f - 0.9f * fabs(pd2) / sqrt(p_body.norm());
                if (s > 0.9f)
                {
                    point_selected_surf_[i] = true;
                    normvec_->points[i].x   = pabcd(0);
                    normvec_->points[i].y   = pabcd(1);
                    normvec_->points[i].z   = pabcd(2);
                    normvec_->points[i].intensity = pd2;
                    res_last_[i] = fabs(pd2);
                }
            }
        }

        effct_feat_num_ = 0;
        for (int i = 0; i < feats_down_size_; i++)
        {
            if (point_selected_surf_[i])
            {
                laserCloudOri_->points[effct_feat_num_]  = feats_down_body_->points[i];
                corr_normvect_->points[effct_feat_num_]  = normvec_->points[i];
                total_residual_ += res_last_[i];
                effct_feat_num_++;
            }
        }

        if (effct_feat_num_ < 1)
        {
            ekfom_data.valid = false;
            std::cerr << "No Effective Points!\n";
            return;
        }

        res_mean_last_ = total_residual_ / effct_feat_num_;
        match_time_   += omp_get_wtime() - match_start;
        double solve_start = omp_get_wtime();

        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num_, 12);
        ekfom_data.h.resize(effct_feat_num_);

        for (int i = 0; i < effct_feat_num_; i++)
        {
            const PointType& laser_p = laserCloudOri_->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            const PointType& norm_p = corr_normvect_->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
            V3D C(s.rot.conjugate() * norm_vec);
            V3D A(point_crossmat * C);

            if (extrinsic_est_en_)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                    VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                    VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }
            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time_ += omp_get_wtime() - solve_start;
    }

    // ── log helper ─────────────────────────────────────────────────────────────
    void dumpLioStateToLog()
    {
        if (!fp_) return;
        V3D rot_ang(Log(state_point_.rot.toRotationMatrix()));
        fprintf(fp_, "%lf ", Measures_.lidar_beg_time - first_lidar_time_);
        fprintf(fp_, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));
        fprintf(fp_, "%lf %lf %lf ", state_point_.pos(0), state_point_.pos(1), state_point_.pos(2));
        fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);
        fprintf(fp_, "%lf %lf %lf ", state_point_.vel(0), state_point_.vel(1), state_point_.vel(2));
        fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);
        fprintf(fp_, "%lf %lf %lf ", state_point_.bg(0), state_point_.bg(1), state_point_.bg(2));
        fprintf(fp_, "%lf %lf %lf ", state_point_.ba(0), state_point_.ba(1), state_point_.ba(2));
        fprintf(fp_, "%lf %lf %lf ", state_point_.grav[0], state_point_.grav[1], state_point_.grav[2]);
        fprintf(fp_, "\r\n");
        fflush(fp_);
    }

    // ── update snapshot ────────────────────────────────────────────────────────
    void updateSnapshot(bool valid)
    {
        SlamSnapshot snap;
        snap.valid          = valid;
        snap.lidar_end_time = lidar_end_time_;
        if (valid)
        {
            snap.pose.pos          = state_point_.pos;
            snap.pose.rot          = state_point_.rot;
            snap.pose.vel          = state_point_.vel;
            snap.pose.offset_R_L_I = state_point_.offset_R_L_I.toRotationMatrix();
            snap.pose.offset_T_L_I = state_point_.offset_T_L_I;

            snap.geoQuat.x = state_point_.rot.coeffs()[0];
            snap.geoQuat.y = state_point_.rot.coeffs()[1];
            snap.geoQuat.z = state_point_.rot.coeffs()[2];
            snap.geoQuat.w = state_point_.rot.coeffs()[3];

            snap.cov_P           = kf_.get_P();
            snap.feats_undistort = feats_undistort_;
            snap.feats_down_body = feats_down_body_;
            snap.laserCloudOri   = laserCloudOri_;
            snap.pcl_wait_pub    = pcl_wait_pub_;
            snap.effct_feat_num  = effct_feat_num_;
        }

        std::lock_guard<std::mutex> lk(snapshot_mutex_);
        latest_snapshot_ = snap;
    }

public:
    // ── member variables ───────────────────────────────────────────────────────
    std::mutex mtx_buffer_;
    std::deque<double>                                  time_buffer_;
    std::deque<PointCloudXYZI::Ptr>                     lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr>   imu_buffer_;

    double last_timestamp_lidar_ = 0.0;
    double last_timestamp_imu_   = -1.0;
    bool   lidar_pushed_         = false;
    bool   is_first_lidar_       = true;
    double timediff_lidar_wrt_imu_ = 0.0;
    bool   timediff_set_flg_       = false;
    double lidar_mean_scantime_ = 0.0;
    int    scan_num_            = 0;
    double lidar_end_time_      = 0.0;

    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    vect3       pos_lid_;

    MeasureGroup Measures_;
    double first_lidar_time_ = 0.0;
    bool   flg_first_scan_   = true;
    bool   flg_EKF_inited_   = false;
    double epsi_[23];

    std::shared_ptr<ImuProcess> p_imu_;

    KD_TREE<PointType>         ikdtree_;
    BoxPointType               LocalMap_Points_;
    bool                       Localmap_Initialized_ = false;

    std::vector<BoxPointType>            cub_needrm_;
    std::vector<PointVector>             Nearest_Points_;
    std::vector<std::vector<int>>        pointSearchInd_surf_;

    pcl::VoxelGrid<PointType>  downSizeFilterSurf_;
    pcl::VoxelGrid<PointType>  downSizeFilterMap_;

    PointCloudXYZI::Ptr featsFromMap_;
    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    PointCloudXYZI::Ptr normvec_;
    PointCloudXYZI::Ptr laserCloudOri_;
    PointCloudXYZI::Ptr corr_normvect_;
    PointCloudXYZI::Ptr _featsArray_;
    PointCloudXYZI::Ptr pcl_wait_pub_;

    V3F XAxisPoint_body_;
    V3F XAxisPoint_world_;

    float  res_last_[100000];
    bool   point_selected_surf_[100000];
    int    feats_down_size_   = 0;
    int    effct_feat_num_    = 0;
    double res_mean_last_     = 0.05;
    double total_residual_    = 0.0;

    double kdtree_incremental_time_ = 0.0;
    double kdtree_search_time_      = 0.0;
    double kdtree_delete_time_      = 0.0;
    int    kdtree_size_st_          = 0;
    int    kdtree_size_end_         = 0;
    int    add_point_size_          = 0;
    int    kdtree_delete_counter_   = 0;
    double match_time_              = 0.0;
    double solve_time_              = 0.0;
    int    scan_count_              = 0;
    int    publish_count_           = 0;

    SlamParams params_;
    float  DET_RANGE_;
    double cube_len_;
    double filter_size_map_min_;
    double filter_size_surf_min_;
    bool   extrinsic_est_en_          = true;
    bool   time_sync_en_              = false;
    double time_diff_lidar_to_imu_    = 0.0;
    bool   runtime_pos_log_           = false;
    bool   pcd_save_en_               = false;
    std::string map_file_path_;
    std::string root_dir_;
    int    NUM_MAX_ITERATIONS_        = 4;

    V3D euler_cur_         = V3D::Zero();
    V3D position_last_     = Zero3d;
    V3D Lidar_T_wrt_IMU_   = Zero3d;
    M3D Lidar_R_wrt_IMU_   = Eye3d;

    FILE*    fp_    = nullptr;
    std::ofstream fout_pre_, fout_out_;

    int    frame_num_              = 0;
    double aver_time_consu_        = 0;
    double aver_time_icp_          = 0;
    double aver_time_match_        = 0;
    double aver_time_incre_        = 0;
    double aver_time_solve_        = 0;
    double aver_time_const_H_time_ = 0;

    mutable std::mutex  snapshot_mutex_;
    SlamSnapshot        latest_snapshot_;
};

SlamCoreImpl* SlamCoreImpl::s_instance = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// SlamCore  –  thin wrapper around SlamCoreImpl
// ─────────────────────────────────────────────────────────────────────────────
SlamCore::SlamCore()  : impl_(std::make_unique<SlamCoreImpl>()) {}
SlamCore::~SlamCore() = default;

void SlamCore::init(const SlamParams& p)        { impl_->init(p); }
void SlamCore::pushLidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> cloud, double ts)
                                                { impl_->pushLidar(cloud, ts); }
void SlamCore::pushImu(const sensor_msgs::msg::Imu::SharedPtr msg) { impl_->pushImu(msg); }
void SlamCore::spinFrontendOnce()               { impl_->spinFrontendOnce(); }
SlamSnapshot SlamCore::getSnapshot() const      { return impl_->getSnapshot(); }
bool SlamCore::saveMap() const                  { return impl_->saveMap(); }
