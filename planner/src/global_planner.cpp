#include "global_planner.h"

#include <iostream>
#include <atomic>
#include <omp.h>

namespace global_planner
{

    GlobalPlanner::GlobalPlanner():map_ready_(false),has_start_(false),has_goal_(false),planning_in_progress_(false),plan_seq_(0),last_success_seq_(0)
    {
        std::cout << "GlobalPlanner Constructure!!! " << std::endl;
    }

    GlobalPlanner::~GlobalPlanner()
    {
        std::cout << "GlobalPlanner Destructure!!! " << std::endl;
    }

    void GlobalPlanner::setOctomap(std::shared_ptr<octomap::OcTree> map)
    {
        if (!map)
        {
            std::cout << "Octomap is Null!!! return." << std::endl;
            return;
        }

        if (octree_ == map)
        {
            std::cout << "Octomap is No Update!!! return." << std::endl;
            return;
        }

        std::cout << "GlobalPlanner::setOctomap setting Octomap... " << std::endl;

        octree_ = map;
        map_ready_ = true;
        buildOccupancyCache();
        rebuildPreblockedCells();
        rebuildDerivedLayers();
        rebuildPreblockedCostmap();
        buildTraversableGrid();
    }

    void GlobalPlanner::makePlan(const PointPose start,const PointPose goal)
    {
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;

        std::cout << "start = (" << start_point_.x << "," << start_point_.y << "," << start_point_.z << "),goal = (" << goal_point_.x << "," << goal_point_.y << "," << goal_point_.z << ") " << std::endl;
         
        tryPlan();

    }

    void GlobalPlanner::tryPlan()
    {
        std::cout << "GlobalPlanner::tryPlan planning..." << std::endl;
        if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) 
        {
            std::cout << "GlobalPlanner::tryPlan 异常退出规划器" << std::endl;
            return;
        }
        planning_in_progress_ = true;
        ++plan_seq_;
        const bool ok = startPlan();
        planning_in_progress_ = false;
        if (!ok) 
        {
            std::cout << "GlobalPlanner::tryPlan() A* planning failed. " << std::endl;
            planner_results_.clear();  // 失败时清空旧路径，避免返回过期结果
        } 
        else 
        {
            last_success_seq_ = plan_seq_;
        }
    }

    void GlobalPlanner::getPlannerResults(std::vector<PointPose>& plannerResults)
    {
       plannerResults = planner_results_;
    }

    bool GlobalPlanner::startPlan()
    {
        const double robot_radius = robot_radius_;
        const int max_iterations = max_iterations_;
        const int snap_radius = snap_search_radius_cells_;
        const bool require_ground_support = require_ground_support_; 
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells =  ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const bool enable_preblocked_costmap =  enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;

        const GridIndex start_raw = worldToGrid(
        start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToGrid(
        goal_point_.x, goal_point_.y, goal_point_.z);

        GridIndex start = start_raw;
        GridIndex goal = goal_raw;
        const bool start_ok = findNearestFreeCell(
        start_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, start);
        const bool goal_ok = findNearestFreeCell(
        goal_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, goal);

        if (!start_ok) 
        {
            std::cout << "GlobalPlanner::startPlan() Start is occupied/out of map and no nearby free cell." << std::endl;
            return false;
        }

        if (!goal_ok) 
        {
            std::cout << "GlobalPlanner::startPlan() Goal is occupied/out of map and no nearby free cell." << std::endl;
            return false;
        }

        if (!(start == start_raw)) 
        {
            const auto p = gridToWorld(start);
            std::cout << "GlobalPlanner::startPlan() Start snapped to free cell: [" << p.x() << ", " << p.y() << ", " << p.z() << "] " << std::endl;
        }

        if (!(goal == goal_raw))
        {
            const auto p = gridToWorld(goal);
            std::cout << "GlobalPlanner::startPlan() Goal snapped to free cell: [" << p.x() << ", " << p.y() << ", " << p.z() << "] " << std::endl;
        }

        const int64_t grid_size = grid_dim_x_ * grid_dim_y_ * grid_dim_z_;
        const double INF = 1e20;

        std::vector<double> g_score(grid_size, INF);
        std::vector<int64_t> came_from(grid_size, -1);
        std::vector<bool> closed_set(grid_size, false);

        const int64_t start_lin = gridLinear(start);
        g_score[start_lin] = 0.0;

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;

        const auto h_start = euclidean(start, goal);
        open_set.push(QueueNode{start, h_start, 0.0});

        std::vector<std::pair<GridIndex, double>> directions;
        directions.reserve(26);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    const double dist = std::sqrt(static_cast<double>(dx*dx + dy*dy + dz*dz));
                    directions.emplace_back(GridIndex{dx, dy, dz}, dist);
                }
            }
        }

        int iters = 0;
        const clock_t plan_start = clock();
        const int log_interval = std::max(1, max_iterations / 20);
        int next_log_iter = log_interval;

        std::cout << "  A* planning: start=(" << start.x << "," << start.y << "," << start.z << ") goal=(" << goal.x << "," << goal.y << "," << goal.z << "), max_iterations=" << max_iterations << ", grid resolution=" << octree_->getResolution() << ", grid_size=" << grid_size << std::endl;

        const int64_t goal_lin = gridLinear(goal);

        while (!open_set.empty() && iters < max_iterations) 
        {
            const QueueNode current = open_set.top();
            open_set.pop();
            ++iters;

            const int64_t cur_lin = gridLinear(current.idx);
            if (closed_set[cur_lin]) {
                continue;
            }
            closed_set[cur_lin] = true;

            if (cur_lin == goal_lin) {
                std::vector<GridIndex> path;
                GridIndex c = current.idx;
                int64_t lin = cur_lin;
                path.push_back(c);
                while (came_from[lin] >= 0) {
                    const int64_t parent_lin = came_from[lin];
                    const int64_t z = parent_lin % grid_dim_z_;
                    const int64_t rem = parent_lin / grid_dim_z_;
                    const int64_t y = rem % grid_dim_y_;
                    const int64_t x = rem / grid_dim_y_;
                    c = GridIndex{static_cast<int>(x + grid_min_.x), static_cast<int>(y + grid_min_.y), static_cast<int>(z + grid_min_.z)};
                    path.push_back(c);
                    lin = parent_lin;
                }
                std::reverse(path.begin(), path.end());

                const double elapsed = (clock() - plan_start) / static_cast<double>(CLOCKS_PER_SEC);
                std::cout << "  A* planning: path found in " << iters << " iterations (" << path.size() << " waypoints, " << elapsed << " s)" << std::endl;
                planner_results_.clear();
                for (std::size_t i = 0; i < path.size(); ++i) 
                {
                    const auto & c = path[i];
                    const auto p = gridToWorld(c);
                    PointPose temp;
                    temp.x = p.x();
                    temp.y = p.y();
                    temp.z = p.z();
                    planner_results_.push_back(temp);
                }
                return true;
            }

            if (iters >= next_log_iter) {
                const double elapsed = (clock() - plan_start) / static_cast<double>(CLOCKS_PER_SEC);
                const auto cur_world = gridToWorld(current.idx);
                const double dist_to_goal = std::sqrt(
                    static_cast<double>((current.idx.x - goal.x) * (current.idx.x - goal.x) +
                                        (current.idx.y - goal.y) * (current.idx.y - goal.y) +
                                        (current.idx.z - goal.z) * (current.idx.z - goal.z))) * octree_->getResolution();
                int closed_count = 0;
                for (bool b : closed_set) if (b) closed_count++;
                std::cout << "  A* planning: " << iters << " / " << max_iterations << " iterations (" << (100.0 * iters / max_iterations) << "%), open=" << open_set.size() << ", closed=" << closed_count << ", dist_to_goal=" << dist_to_goal << ", " << elapsed << " s" << std::endl;
                next_log_iter = iters + log_interval;
            }

            for (const auto & dir : directions)
            {
                GridIndex nbr{current.idx.x + dir.first.x, current.idx.y + dir.first.y, current.idx.z + dir.first.z};

                if (!isTraversableGrid(nbr)) {
                    continue;
                }

                const int64_t nbr_lin = gridLinear(nbr);
                if (closed_set[nbr_lin]) {
                    continue;
                }

                double tentative_g = current.g + dir.second;
                if (enable_preblocked_costmap) {
                    tentative_g += preblocked_costmap_weight * getPreblockedCostGrid(nbr);
                }

                // 方向一致性惩罚：减少在阶跃位置斜向变向，鼓励沿路径方向直行
                if (came_from[cur_lin] >= 0) {
                    // 从 came_from 恢复父节点 GridIndex
                    const int64_t parent_lin = came_from[cur_lin];
                    const int64_t pz = parent_lin % grid_dim_z_;
                    const int64_t prem = parent_lin / grid_dim_z_;
                    const int64_t py = prem % grid_dim_y_;
                    const int64_t px = prem / grid_dim_y_;
                    const GridIndex parent{
                        static_cast<int>(px + grid_min_.x),
                        static_cast<int>(py + grid_min_.y),
                        static_cast<int>(pz + grid_min_.z)};

                    // 进方向 (parent → current) 和出方向 (current → nbr)
                    const int in_dx = current.idx.x - parent.x;
                    const int in_dy = current.idx.y - parent.y;
                    const int in_dz = current.idx.z - parent.z;
                    const int out_dx = dir.first.x;
                    const int out_dy = dir.first.y;
                    const int out_dz = dir.first.z;

                    // 归一化点积 → cos(angle)，用于衡量方向变化
                    const double in_len = std::sqrt(static_cast<double>(in_dx * in_dx + in_dy * in_dy + in_dz * in_dz));
                    const double out_len = dir.second;
                    if (in_len > 1e-6 && out_len > 1e-6) {
                        const double dot = (in_dx * out_dx + in_dy * out_dy + in_dz * out_dz) / (in_len * out_len);
                        // (1 - dot) / 2 ∈ [0, 1]：0=同向, 0.5=正交, 1=反向
                        const double dir_change = (1.0 - dot) * 0.5;
                        tentative_g += dir_change_weight_ * dir_change;
                    }
                }

                // 对角线跨越惩罚：xy 平面对角移动时，检查中间轴对齐格子
                // 若中间格子在目标 z 层不可通行，说明对角格子是噪声"桥梁"
                // 用于绕过高度变化，应加罚以避免折角路径
                if (diagonal_bridge_weight_ > 0.0 &&
                    std::abs(dir.first.x) + std::abs(dir.first.y) >= 2) {
                    bool bridge = false;
                    // 中间格1: 仅沿 x 方向走到目标列，y 保持当前行
                    const GridIndex mid1{
                        current.idx.x + dir.first.x,
                        current.idx.y,
                        current.idx.z + dir.first.z};
                    if (!isTraversableGrid(mid1)) bridge = true;
                    // 中间格2: 仅沿 y 方向走到目标行，x 保持当前列
                    const GridIndex mid2{
                        current.idx.x,
                        current.idx.y + dir.first.y,
                        current.idx.z + dir.first.z};
                    if (!isTraversableGrid(mid2)) bridge = true;
                    if (bridge) {
                        tentative_g += diagonal_bridge_weight_;
                    }
                }

                if (tentative_g < g_score[nbr_lin]) {
                    came_from[nbr_lin] = cur_lin;
                    g_score[nbr_lin] = tentative_g;
                    const double h = std::sqrt(
                        static_cast<double>((nbr.x - goal.x) * (nbr.x - goal.x) +
                                            (nbr.y - goal.y) * (nbr.y - goal.y) +
                                            (nbr.z - goal.z) * (nbr.z - goal.z)));
                    const double f = tentative_g + h;
                    open_set.push(QueueNode{nbr, f, tentative_g});
                }
            }
        }

        const double elapsed = (clock() - plan_start) / static_cast<double>(CLOCKS_PER_SEC);
        int closed_count = 0;
        for (bool b : closed_set) if (b) closed_count++;
        std::cout << "  A* planning: FAILED after " << iters << " / " << max_iterations << " iterations (" << elapsed << " s), open=" << open_set.size() << ", closed=" << closed_count << std::endl;
        return false;
    }

    std::vector<GridIndex> GlobalPlanner::reconstructPath(const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,GridIndex current) const
    {
        std::vector<GridIndex> path;
        path.push_back(current);
        while (came_from.find(current) != came_from.end()) 
        {
            current = came_from.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    bool GlobalPlanner::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells, bool require_ground_support,bool strict_direct_ground_support, int support_xy_radius_cells, int support_depth_cells,GridIndex & out) const
    {
        if (isCellTraversable(
            seed, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells))
        {
        out = seed;
        return true;
        }

        for (int r = 1; r <= radius_cells; ++r) {
        for (int dz = 0; dz <= r; ++dz) {
            for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                continue;
                }

                GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
                if (isCellTraversable(
                    c1, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                out = c1;
                return true;
                }

                if (dz > 0) {
                GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
                if (isCellTraversable(
                    c2, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                    out = c2;
                    return true;
                }
                }
            }
            }
        }
        }
        return false;
    }

    double GlobalPlanner::getPreblockedCost(const GridIndex & idx) const
    {
        const auto it = preblocked_costmap_.find(idx);
        if (it == preblocked_costmap_.end()) {
        return 0.0;
        }
        return it->second;
    }

    std::vector<GridIndex> GlobalPlanner::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        dirs.reserve(26);
        for (int dx = -1; dx <= 1; ++dx) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                for (int dz = -1; dz <= 1; ++dz) 
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }
                    dirs.push_back(GridIndex{dx, dy, dz});
                }
            }
        }
        return dirs;
    }

    bool GlobalPlanner::isCellTraversable(const GridIndex & idx, double robot_radius, bool require_ground_support,bool strict_direct_ground_support,int support_xy_radius_cells, int support_depth_cells) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }

        if (require_ground_support &&
        !hasGroundSupport(
            idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
        {
        return false;
        }

        for (int z = idx.z - 1; z >= 0; --z) {
        const GridIndex below_idx{idx.x, idx.y, z};
        if (isOccupiedCell(below_idx)) {
            break;
        }
        if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) {
            return false;
        }
        }

        const octomap::point3d center = gridToWorld(idx);
        const double r = octree_->getResolution();
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
        const double radius_sq = robot_radius * robot_radius;

        // Collision check for vehicle body volume (same height and above),
        // while allowing occupied support cells below. Apply the same footprint
        // rule to preblocked cells so a cell is rejected if the vehicle radius
        // overlaps any preblocked voxel.
        for (int dx = -n; dx <= n; ++dx) {
        for (int dy = -n; dy <= n; ++dy) {
            for (int dz = 0; dz <= n; ++dz) {
            const double dist_x = static_cast<double>(dx) * r;
            const double dist_y = static_cast<double>(dy) * r;
            const double dist_z = static_cast<double>(dz) * r;
            const double dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
            if (dist_sq > radius_sq) {
                continue;
            }
            const octomap::point3d p(
                center.x() + static_cast<float>(dx * r),
                center.y() + static_cast<float>(dy * r),
                center.z() + static_cast<float>(dz * r));
            const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
            if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) {
                return false;
            }
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                return false;
            }
            }
        }
        }
        return true;
    }

    bool GlobalPlanner::hasGroundSupport(const GridIndex & idx, bool strict_direct_ground_support, int support_xy_radius_cells,int support_depth_cells) const
    {
        if (strict_direct_ground_support) {
        GridIndex below{idx.x, idx.y, idx.z - 1};
        if (!isInsideMetricBounds(below)) {
            return false;
        }
        const auto p = gridToWorld(below);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
        }

        for (int dz = 1; dz <= std::max(1, support_depth_cells); ++dz) {
        for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
            for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
            GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
            if (!isInsideMetricBounds(below)) {
                continue;
            }
            const auto p = gridToWorld(below);
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                return true;
            }
            }
        }
        }
        return false;
    }

    void GlobalPlanner::rebuildPreblockedCells()
    {
        preblocked_cells_.clear();
        if (!octree_) {
        return;
        }

        const clock_t build_start = clock();
        const int nt = (num_threads_ > 0) ? num_threads_ : 0;

        // === Phase 1: 收集所有被占用的叶子节点位置（串行，快） ===
        std::vector<GridIndex> occupied;
        const size_t total_leafs = octree_->getNumLeafNodes();
        occupied.reserve(total_leafs);
        size_t skipped = 0;

        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupied.push_back(worldToGrid(it.getX(), it.getY(), it.getZ()));
            } else {
                ++skipped;
            }
        }
        std::cout << "  Preblocked cells: collected " << occupied.size()
                  << " occupied leaf nodes (" << skipped << " free, "
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;

        // === Phase 2: 并行生成候选体素 ===
        const size_t occ_size = occupied.size();
        std::atomic<int> processed{0};
        int next_log_pct = 10;

        const int num_threads = nt > 0 ? nt : omp_get_max_threads();
        std::vector<std::unordered_set<GridIndex, GridIndexHash>> local_candidates(num_threads);

#pragma omp parallel num_threads(nt) if(nt != 1)
        {
            const int tid = omp_get_thread_num();
            auto & my_candidates = local_candidates[tid];
            my_candidates.reserve(occ_size * 8 / num_threads);

#pragma omp for schedule(dynamic, 1024)
            for (int64_t i = 0; i < static_cast<int64_t>(occ_size); ++i) {
                const auto & occ = occupied[static_cast<size_t>(i)];
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        my_candidates.insert(GridIndex{occ.x + dx, occ.y + dy, occ.z});
                    }
                }

                const int done = processed.fetch_add(1) + 1;
                const int pct = done * 100 / static_cast<int>(occ_size);
                if (pct >= next_log_pct) {
#pragma omp critical
                    if (pct >= next_log_pct) {
                        size_t total_cand = 0;
                        for (const auto & lc : local_candidates) total_cand += lc.size();
                        std::cout << "  Preblocked cells: generating " << pct << "% ("
                                  << done << " / " << occ_size << " occ, "
                                  << total_cand << " candidates, "
                                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC))
                                  << " s)" << std::endl;
                        next_log_pct = pct + 10;
                        if (next_log_pct > 100) next_log_pct = 100;
                    }
                }
            }
        }

        // 合并线程局部候选集
        std::unordered_set<GridIndex, GridIndexHash> candidates;
        size_t total_cand = 0;
        for (const auto & lc : local_candidates) total_cand += lc.size();
        candidates.reserve(total_cand);
        for (auto & lc : local_candidates) {
            for (const auto & c : lc) candidates.insert(c);
        }
        std::cout << "  Preblocked cells: merged " << candidates.size()
                  << " unique candidates from " << occ_size << " occupied ("
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;

        // === Phase 3: 并行过滤候选体素 ===
        const int64_t cand_total = static_cast<int64_t>(candidates.size());
        if (cand_total == 0) {
            std::cout << "  Preblocked cells: done (0 candidates, "
                      << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;
            return;
        }

        std::vector<GridIndex> cand_vec(candidates.begin(), candidates.end());
        candidates.clear();

        processed = 0;
        next_log_pct = 10;

        std::vector<std::unordered_set<GridIndex, GridIndexHash>> local_preblocked(num_threads);

#pragma omp parallel num_threads(nt) if(nt != 1)
        {
            const int tid = omp_get_thread_num();
            auto & my_pb = local_preblocked[tid];
            my_pb.reserve(cand_total / num_threads);

#pragma omp for schedule(dynamic, 256)
            for (int64_t i = 0; i < cand_total; ++i) {
                const auto & c = cand_vec[static_cast<size_t>(i)];
                if (!isInsideMetricBounds(c)) continue;
                if (isOccupiedCell(c)) continue;

                const GridIndex below0{c.x, c.y, c.z - 1};
                const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
                if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
                    my_pb.insert(c);
                    continue;
                }
                const GridIndex above1{c.x, c.y, c.z + 1};
                const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
                if (!hasNonOccupiedNeighborSameLevel(c)) continue;
                if (above1_occ) continue;
                const GridIndex below1{c.x, c.y, c.z - 1};
                if (!isInsideMetricBounds(below1)) continue;
                if (!isOccupiedCell(below1)) {
                    my_pb.insert(c);
                }

                const int done = processed.fetch_add(1) + 1;
                const int pct = done * 100 / static_cast<int>(cand_total);
                if (pct >= next_log_pct) {
#pragma omp critical
                    if (pct >= next_log_pct) {
                        size_t total_pb = 0;
                        for (const auto & lp : local_preblocked) total_pb += lp.size();
                        std::cout << "  Preblocked cells: filtering " << pct << "% ("
                                  << done << " / " << cand_total << " candidates, "
                                  << total_pb << " preblocked, "
                                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC))
                                  << " s)" << std::endl;
                        next_log_pct = pct + 10;
                        if (next_log_pct > 100) next_log_pct = 100;
                    }
                }
            }
        }

        // 合并线程局部结果
        for (auto & lp : local_preblocked) {
            for (const auto & c : lp) preblocked_cells_.insert(c);
        }

        // === Phase 4: 合并外部预遮挡体素 ===
        for (const auto & c : external_preblocked_cells_) {
            if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
                preblocked_cells_.insert(c);
            }
        }

        std::cout << "  Preblocked cells: done ("
                  << preblocked_cells_.size() << " preblocked, "
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;
        // publishPreblockedCellsMarker();
    }

    bool GlobalPlanner::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            const GridIndex n_above1{n.x, n.y, n.z + 1};
            if (!isInsideMetricBounds(n_above1)) {
            continue;
            }
            if (isOccupiedCell(n_above1)) {
            return true;
            }
        }
        }
        return false;
    }

    bool GlobalPlanner::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            if (!isOccupiedCell(n)) {
            return true;
            }
        }
        }
        return false;
    }

    void GlobalPlanner::rebuildDerivedLayers()
    {
        std::cout << "GlobalPlanner::rebuildDerivedLayers rebuilding derived layers... " << std::endl;
        traversable_cells_.clear();
        if (!octree_) {
        return;
        }

        const bool require_ground_support = require_ground_support_;
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells = ground_support_xy_radius_cells_;  
        const int support_depth_cells = ground_support_depth_cells_;
        const double robot_radius = robot_radius_;
        const bool lowest_traversable_only = lowest_traversable_only_;

        const clock_t build_start = clock();

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
        const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

        // 预计算所有 (x,y) 列对
        struct XY { int x, y; };
        std::vector<XY> xy_pairs;
        xy_pairs.reserve(static_cast<size_t>(max_idx.x - min_idx.x + 1)
                       * static_cast<size_t>(max_idx.y - min_idx.y + 1));
        for (int x = min_idx.x; x <= max_idx.x; ++x) {
            for (int y = min_idx.y; y <= max_idx.y; ++y) {
                xy_pairs.push_back({x, y});
            }
        }

        const int64_t total_xy = static_cast<int64_t>(xy_pairs.size());
        const int64_t total_voxels = total_xy
            * (static_cast<int64_t>(max_idx.z) - min_idx.z + 1);
        std::atomic<int64_t> processed{0};
        int next_log_pct = 10;

        const int nt = (num_threads_ > 0) ? num_threads_ : 0;

        std::cout << "  Traversable layer: scanning " << static_cast<long long>(total_xy) << " (x,y) columns (" << static_cast<long long>(total_voxels) << " voxels total), " << (nt > 0 ? nt : omp_get_max_threads()) << " thread(s)..." << std::endl;

#pragma omp parallel for num_threads(nt) schedule(dynamic, 64) if(nt != 1)
        for (int64_t i = 0; i < total_xy; ++i) {
            const int x = xy_pairs[static_cast<size_t>(i)].x;
            const int y = xy_pairs[static_cast<size_t>(i)].y;

            std::vector<GridIndex> local_results;

            // 先用 occupancy cache 快速定位第一个非占据 z 层（即地面以上），
            // 跳过大段地下岩层，避免对每个 z 都做昂贵的 isCellTraversable。
            int first_free_z = max_idx.z + 1;
            for (int z = min_idx.z; z <= max_idx.z; ++z) {
                if (!isOccupiedCell(GridIndex{x, y, z})) {
                    first_free_z = z;
                    break;
                }
            }

            // 从第一个 free cell 开始检查 traversability
            for (int z = first_free_z; z <= max_idx.z; ++z) {
                const GridIndex idx{x, y, z};
                if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
                    continue;
                }
                if (isCellTraversable(
                    idx, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                    local_results.push_back(idx);
                    if (lowest_traversable_only) {
                        break;
                    }
                }
            }

            if (!local_results.empty()) {
#pragma omp critical
                for (const auto & g : local_results) {
                    traversable_cells_.insert(g);
                }
            }

            // 进度日志（仅主线程）
            const int64_t done = processed.fetch_add(1) + 1;
            const int pct = static_cast<int>(done * 100LL / total_xy);
            if (pct >= next_log_pct) {
#pragma omp critical
                if (pct >= next_log_pct) {
                    std::cout << "  Traversable layer: " << pct << "% (" << static_cast<long long>(done) << " / " << static_cast<long long>(total_xy) << " columns, " << traversable_cells_.size() << " traversable, " << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;
                    next_log_pct = pct + 10;
                    if (next_log_pct > 100) next_log_pct = 100;
                }
            }
        }

        std::cout << "  Traversable layer: 100% (" << static_cast<long long>(total_xy) << " / " << static_cast<long long>(total_xy) << " columns, " << traversable_cells_.size() << " traversable, " << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;

        // publishCellSetMarker(
        // traversable_cells_, traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F,
        // 0.55F);
    }

    void GlobalPlanner::buildTraversableGrid() {
        if (!octree_) {
            return;
        }
        const clock_t build_start = clock();

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        grid_min_ = worldToGrid(min_x, min_y, min_z);
        grid_max_ = worldToGrid(max_x, max_y, max_z);
        grid_dim_x_ = static_cast<int64_t>(grid_max_.x - grid_min_.x + 1);
        grid_dim_y_ = static_cast<int64_t>(grid_max_.y - grid_min_.y + 1);
        grid_dim_z_ = static_cast<int64_t>(grid_max_.z - grid_min_.z + 1);

        const int64_t grid_size = grid_dim_x_ * grid_dim_y_ * grid_dim_z_;
        if (grid_size <= 0 || grid_size > 2000000000LL) {
            std::cout << "  Traversable grid: grid too large (" << grid_size << "), skipping." << std::endl;
            return;
        }

        traversable_grid_.resize(grid_size, false);
        preblocked_cost_grid_.resize(grid_size, 0.0);

        for (const auto & c : traversable_cells_) {
            const int64_t lin = gridLinear(c);
            if (lin >= 0 && lin < grid_size) {
                traversable_grid_[lin] = true;
            }
        }

        for (const auto & entry : preblocked_costmap_) {
            const int64_t lin = gridLinear(entry.first);
            if (lin >= 0 && lin < grid_size) {
                preblocked_cost_grid_[lin] = entry.second;
            }
        }

        // 让代价场成为完整的“障碍接近度”场：非可通行格（障碍/膨胀壳）设为最高代价 1.0。
        // 此前这些格 cost=0（未赋值），会导致边界处 −∇cost 指向障碍（反方向）。
        // 修正后：障碍=1.0、近障碍可通行=[0.25,0.75]、远障碍=0，−∇cost 始终远离障碍，
        // 可供平滑优化直接做梯度下降，无需逐次迭代碰撞检查。
        // 说明：A* 仅在 isTraversableGrid(nbr) 为真时查询 cost（见 startPlan），故不受影响。
        const int nt_bt = (num_threads_ > 0) ? num_threads_ : 0;
#pragma omp parallel for num_threads(nt_bt) schedule(static) if(nt_bt != 1)
        for (int64_t lin = 0; lin < grid_size; ++lin) {
            if (!traversable_grid_[lin]) {
                preblocked_cost_grid_[lin] = 1.0;
            }
        }

        std::cout << "  Traversable grid: built " << grid_dim_x_ << "x" << grid_dim_y_ << "x" << grid_dim_z_
                  << " = " << grid_size << " cells in "
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s" << std::endl;
    }

    bool GlobalPlanner::isInsideMetricBounds(const GridIndex & idx) const
    {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const auto p = gridToWorld(idx);
        return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
            p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
            p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
    }

    bool GlobalPlanner::isOccupiedCell(const GridIndex & idx) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }
        // 若已有占用缓存，使用 O(1) 哈希查询
        if (!occupancy_cache_.empty()) {
            return occupancy_cache_.count(idx) > 0;
        }
        // 退化到 octree 树搜索
        const auto p = gridToWorld(idx);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
    }

    void GlobalPlanner::buildOccupancyCache()
    {
        occupancy_cache_.clear();
        if (!octree_) {
            return;
        }
        const clock_t build_start = clock();

        size_t occupied_leaves = 0, expanded_cells = 0;

        const double res = octree_->getResolution();
        const double eps = res * 1e-6;

        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (!octree_->isNodeOccupied(*it)) continue;
            ++occupied_leaves;

            double size = it.getSize();
            if (size <= res * 1.01) {
                // 细叶子：直接映射单个格子
                GridIndex gi = worldToGrid(it.getX(), it.getY(), it.getZ());
                occupancy_cache_.insert(gi);
            } else {
                // 粗叶子：展开为全部被覆盖的细格子
                double half = size * 0.5;
                double cx = it.getX(), cy = it.getY(), cz = it.getZ();
                GridIndex lo = worldToGrid(cx - half + eps, cy - half + eps, cz - half + eps);
                GridIndex hi = worldToGrid(cx + half - eps, cy + half - eps, cz + half - eps);
                for (int gx = lo.x; gx <= hi.x; ++gx) {
                    for (int gy = lo.y; gy <= hi.y; ++gy) {
                        for (int gz = lo.z; gz <= hi.z; ++gz) {
                            GridIndex gi{gx, gy, gz};
                            occupancy_cache_.insert(gi);
                            ++expanded_cells;
                        }
                    }
                }
            }
        }

        std::cout << "  Occupancy cache: " << occupied_leaves << " occupied leaves, "
                  << occupancy_cache_.size() << " grid cells";
        if (expanded_cells > 0) {
            std::cout << " (" << expanded_cells << " from coarse leaf expansion)";
        }
        std::cout << " in "
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC))
                  << " s" << std::endl;
    }

    GridIndex GlobalPlanner::worldToGrid(double x, double y, double z) const
    {
        const double r = octree_->getResolution();
        return GridIndex{
        static_cast<int>(std::floor(x / r)),
        static_cast<int>(std::floor(y / r)),
        static_cast<int>(std::floor(z / r))};
    }

    octomap::point3d GlobalPlanner::gridToWorld(const GridIndex & idx) const
    {
        const double r = octree_->getResolution();
        return octomap::point3d(
        static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
    }

    bool GlobalPlanner::savePlanningCache(const std::string & cache_path) const
    {
        // 格式: [4B version=2]
        //       [8B resolution][8B robot_radius]
        //       [1B require_ground_support][1B strict_direct_ground_support]
        //       [4B support_xy_radius_cells][4B support_depth_cells]
        //       [1B lowest_traversable_only][1B enable_preblocked_costmap]
        //       [4B preblocked_costmap_radius_cells][8B preblocked_costmap_weight]
        //       [8B count_traversable][count_traversable × GridIndex]
        //       [8B count_preblocked][count_preblocked × GridIndex]
        //       [8B count_costmap][count_costmap × (GridIndex + double)]
        std::ofstream f(cache_path, std::ios::binary);
        if (!f.is_open()) {
            std::cout << "GlobalPlanner: cannot write planning cache " << cache_path << std::endl;
            return false;
        }

        const uint32_t version = 2;
        f.write(reinterpret_cast<const char *>(&version), sizeof(version));

        // 参数校验头
        const double resolution = octree_ ? octree_->getResolution() : 0.0;
        const double r_radius = robot_radius_;
        const uint8_t u_require_gs = require_ground_support_ ? 1 : 0;
        const uint8_t u_strict = strict_direct_ground_support_ ? 1 : 0;
        const int32_t i_support_xy = static_cast<int32_t>(ground_support_xy_radius_cells_);
        const int32_t i_support_d = static_cast<int32_t>(ground_support_depth_cells_);
        const uint8_t u_lowest = lowest_traversable_only_ ? 1 : 0;
        const uint8_t u_enable_pc = enable_preblocked_costmap_ ? 1 : 0;
        const int32_t i_pc_radius = static_cast<int32_t>(preblocked_costmap_radius_cells_);
        const double d_pc_weight = preblocked_costmap_weight_;

        f.write(reinterpret_cast<const char *>(&resolution), sizeof(resolution));
        f.write(reinterpret_cast<const char *>(&r_radius), sizeof(r_radius));
        f.write(reinterpret_cast<const char *>(&u_require_gs), sizeof(u_require_gs));
        f.write(reinterpret_cast<const char *>(&u_strict), sizeof(u_strict));
        f.write(reinterpret_cast<const char *>(&i_support_xy), sizeof(i_support_xy));
        f.write(reinterpret_cast<const char *>(&i_support_d), sizeof(i_support_d));
        f.write(reinterpret_cast<const char *>(&u_lowest), sizeof(u_lowest));
        f.write(reinterpret_cast<const char *>(&u_enable_pc), sizeof(u_enable_pc));
        f.write(reinterpret_cast<const char *>(&i_pc_radius), sizeof(i_pc_radius));
        f.write(reinterpret_cast<const char *>(&d_pc_weight), sizeof(d_pc_weight));

        uint64_t count;

        count = static_cast<uint64_t>(traversable_cells_.size());
        f.write(reinterpret_cast<const char *>(&count), sizeof(count));
        for (const auto & c : traversable_cells_) {
            f.write(reinterpret_cast<const char *>(&c), sizeof(c));
        }

        count = static_cast<uint64_t>(preblocked_cells_.size());
        f.write(reinterpret_cast<const char *>(&count), sizeof(count));
        for (const auto & c : preblocked_cells_) {
            f.write(reinterpret_cast<const char *>(&c), sizeof(c));
        }

        count = static_cast<uint64_t>(preblocked_costmap_.size());
        f.write(reinterpret_cast<const char *>(&count), sizeof(count));
        for (const auto & entry : preblocked_costmap_) {
            f.write(reinterpret_cast<const char *>(&entry.first), sizeof(entry.first));
            f.write(reinterpret_cast<const char *>(&entry.second), sizeof(entry.second));
        }

        std::cout << "GlobalPlanner: saved planning cache (" << traversable_cells_.size() << " traversable, " << preblocked_cells_.size() << " preblocked, " << preblocked_costmap_.size() << " costmap) to " << cache_path << std::endl;
        return true;
    }

    bool GlobalPlanner::loadPlanningCache(const std::string & cache_path)
    {
        std::ifstream f(cache_path, std::ios::binary);
        if (!f.is_open()) {
            return false;
        }

        uint32_t version = 0;
        f.read(reinterpret_cast<char *>(&version), sizeof(version));
        if (!f || version == 0) {
            return false;
        }

        // 参数校验（v2+）
        if (version >= 2) {
            double stored_resolution = 0.0, stored_robot_radius = 0.0;
            uint8_t stored_require_gs = 0, stored_strict = 0;
            int32_t stored_support_xy = 0, stored_support_d = 0;
            uint8_t stored_lowest = 0, stored_enable_pc = 0;
            int32_t stored_pc_radius = 0;
            double stored_pc_weight = 0.0;

            f.read(reinterpret_cast<char *>(&stored_resolution), sizeof(stored_resolution));
            f.read(reinterpret_cast<char *>(&stored_robot_radius), sizeof(stored_robot_radius));
            f.read(reinterpret_cast<char *>(&stored_require_gs), sizeof(stored_require_gs));
            f.read(reinterpret_cast<char *>(&stored_strict), sizeof(stored_strict));
            f.read(reinterpret_cast<char *>(&stored_support_xy), sizeof(stored_support_xy));
            f.read(reinterpret_cast<char *>(&stored_support_d), sizeof(stored_support_d));
            f.read(reinterpret_cast<char *>(&stored_lowest), sizeof(stored_lowest));
            f.read(reinterpret_cast<char *>(&stored_enable_pc), sizeof(stored_enable_pc));
            f.read(reinterpret_cast<char *>(&stored_pc_radius), sizeof(stored_pc_radius));
            f.read(reinterpret_cast<char *>(&stored_pc_weight), sizeof(stored_pc_weight));
            if (!f) return false;

            const double cur_resolution = octree_ ? octree_->getResolution() : 0.0;
            const auto mismatch = [&](const char * name, auto stored, auto current) {
                std::cout << "GlobalPlanner: " << name << " changed (" << std::to_string(stored) << " vs " << std::to_string(current) << "), rebuilding planning cache." << std::endl;
            };

            bool ok = true;
            if (std::abs(stored_resolution - cur_resolution) > 1e-6) {
                mismatch("resolution", stored_resolution, cur_resolution); ok = false;
            }
            if (std::abs(stored_robot_radius - robot_radius_) > 1e-6) {
                mismatch("robot_radius", stored_robot_radius, robot_radius_); ok = false;
            }
            if (static_cast<bool>(stored_require_gs) != require_ground_support_) {
                mismatch("require_ground_support", static_cast<int>(stored_require_gs),
                         static_cast<int>(require_ground_support_)); ok = false;
            }
            if (static_cast<bool>(stored_strict) != strict_direct_ground_support_) {
                mismatch("strict_direct_ground_support", static_cast<int>(stored_strict),
                         static_cast<int>(strict_direct_ground_support_)); ok = false;
            }
            if (stored_support_xy != static_cast<int32_t>(ground_support_xy_radius_cells_)) {
                mismatch("ground_support_xy_radius_cells", stored_support_xy,
                         static_cast<int32_t>(ground_support_xy_radius_cells_)); ok = false;
            }
            if (stored_support_d != static_cast<int32_t>(ground_support_depth_cells_)) {
                mismatch("ground_support_depth_cells", stored_support_d,
                         static_cast<int32_t>(ground_support_depth_cells_)); ok = false;
            }
            if (static_cast<bool>(stored_lowest) != lowest_traversable_only_) {
                mismatch("lowest_traversable_only", static_cast<int>(stored_lowest),
                         static_cast<int>(lowest_traversable_only_)); ok = false;
            }
            if (static_cast<bool>(stored_enable_pc) != enable_preblocked_costmap_) {
                mismatch("enable_preblocked_costmap", static_cast<int>(stored_enable_pc),
                         static_cast<int>(enable_preblocked_costmap_)); ok = false;
            }
            if (stored_pc_radius != static_cast<int32_t>(preblocked_costmap_radius_cells_)) {
                mismatch("preblocked_costmap_radius_cells", stored_pc_radius,
                         static_cast<int32_t>(preblocked_costmap_radius_cells_)); ok = false;
            }
            if (std::abs(stored_pc_weight - preblocked_costmap_weight_) > 1e-6) {
                mismatch("preblocked_costmap_weight", stored_pc_weight,
                         preblocked_costmap_weight_); ok = false;
            }
            if (!ok) return false;
        }

        uint64_t count;
        traversable_cells_.clear();
        preblocked_cells_.clear();
        preblocked_costmap_.clear();

        // traversable_cells_
        f.read(reinterpret_cast<char *>(&count), sizeof(count));
        if (!f) return false;
        traversable_cells_.reserve(static_cast<size_t>(count));
        for (uint64_t i = 0; i < count; ++i) {
            GridIndex idx;
            f.read(reinterpret_cast<char *>(&idx), sizeof(idx));
            if (!f) return false;
            traversable_cells_.insert(idx);
        }

        // preblocked_cells_
        f.read(reinterpret_cast<char *>(&count), sizeof(count));
        if (!f) return false;
        preblocked_cells_.reserve(static_cast<size_t>(count));
        for (uint64_t i = 0; i < count; ++i) {
            GridIndex idx;
            f.read(reinterpret_cast<char *>(&idx), sizeof(idx));
            if (!f) return false;
            preblocked_cells_.insert(idx);
        }

        // preblocked_costmap_
        f.read(reinterpret_cast<char *>(&count), sizeof(count));
        if (!f) return false;
        preblocked_costmap_.reserve(static_cast<size_t>(count));
        for (uint64_t i = 0; i < count; ++i) {
            GridIndex idx;
            double val;
            f.read(reinterpret_cast<char *>(&idx), sizeof(idx));
            if (!f) return false;
            f.read(reinterpret_cast<char *>(&val), sizeof(val));
            if (!f) return false;
            preblocked_costmap_[idx] = val;
        }

        std::cout << "GlobalPlanner: loaded planning cache (" << traversable_cells_.size() << " traversable, " << preblocked_cells_.size() << " preblocked, " << preblocked_costmap_.size() << " costmap) from " << cache_path << std::endl;
        return true;
    }

    void GlobalPlanner::setOctomapWithCache(
        std::shared_ptr<octomap::OcTree> map,
        const std::string & cache_path)
    {
        if (!map)
        {
            std::cout << "Octomap is Null!!! return." << std::endl;
            return;
        }

        if (octree_ == map)
        {
            std::cout << "Octomap is No Update!!! return." << std::endl;
            return;
        }

        octree_ = map;
        map_ready_ = true;

        if (loadPlanningCache(cache_path)) {
            std::cout << "GlobalPlanner: using cached planning layers." << std::endl;
            buildTraversableGrid();
            return;
        }

        std::cout << "GlobalPlanner: planning cache not found, rebuilding..." << std::endl;
        buildOccupancyCache();
        rebuildPreblockedCells();
        rebuildDerivedLayers();
        rebuildPreblockedCostmap();
        savePlanningCache(cache_path);
        buildTraversableGrid();
    }

    void GlobalPlanner::rebuildPreblockedCostmap()
    {
        preblocked_costmap_.clear();
        if (!octree_) {
        return;
        }
        const bool enable = enable_preblocked_costmap_;
        if (!enable) {
        return;
        }

        const int radius_cells = std::max(
        1, static_cast<int>(preblocked_costmap_radius_cells_));
        const double denom = static_cast<double>(radius_cells) + 1.0;

        const clock_t build_start = clock();
        const size_t total_cells = preblocked_cells_.size();
        const int nt = (num_threads_ > 0) ? num_threads_ : 0;
        const int num_threads = nt > 0 ? nt : omp_get_max_threads();
        std::cout << "  Preblocked costmap: spreading " << total_cells << " cells, radius " << radius_cells << ", " << num_threads << " thread(s)..." << std::endl;

        // 将 preblocked_cells_ 转为 vector 以便 OpenMP 随机访问
        std::vector<GridIndex> pb_vec(preblocked_cells_.begin(), preblocked_cells_.end());

        // === 构建密集三维布尔网格（O(1) 无哈希查找，替代 unordered_set） ===
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const GridIndex gmin = worldToGrid(min_x, min_y, min_z);
        const GridIndex gmax = worldToGrid(max_x, max_y, max_z);
        const int64_t dim_x = static_cast<int64_t>(gmax.x - gmin.x + 1);
        const int64_t dim_y = static_cast<int64_t>(gmax.y - gmin.y + 1);
        const int64_t dim_z = static_cast<int64_t>(gmax.z - gmin.z + 1);
        const int64_t grid_size = dim_x * dim_y * dim_z;

        if (grid_size <= 0 || grid_size > 2000000000LL) {
            std::cout << "  Preblocked costmap: grid too large (" << grid_size << "), aborting." << std::endl;
            return;
        }

        auto grid_linear = [&](const GridIndex & idx) -> int64_t {
            return (static_cast<int64_t>(idx.x - gmin.x) * dim_y +
                    static_cast<int64_t>(idx.y - gmin.y)) * dim_z +
                    static_cast<int64_t>(idx.z - gmin.z);
        };

        std::vector<bool> traversable_grid(grid_size, false);
        std::vector<bool> preblocked_grid(grid_size, false);
        for (const auto & c : traversable_cells_) {
            if (isInsideMetricBounds(c)) {
                traversable_grid[grid_linear(c)] = true;
            }
        }
        for (const auto & c : preblocked_cells_) {
            if (isInsideMetricBounds(c)) {
                preblocked_grid[grid_linear(c)] = true;
            }
        }
        std::cout << "  Preblocked costmap: dense grid built (" << dim_x << "x" << dim_y << "x" << dim_z
                  << " = " << grid_size << " cells) in "
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s" << std::endl;

        // 预计算邻居偏移（球体内的整数偏移）
        std::vector<GridIndex> neighbor_offsets;
        neighbor_offsets.reserve(static_cast<size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1) * (2 * radius_cells + 1)));
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    const double d = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
                    if (d <= static_cast<double>(radius_cells)) {
                        neighbor_offsets.push_back({dx, dy, dz});
                    }
                }
            }
        }

        // 线程局部 costmap 向量
        std::vector<std::unordered_map<GridIndex, double, GridIndexHash>> local_costmaps(num_threads);

        std::atomic<int64_t> processed{0};
        int next_log_pct = 10;

#pragma omp parallel num_threads(nt) if(nt != 1)
        {
            const int tid = omp_get_thread_num();
            auto & my_costmap = local_costmaps[tid];
            my_costmap.reserve(total_cells / num_threads);

#pragma omp for schedule(dynamic, 32)
            for (int64_t i = 0; i < static_cast<int64_t>(pb_vec.size()); ++i) {
                const auto & c = pb_vec[static_cast<size_t>(i)];
                const int64_t base_lin = grid_linear(c);

                for (const auto & off : neighbor_offsets) {
                    const GridIndex n{c.x + off.x, c.y + off.y, c.z + off.z};
                    if (!isInsideMetricBounds(n)) {
                        continue;
                    }
                    const int64_t n_lin = base_lin +
                        off.x * dim_y * dim_z +
                        off.y * dim_z +
                        off.z;
                    if (!traversable_grid[n_lin]) {
                        continue;
                    }
                    if (preblocked_grid[n_lin]) {
                        continue;
                    }
                    const double d = std::sqrt(
                        static_cast<double>(off.x * off.x + off.y * off.y + off.z * off.z));
                    const double cst = std::max(0.0, (denom - d) / denom);
                    auto it = my_costmap.find(n);
                    if (it == my_costmap.end() || cst > it->second) {
                        my_costmap[n] = cst;
                    }
                }

                const int64_t done = processed.fetch_add(1) + 1;
                const int pct = static_cast<int>(done * 100LL / static_cast<int64_t>(total_cells));
                if (pct >= next_log_pct) {
#pragma omp critical
                    if (pct >= next_log_pct) {
                        size_t total_local = 0;
                        for (const auto & lc : local_costmaps) {
                            total_local += lc.size();
                        }
                        std::cout << "  Preblocked costmap: " << pct << "% ("
                                  << static_cast<long long>(done) << " / " << total_cells
                                  << " cells, " << total_local << " costmap, "
                                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC))
                                  << " s)" << std::endl;
                        next_log_pct = pct + 10;
                        if (next_log_pct > 100) next_log_pct = 100;
                    }
                }
            }
        }

        // 合并局部结果到全局 costmap（取最大值）
        const clock_t merge_start = clock();
        for (const auto & lc : local_costmaps) {
            for (const auto & entry : lc) {
                auto it = preblocked_costmap_.find(entry.first);
                if (it == preblocked_costmap_.end() || entry.second > it->second) {
                    preblocked_costmap_[entry.first] = entry.second;
                }
            }
        }
        std::cout << "  Preblocked costmap: merged " << preblocked_costmap_.size()
                  << " entries in " << ((clock() - merge_start) / static_cast<double>(CLOCKS_PER_SEC))
                  << " s" << std::endl;

        std::cout << "  Preblocked costmap: 100% done ("
                  << preblocked_costmap_.size() << " costmap entries, "
                  << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;
    }

    // RCLCPP_INFO(
    // get_logger(),
    // "Preblocked costmap rebuilt. cells=%zu radius=%d",
    // preblocked_costmap_.size(), radius_cells);
    // std::cout << "Preblocked costmap rebuilt. cells=" << preblocked_costmap_.size() << " radius=" << radius_cells << " " << std::endl;
    // publishRiskCostCloud();


    // ============================================================================
    // Path Smoothing Implementation
    // ============================================================================

    void GlobalPlanner::smoothPath()
    {
        if (!smoothing_enabled_ || planner_results_.size() < 3) {
            return;
        }

        simplifyPath(planner_results_, smoothing_simplify_epsilon_);
        if (planner_results_.size() < 2) return;

        smoothZStairSteps(planner_results_, smoothing_z_window_radius_);

        constexpr size_t MIN_GRADIENT_POINTS = 8;
        if (planner_results_.size() >= MIN_GRADIENT_POINTS) {
            gradientDescentSmooth(
                planner_results_, smoothing_gradient_iters_,
                smoothing_gradient_alpha_, smoothing_cost_gradient_beta_);
        }

        std::vector<PointPose> interp;
        interpolatePath(planner_results_, interp, smoothing_interp_spacing_);
        planner_results_ = std::move(interp);
    }

    // ============================================================================
    // z 方向台阶平滑：消除 3D 栅格离散化导致的楼梯状路径
    // ============================================================================
    void GlobalPlanner::smoothZStairSteps(
        std::vector<PointPose> & path,
        int window_radius) const
    {
        if (path.size() < 3 || window_radius < 1) return;

        const double res = octree_ ? octree_->getResolution() : 0.5;
        std::vector<PointPose> smoothed = path;

        for (size_t i = 1; i + 1 < path.size(); ++i) {
            // 加权滑动平均：离 i 越近的邻居权重越大（三角权重）
            double z_sum = 0.0;
            double weight_sum = 0.0;
            for (int k = -window_radius; k <= window_radius; ++k) {
                int j = static_cast<int>(i) + k;
                if (j < 0 || j >= static_cast<int>(path.size())) continue;
                double w = window_radius + 1.0 - std::abs(k);  // 三角权重
                z_sum += w * path[j].z;
                weight_sum += w;
            }
            double z_smooth = z_sum / weight_sum;

            // 约束：z 位移不超过 ±res（保持在原栅格或相邻栅格，确保安全）
            const double z_min = path[i].z - res;
            const double z_max = path[i].z + res;
            smoothed[i].z = std::clamp(z_smooth, z_min, z_max);

            // 验证新 z 所在体素是否可通行
            const GridIndex gi = worldToGrid(smoothed[i].x, smoothed[i].y, smoothed[i].z);
            if (!isTraversableGrid(gi)) {
                smoothed[i].z = path[i].z;  // 回退
            }
        }

        // 逐段 DDA 验证，z 平滑后个别段可能穿障，不安全则对该段两点回退 z
        for (size_t i = 1; i < path.size(); ++i) {
            if (!isSegmentTraversable(smoothed[i - 1], smoothed[i])) {
                smoothed[i].z = path[i].z;
                smoothed[i - 1].z = path[i - 1].z;
            }
        }

        path = std::move(smoothed);
    }

    bool GlobalPlanner::isSegmentTraversable(const PointPose & a, const PointPose & b) const
    {
        if (!octree_) return true;
        const double res = octree_->getResolution();

        const GridIndex ia = worldToGrid(a.x, a.y, a.z);
        const GridIndex ib = worldToGrid(b.x, b.y, b.z);

        // 端点本身必须可通行（traversable_grid_ 已含 robot_radius 膨胀）
        if (!isTraversableGrid(ia) || !isTraversableGrid(ib)) {
            return false;
        }
        if (ia.x == ib.x && ia.y == ib.y && ia.z == ib.z) {
            return true;
        }

        // Amanatides-Woo 3D DDA：逐体素遍历线段穿过的所有栅格
        int x = ia.x, y = ia.y, z = ia.z;
        const int stepX = (ib.x > ia.x) ? 1 : (ib.x < ia.x ? -1 : 0);
        const int stepY = (ib.y > ia.y) ? 1 : (ib.y < ia.y ? -1 : 0);
        const int stepZ = (ib.z > ia.z) ? 1 : (ib.z < ia.z ? -1 : 0);

        const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        const double inf = std::numeric_limits<double>::infinity();

        double tMaxX, tMaxY, tMaxZ;
        if (stepX > 0)      tMaxX = (static_cast<double>(x + 1) * res - a.x) / dx;
        else if (stepX < 0) tMaxX = (static_cast<double>(x) * res - a.x) / dx;
        else                tMaxX = inf;
        if (stepY > 0)      tMaxY = (static_cast<double>(y + 1) * res - a.y) / dy;
        else if (stepY < 0) tMaxY = (static_cast<double>(y) * res - a.y) / dy;
        else                tMaxY = inf;
        if (stepZ > 0)      tMaxZ = (static_cast<double>(z + 1) * res - a.z) / dz;
        else if (stepZ < 0) tMaxZ = (static_cast<double>(z) * res - a.z) / dz;
        else                tMaxZ = inf;

        const double tDeltaX = (stepX != 0) ? res / std::fabs(dx) : inf;
        const double tDeltaY = (stepY != 0) ? res / std::fabs(dy) : inf;
        const double tDeltaZ = (stepZ != 0) ? res / std::fabs(dz) : inf;

        // 最大步数 = 各轴跨格数之和 + 2，防止极端浮点误差下死循环
        const int max_steps =
            std::abs(ib.x - ia.x) + std::abs(ib.y - ia.y) + std::abs(ib.z - ia.z) + 2;

        for (int step = 0; step < max_steps; ++step) {
            if (tMaxX < tMaxY) {
                if (tMaxX < tMaxZ) { x += stepX; tMaxX += tDeltaX; }
                else               { z += stepZ; tMaxZ += tDeltaZ; }
            } else {
                if (tMaxY < tMaxZ) { y += stepY; tMaxY += tDeltaY; }
                else               { z += stepZ; tMaxZ += tDeltaZ; }
            }
            if (!isTraversableGrid(GridIndex{x, y, z})) {
                return false;
            }
            if (x == ib.x && y == ib.y && z == ib.z) {
                break;
            }
        }
        return true;
    }

    double GlobalPlanner::costFieldAt(const GridIndex & idx) const
    {
        // 越界视为最高代价（保守，避免规划贴地图边缘）
        if (idx.x < grid_min_.x || idx.x > grid_max_.x ||
            idx.y < grid_min_.y || idx.y > grid_max_.y ||
            idx.z < grid_min_.z || idx.z > grid_max_.z) {
            return 1.0;
        }
        // buildTraversableGrid 已将非可通行格的 cost 置为 1.0，
        // 可通行格返回其预阻塞代价（远障碍=0，近障碍=[0.25,0.75]）
        return preblocked_cost_grid_[gridLinear(idx)];
    }

    void GlobalPlanner::getCostFieldCloud(
        std::vector<PointPose> & out_positions,
        std::vector<double> & out_costs) const
    {
        out_positions.clear();
        out_costs.clear();
        if (!octree_ || preblocked_cost_grid_.empty()) {
            std::cout << "getCostFieldCloud: octree=" << (octree_ ? "ok" : "null")
                      << " grid_size=" << preblocked_cost_grid_.size() << std::endl;
            return;
        }

        const int64_t size = static_cast<int64_t>(preblocked_cost_grid_.size());
        const double res = octree_->getResolution();

        // 第一遍：找到最高可通行层的z坐标
        double max_traversable_z = -1e9;
        for (int64_t lin = 0; lin < size; ++lin) {
            if (preblocked_cost_grid_[lin] < 0.999) {
                const int64_t z_off = lin % grid_dim_z_;
                const double wz = (z_off + grid_min_.z + 0.5) * res;
                if (wz > max_traversable_z) max_traversable_z = wz;
            }
        }
        const double z_cutoff = max_traversable_z - 1.0;

        std::vector<PointPose> pos;
        std::vector<double> cost;
        pos.reserve(size);
        cost.reserve(size);

        int64_t total = 0;
        for (int64_t lin = 0; lin < size; ++lin) {
            const double c = preblocked_cost_grid_[lin];
            if (c >= 0.999) continue;  // 排除障碍(c=1.0)，已在occupied_map_cloud中可见

            const int64_t z_off = lin % grid_dim_z_;
            const double wz = (z_off + grid_min_.z + 0.5) * res;
            if (wz > z_cutoff) continue;  // 排除高于可通行层1m的屋顶区域

            ++total;

            // 从线性索引反求栅格坐标
            const int64_t rem = lin / grid_dim_z_;
            const int64_t y_off = rem % grid_dim_y_;
            const int64_t x_off = rem / grid_dim_y_;
            const GridIndex idx{
                static_cast<int>(x_off + grid_min_.x),
                static_cast<int>(y_off + grid_min_.y),
                static_cast<int>(z_off + grid_min_.z)};
            const auto p = gridToWorld(idx);
            pos.push_back(PointPose{p.x(), p.y(), p.z()});
            cost.push_back(c);
        }

        out_positions = std::move(pos);
        out_costs = std::move(cost);
        std::cout << "getCostFieldCloud: grid_size=" << size
                  << " max_tz=" << max_traversable_z
                  << " z_cutoff=" << z_cutoff
                  << " included=" << out_positions.size() << std::endl;
    }

    void GlobalPlanner::simplifyPath(std::vector<PointPose> & path, double epsilon) const
    {
        if (path.size() < 3) return;
        const double eps_sq = epsilon * epsilon;

        std::vector<PointPose> result;
        result.reserve(path.size());
        result.push_back(path.front());

        for (size_t i = 1; i + 1 < path.size(); ++i) {
            const auto & prev = result.back();  // 上一个被保留的点
            const auto & cur  = path[i];
            const auto & next = path[i + 1];

            // 计算 cur 到直线 prev->next 的距离平方
            const double dx = next.x - prev.x;
            const double dy = next.y - prev.y;
            const double dz = next.z - prev.z;
            const double len_sq = dx * dx + dy * dy + dz * dz;
            double dist_sq;
            if (len_sq < 1e-12) {
                dist_sq = (cur.x - prev.x) * (cur.x - prev.x) +
                          (cur.y - prev.y) * (cur.y - prev.y) +
                          (cur.z - prev.z) * (cur.z - prev.z);
            } else {
                const double t = ((cur.x - prev.x) * dx + (cur.y - prev.y) * dy +
                                  (cur.z - prev.z) * dz) / len_sq;
                const double tc = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
                const double proj_x = prev.x + tc * dx;
                const double proj_y = prev.y + tc * dy;
                const double proj_z = prev.z + tc * dz;
                dist_sq = (cur.x - proj_x) * (cur.x - proj_x) +
                          (cur.y - proj_y) * (cur.y - proj_y) +
                          (cur.z - proj_z) * (cur.z - proj_z);
            }

            // 仅当 cur 近似共线（dist <= epsilon）且跳过 cur 后的弦 prev->next
            // 仍可通行时，才跳过 cur。这保证简化后的每段都无碰撞，避免切角穿障。
            if (dist_sq <= eps_sq && isSegmentTraversable(prev, next)) {
                continue;
            }
            result.push_back(cur);
        }
        result.push_back(path.back());

        if (result.size() < path.size()) {
            path = std::move(result);
        }
    }

    void GlobalPlanner::interpolatePath(
        const std::vector<PointPose> & input,
        std::vector<PointPose> & output,
        double spacing) const
    {
        output.clear();
        if (input.size() < 2) {
            if (input.size() == 1) output = input;
            return;
        }

        const double res = octree_ ? octree_->getResolution() : 0.5;
        const double step = std::max(spacing, res * 0.1);

        // 辅助：两点间距离
        auto chord_len = [](const PointPose & a, const PointPose & b) -> double {
            const double dx = a.x - b.x;
            const double dy = a.y - b.y;
            const double dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        };

        // 辅助：centripetal Catmull-Rom 插值（alpha=0.5，减少 overshoot）
        // 使用 Barry-Goldman 递归求值，支持非均匀参数化
        auto centripetal_catmull_rom = [&chord_len](
            const PointPose & p0, const PointPose & p1,
            const PointPose & p2, const PointPose & p3, double t) -> PointPose {
            const double alpha = 0.5; // centripetal
            double t0 = 0.0;
            double t1 = t0 + std::pow(chord_len(p0, p1), alpha);
            double t2 = t1 + std::pow(chord_len(p1, p2), alpha);
            double t3 = t2 + std::pow(chord_len(p2, p3), alpha);
            // 避免除零
            if (t2 - t1 < 1e-12) {
                return p1;
            }
            const double u = t1 + (t2 - t1) * t; // 映射到 [t1, t2]

            auto lerp = [](const PointPose & a, const PointPose & b,
                           double ta, double tb, double t_cur) -> PointPose {
                if (std::abs(tb - ta) < 1e-12) return a;
                const double w = (t_cur - ta) / (tb - ta);
                PointPose r;
                r.x = a.x + (b.x - a.x) * w;
                r.y = a.y + (b.y - a.y) * w;
                r.z = a.z + (b.z - a.z) * w;
                return r;
            };

            const PointPose A1 = lerp(p0, p1, t0, t1, u);
            const PointPose A2 = lerp(p1, p2, t1, t2, u);
            const PointPose A3 = lerp(p2, p3, t2, t3, u);
            const PointPose B1 = lerp(A1, A2, t0, t2, u);
            const PointPose B2 = lerp(A2, A3, t1, t3, u);
            return lerp(B1, B2, t1, t2, u);
        };

        output.reserve(static_cast<size_t>(input.size() * 4));
        output.push_back(input.front());

        for (size_t i = 0; i + 1 < input.size(); ++i) {
            const auto & p0 = (i == 0) ? input[0] : input[i - 1];
            const auto & p1 = input[i];
            const auto & p2 = input[i + 1];
            const auto & p3 = (i + 2 < input.size()) ? input[i + 2] : input[i + 1];

            const double seg_len = chord_len(p1, p2);
            const int num_steps = std::max(1, static_cast<int>(seg_len / step));
            const double inv_steps = 1.0 / static_cast<double>(num_steps);

            // 先用 Catmull-Rom 生成候选点
            std::vector<PointPose> cand;
            cand.reserve(num_steps);
            for (int j = 1; j <= num_steps; ++j) {
                const double t = static_cast<double>(j) * inv_steps;
                cand.push_back(centripetal_catmull_rom(p0, p1, p2, p3, t));
            }

            // 逐段碰撞检查：Catmull-Rom 在折角处可能外凸离开安全走廊，
            // 若任一子段穿障，则该段整体回退到线性插值。
            // 线性插值沿 p1->p2，而该线段已由平滑阶段保证可通行，故必然安全。
            bool catmull_safe = true;
            PointPose prev = p1;
            for (const auto & c : cand) {
                if (!isSegmentTraversable(prev, c)) {
                    catmull_safe = false;
                    break;
                }
                prev = c;
            }

            if (catmull_safe) {
                for (const auto & c : cand) output.push_back(c);
            } else {
                for (int j = 1; j <= num_steps; ++j) {
                    const double t = static_cast<double>(j) * inv_steps;
                    PointPose pt;
                    pt.x = p1.x + (p2.x - p1.x) * t;
                    pt.y = p1.y + (p2.y - p1.y) * t;
                    pt.z = p1.z + (p2.z - p1.z) * t;
                    output.push_back(pt);
                }
            }
        }
    }

    void GlobalPlanner::gradientDescentSmooth(
        std::vector<PointPose> & path,
        int max_iters,
        double alpha,
        double beta) const
    {
        if (path.size() < 3) return;

        const double res = octree_ ? octree_->getResolution() : 0.5;
        // 每步位移上限，防止拐角处大幅跳跃（0=auto=半栅格）
        const double max_step = (smoothing_max_step_ > 0.0) ? smoothing_max_step_ : (res * 0.5);
        // 代价门控容差：允许沿等代价线微调（容许小幅 uphill 以便拐角圆化），
        // 但禁止明显向障碍靠近，从源头阻止拉普拉斯把点拉向墙
        const double cost_tol = smoothing_cost_tolerance_;

        // 保留简化后路径用于末尾安全回退（简化阶段已保证每段可通行）
        const std::vector<PointPose> original = path;

        for (int iter = 0; iter < max_iters; ++iter) {
            // 迭代衰减：sqrt(1−t) 衰减曲线比线性更平缓
            const double t = static_cast<double>(iter) / max_iters;
            const double decay = std::sqrt(1.0 - t);
            const double alpha_eff = alpha * decay;

            // 两阶段策略：
            //   前半迭代 → 纯 Laplace 平滑，无视 cost，自由拉平台阶和折角
            //   后半迭代 → 加入 xy cost 梯度排斥，推开墙壁微调（z 方向梯度始终禁用）
            const bool use_cost = (iter >= max_iters / 2);
            const double beta_eff = use_cost ? (beta * decay) : 0.0;

            double max_delta = 0.0;

            // Gauss-Seidel 逐点原地更新：
            //   处理 path[i] 时 path[i-1] 已是本轮新位置，DDA 检查更准确；
            //   path[i+1] 仍为旧值，但在下一轮处理 i+1 时会用新 path[i] 重新校验。
            for (size_t i = 1; i + 1 < path.size(); ++i) {
                // ---- 1. 拉普拉斯平滑力：移向前后邻居中点 ----
                PointPose smooth_force;
                smooth_force.x = (path[i - 1].x + path[i + 1].x) * 0.5 - path[i].x;
                smooth_force.y = (path[i - 1].y + path[i + 1].y) * 0.5 - path[i].y;
                smooth_force.z = (path[i - 1].z + path[i + 1].z) * 0.5 - path[i].z;

                // ---- 2. cost 场梯度排斥力（仅后半阶段，仅 xy） ----
                double gx = 0.0, gy = 0.0;
                if (use_cost) {
                    const GridIndex gi = worldToGrid(path[i].x, path[i].y, path[i].z);
                    gx = (costFieldAt(GridIndex{gi.x + 1, gi.y, gi.z}) -
                          costFieldAt(GridIndex{gi.x - 1, gi.y, gi.z})) / (2.0 * res);
                    gy = (costFieldAt(GridIndex{gi.x, gi.y + 1, gi.z}) -
                          costFieldAt(GridIndex{gi.x, gi.y - 1, gi.z})) / (2.0 * res);
                }
                // z 方向 cost 梯度始终禁用（地面支撑层会制造虚假向上推力）

                PointPose candidate = path[i];
                candidate.x += alpha_eff * smooth_force.x - beta_eff * gx;
                candidate.y += alpha_eff * smooth_force.y - beta_eff * gy;
                candidate.z += alpha_eff * smooth_force.z;

                // ---- 3. 限制单步位移 ----
                double move_dist = std::sqrt(
                    (candidate.x - path[i].x) * (candidate.x - path[i].x) +
                    (candidate.y - path[i].y) * (candidate.y - path[i].y) +
                    (candidate.z - path[i].z) * (candidate.z - path[i].z));
                if (move_dist > max_step && move_dist > 1e-12) {
                    const double scale = max_step / move_dist;
                    candidate.x = path[i].x + (candidate.x - path[i].x) * scale;
                    candidate.y = path[i].y + (candidate.y - path[i].y) * scale;
                    candidate.z = path[i].z + (candidate.z - path[i].z) * scale;
                    move_dist = max_step;
                }

                // ---- 4. 安全检查 ----
                bool accept = isSegmentTraversable(path[i - 1], candidate) &&
                              isSegmentTraversable(candidate, path[i + 1]);
                if (use_cost && accept) {
                    // 后半阶段追加代价门控：不容许明显向障碍靠近
                    const GridIndex gi = worldToGrid(path[i].x, path[i].y, path[i].z);
                    const GridIndex ci = worldToGrid(candidate.x, candidate.y, candidate.z);
                    accept = (costFieldAt(ci) <= costFieldAt(gi) + cost_tol);
                }
                if (accept) {
                    path[i] = candidate;
                    if (move_dist > max_delta) max_delta = move_dist;
                }
            }

            if (max_delta < 1e-6) break;  // 收敛
        }

        // ---- 末尾安全验证：二分搜索找回最大安全位移 ----
        const int BS_DEPTH = 10;
        for (int pass = 0; pass < 5; ++pass) {
            bool any_unsafe = false;
            for (size_t i = 1; i + 1 < path.size(); ++i) {
                if (!isSegmentTraversable(path[i - 1], path[i]) ||
                    !isSegmentTraversable(path[i], path[i + 1])) {
                    PointPose lo = original[i];
                    PointPose hi = path[i];
                    for (int bs = 0; bs < BS_DEPTH; ++bs) {
                        PointPose mid;
                        mid.x = (lo.x + hi.x) * 0.5;
                        mid.y = (lo.y + hi.y) * 0.5;
                        mid.z = (lo.z + hi.z) * 0.5;
                        if (isSegmentTraversable(path[i - 1], mid) &&
                            isSegmentTraversable(mid, path[i + 1])) {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    path[i] = lo;
                    any_unsafe = true;
                }
            }
            if (!any_unsafe) break;
        }

        // 兜底：顽固点退回 original；全部退回则整体恢复
        bool all_original = true;
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            if (!isSegmentTraversable(path[i - 1], path[i]) ||
                !isSegmentTraversable(path[i], path[i + 1])) {
                path[i] = original[i];
            }
        }
        for (size_t i = 0; i < path.size() && all_original; ++i) {
            if (std::abs(path[i].x - original[i].x) > 1e-6 ||
                std::abs(path[i].y - original[i].y) > 1e-6 ||
                std::abs(path[i].z - original[i].z) > 1e-6) {
                all_original = false;
            }
        }
        if (all_original) path = original;
    }

}