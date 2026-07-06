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

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;

        g_score[start] = 0.0;
        open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

        const std::vector<GridIndex> directions = make26Directions();
        int iters = 0;

        const clock_t plan_start = clock();
        const int log_interval = std::max(1, max_iterations / 20);  // 每 5% 输出一次
        int next_log_iter = log_interval;

        std::cout << "  A* planning: start=(" << start.x << "," << start.y << "," << start.z << ") goal=(" << goal.x << "," << goal.y << "," << goal.z << "), max_iterations=" << max_iterations << ", grid resolution=" << octree_->getResolution() << std::endl;

        while (!open_set.empty() && iters < max_iterations) 
        {
            const QueueNode current = open_set.top();
            open_set.pop();
            ++iters;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);

            if (current.idx == goal) {
                const auto cells = reconstructPath(came_from, current.idx);
                const double elapsed = (clock() - plan_start) / static_cast<double>(CLOCKS_PER_SEC);
                std::cout << "  A* planning: path found in " << iters << " iterations (" << cells.size() << " waypoints, " << elapsed << " s)" << std::endl;
                planner_results_.clear();
                for (std::size_t i = 0; i < cells.size(); ++i) 
                {
                    const auto & c = cells[i];
                    const auto p = gridToWorld(c);
                    PointPose temp;
                    temp.x = p.x();
                    temp.y = p.y();
                    temp.z = p.z();
                    planner_results_.push_back(temp);
                }

                return true;
            }

            // 进度日志
            if (iters >= next_log_iter) {
                const double elapsed = (clock() - plan_start) / static_cast<double>(CLOCKS_PER_SEC);
                const auto cur_world = gridToWorld(current.idx);
                std::cout << "  A* planning: " << iters << " / " << max_iterations << " iterations (" << (100.0 * iters / max_iterations) << "%), open=" << open_set.size() << ", closed=" << closed_set.size() << ", dist_to_goal=" << (euclidean(current.idx, goal) * octree_->getResolution()) << ", " << elapsed << " s" << std::endl;
                next_log_iter = iters + log_interval;
            }

            for (const auto & d : directions) 
            {
                GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
                if (closed_set.find(nbr) != closed_set.end()) {
                continue;
                }
                if (!isCellTraversable(
                    nbr, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                continue;
                }
                const double step_cost = euclidean(current.idx, nbr);
                double tentative_g = current.g + step_cost;
                if (enable_preblocked_costmap) {
                tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
                }

                auto g_it = g_score.find(nbr);
                if (g_it == g_score.end() || tentative_g < g_it->second) {
                came_from[nbr] = current.idx;
                g_score[nbr] = tentative_g;
                const double f = tentative_g + euclidean(nbr, goal);
                open_set.push(QueueNode{nbr, f, tentative_g});
                }
            }
        }

        const double elapsed = (clock() - plan_start) / static_cast<double>(CLOCKS_PER_SEC);
        std::cout << "  A* planning: FAILED after " << iters << " / " << max_iterations << " iterations (" << elapsed << " s), open=" << open_set.size() << ", closed=" << closed_set.size() << std::endl;
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

            for (int z = min_idx.z; z <= max_idx.z; ++z) {
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
        occupancy_cache_.reserve(octree_->getNumLeafNodes());
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupancy_cache_.insert(worldToGrid(it.getX(), it.getY(), it.getZ()));
            }
        }
        std::cout << "  Occupancy cache: built " << occupancy_cache_.size()
                  << " cells from " << octree_->getNumLeafNodes() << " leaf nodes in "
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
            return;
        }

        std::cout << "GlobalPlanner: planning cache not found, rebuilding..." << std::endl;
        rebuildPreblockedCells();
        rebuildDerivedLayers();
        rebuildPreblockedCostmap();
        savePlanningCache(cache_path);
    }

    void GlobalPlanner::rebuildPreblockedCostmap()
    {
        std::cout << "GlobalPlanner::rebuildPreblockedCostmap rebuilding preblocked costmap... " << std::endl;
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
        std::cout << "  Preblocked costmap: spreading " << total_cells << " cells, radius " << radius_cells << ", " << (num_threads_ > 0 ? num_threads_ : omp_get_max_threads()) << " thread(s)..." << std::endl;

        std::atomic<size_t> processed{0};
        int next_log_pct = 10;

        const int nt = (num_threads_ > 0) ? num_threads_ : 0;

        // 将 preblocked_cells_ 转为 vector 以便 OpenMP 随机访问
        std::vector<GridIndex> pb_vec(preblocked_cells_.begin(), preblocked_cells_.end());

#pragma omp parallel num_threads(nt) if(nt != 1)
        {
            std::unordered_map<GridIndex, double, GridIndexHash> local_costmap;

#pragma omp for schedule(dynamic, 32) nowait
            for (int64_t i = 0; i < static_cast<int64_t>(pb_vec.size()); ++i) {
                const auto & c = pb_vec[static_cast<size_t>(i)];
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            const GridIndex n{c.x + dx, c.y + dy, c.z + dz};
                            if (!isInsideMetricBounds(n)) {
                                continue;
                            }
                            if (traversable_cells_.find(n) == traversable_cells_.end()) {
                                continue;
                            }
                            if (preblocked_cells_.find(n) != preblocked_cells_.end()) {
                                continue;
                            }
                            const double d = std::sqrt(
                                static_cast<double>(dx * dx + dy * dy + dz * dz));
                            if (d > static_cast<double>(radius_cells)) {
                                continue;
                            }
                            const double cst = std::max(0.0, (denom - d) / denom);
                            auto it = local_costmap.find(n);
                            if (it == local_costmap.end() || cst > it->second) {
                                local_costmap[n] = cst;
                            }
                        }
                    }
                }

                // 进度日志
                const int64_t done = processed.fetch_add(1) + 1;
                const int pct = static_cast<int>(done * 100LL / static_cast<int64_t>(total_cells));
                if (pct >= next_log_pct) {
#pragma omp critical
                    if (pct >= next_log_pct) {
                        std::cout << "  Preblocked costmap: " << pct << "% (" << static_cast<long long>(done) << " / " << total_cells << " cells, " << preblocked_costmap_.size() << " costmap, " << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;
                        next_log_pct = pct + 10;
                        if (next_log_pct > 100) next_log_pct = 100;
                    }
                }
            }

            // 合并局部结果到全局 costmap（取最大值）
#pragma omp critical
            for (const auto & entry : local_costmap) {
                auto it = preblocked_costmap_.find(entry.first);
                if (it == preblocked_costmap_.end() || entry.second > it->second) {
                    preblocked_costmap_[entry.first] = entry.second;
                }
            }
        }

        std::cout << "  Preblocked costmap: 100% (" << total_cells << " / " << total_cells << " cells, " << preblocked_costmap_.size() << " costmap, " << ((clock() - build_start) / static_cast<double>(CLOCKS_PER_SEC)) << " s)" << std::endl;

        // RCLCPP_INFO(
        // get_logger(),
        // "Preblocked costmap rebuilt. cells=%zu radius=%d",
        // preblocked_costmap_.size(), radius_cells);
        // std::cout << "Preblocked costmap rebuilt. cells=" << preblocked_costmap_.size() << " radius=" << radius_cells << " " << std::endl;
        // publishRiskCostCloud();
    }







}