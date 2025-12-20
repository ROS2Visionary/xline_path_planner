#include "xline_path_planner/collinear_merger.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>

namespace path_planner
{

CollinearMerger::CollinearMerger(double distance_tolerance, double angle_tolerance, double min_segment_length)
  : distance_tolerance_(distance_tolerance), angle_tolerance_(angle_tolerance), min_segment_length_(min_segment_length)
{
}

double CollinearMerger::point_distance(const Point3D& a, const Point3D& b) const
{
  return a.distance(b);
}

Point3D CollinearMerger::direction_unit(const Line& line) const
{
  const double dx = line.end.x - line.start.x;
  const double dy = line.end.y - line.start.y;
  const double dz = line.end.z - line.start.z;
  const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (len < 1e-12)
  {
    return { 0.0, 0.0, 0.0 };
  }
  return { dx / len, dy / len, dz / len };
}

double CollinearMerger::point_to_line_distance(const Point3D& point, const Point3D& line_start,
                                               const Point3D& line_end) const
{
  const double abx = line_end.x - line_start.x;
  const double aby = line_end.y - line_start.y;
  const double abz = line_end.z - line_start.z;

  const double apx = point.x - line_start.x;
  const double apy = point.y - line_start.y;
  const double apz = point.z - line_start.z;

  const double cx = aby * apz - abz * apy;
  const double cy = abz * apx - abx * apz;
  const double cz = abx * apy - aby * apx;

  const double cross_len = std::sqrt(cx * cx + cy * cy + cz * cz);
  const double ab_len = std::sqrt(abx * abx + aby * aby + abz * abz);
  if (ab_len < 1e-12)
  {
    return point_distance(point, line_start);
  }
  return cross_len / ab_len;
}

bool CollinearMerger::merge_attributes_compatible(const Line& a, const Line& b) const
{
  if (a.layer_id != b.layer_id) return false;
  if (a.layer != b.layer) return false;
  if (a.line_type != b.line_type) return false;
  if (a.color != b.color) return false;
  if (a.hidden != b.hidden) return false;
  if (std::abs(a.thickness - b.thickness) > 1e-9) return false;
  return true;
}

bool CollinearMerger::are_collinear(const Line& a, const Line& b) const
{
  if (a.length < min_segment_length_ || b.length < min_segment_length_)
  {
    return false;
  }

  const double dist_b_start = point_to_line_distance(b.start, a.start, a.end);
  const double dist_b_end = point_to_line_distance(b.end, a.start, a.end);
  if (dist_b_start > distance_tolerance_ || dist_b_end > distance_tolerance_)
  {
    return false;
  }

  const Point3D dir_a = direction_unit(a);
  const Point3D dir_b = direction_unit(b);
  const double dot = std::abs(dir_a.x * dir_b.x + dir_a.y * dir_b.y + dir_a.z * dir_b.z);
  const double threshold = std::cos(std::max(0.0, angle_tolerance_));
  return dot >= threshold;
}

ConnectionType CollinearMerger::get_connection_type(const Line& a, const Line& b) const
{
  const double d_end_start = point_distance(a.end, b.start);
  const double d_end_end = point_distance(a.end, b.end);
  const double d_start_start = point_distance(a.start, b.start);
  const double d_start_end = point_distance(a.start, b.end);

  const double min_dist = std::min({ d_end_start, d_end_end, d_start_start, d_start_end });
  if (min_dist > distance_tolerance_)
  {
    return ConnectionType::NONE;
  }

  if (d_end_start <= distance_tolerance_) return ConnectionType::END_TO_START;
  if (d_end_end <= distance_tolerance_) return ConnectionType::END_TO_END;
  if (d_start_start <= distance_tolerance_) return ConnectionType::START_TO_START;
  if (d_start_end <= distance_tolerance_) return ConnectionType::START_TO_END;
  return ConnectionType::NONE;
}

namespace {

struct EndpointKey
{
  int64_t ix = 0;
  int64_t iy = 0;
  int64_t iz = 0;

  bool operator==(const EndpointKey& o) const { return ix == o.ix && iy == o.iy && iz == o.iz; }
};

struct EndpointKeyHash
{
  size_t operator()(const EndpointKey& k) const
  {
    size_t h = 1469598103934665603ULL;
    auto mix = [&](uint64_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };
    mix(static_cast<uint64_t>(k.ix));
    mix(static_cast<uint64_t>(k.iy));
    mix(static_cast<uint64_t>(k.iz));
    return h;
  }
};

EndpointKey quantize_point(const Point3D& p, double tol)
{
  if (tol <= 0.0)
  {
    return { 0, 0, 0 };
  }
  return { static_cast<int64_t>(std::floor(p.x / tol)), static_cast<int64_t>(std::floor(p.y / tol)),
           static_cast<int64_t>(std::floor(p.z / tol)) };
}

struct DSU
{
  std::vector<int32_t> parent;
  std::vector<int32_t> rank;

  explicit DSU(size_t n) : parent(n), rank(n, 0)
  {
    for (size_t i = 0; i < n; ++i)
      parent[i] = static_cast<int32_t>(i);
  }

  int32_t find(int32_t x)
  {
    if (parent[x] != x)
      parent[x] = find(parent[x]);
    return parent[x];
  }

  void unite(int32_t a, int32_t b)
  {
    a = find(a);
    b = find(b);
    if (a == b) return;
    if (rank[a] < rank[b])
      parent[a] = b;
    else if (rank[a] > rank[b])
      parent[b] = a;
    else
    {
      parent[b] = a;
      rank[a]++;
    }
  }
};

}  // namespace

std::vector<MergeGroup> CollinearMerger::merge(const std::vector<const Line*>& lines) const
{
  const size_t n = lines.size();
  if (n == 0)
  {
    return {};
  }

  DSU dsu(n);

  // 使用端点量化做候选对预筛选，避免全对 O(n^2) 比较
  std::unordered_map<EndpointKey, std::vector<int32_t>, EndpointKeyHash> bucket;
  bucket.reserve(n * 2);

  for (size_t i = 0; i < n; ++i)
  {
    const Line* li = lines[i];
    if (!li || li->type != GeometryType::LINE) continue;
    bucket[quantize_point(li->start, distance_tolerance_)].push_back(static_cast<int32_t>(i));
    bucket[quantize_point(li->end, distance_tolerance_)].push_back(static_cast<int32_t>(i));
  }

  auto add_edge_if_mergeable = [&](int32_t ia, int32_t ib) {
    if (ia == ib) return;
    const Line* a = lines[ia];
    const Line* b = lines[ib];
    if (!a || !b) return;
    if (a->type != GeometryType::LINE || b->type != GeometryType::LINE) return;
    if (!merge_attributes_compatible(*a, *b)) return;
    const ConnectionType conn = get_connection_type(*a, *b);
    if (conn == ConnectionType::NONE) return;
    if (!are_collinear(*a, *b)) return;
    dsu.unite(ia, ib);
  };

  // 针对每条线段，在其端点附近的桶中搜索候选线段（包含邻近桶）
  for (int32_t i = 0; i < static_cast<int32_t>(n); ++i)
  {
    const Line* li = lines[i];
    if (!li || li->type != GeometryType::LINE) continue;

    std::unordered_set<int32_t> candidates;
    candidates.reserve(32);

    auto collect_candidates_near = [&](const Point3D& p) {
      const EndpointKey base = quantize_point(p, distance_tolerance_);
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            EndpointKey k{ base.ix + dx, base.iy + dy, base.iz + dz };
            auto it = bucket.find(k);
            if (it == bucket.end()) continue;
            for (int32_t j : it->second)
            {
              if (j <= i) continue;
              candidates.insert(j);
            }
          }
        }
      }
    };

    collect_candidates_near(li->start);
    collect_candidates_near(li->end);

    for (int32_t j : candidates)
    {
      add_edge_if_mergeable(i, j);
    }
  }

  // root -> indices
  std::unordered_map<int32_t, std::vector<int32_t>> components;
  components.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    if (!lines[i] || lines[i]->type != GeometryType::LINE) continue;
    const int32_t root = dsu.find(static_cast<int32_t>(i));
    components[root].push_back(static_cast<int32_t>(i));
  }

  std::vector<MergeGroup> result;
  result.reserve(components.size());

  // 在每个连通分量内按端点图提取链（可处理分叉：每个分支单独成组）
  for (auto& [root, indices] : components)
  {
    (void)root;

    struct EndpointRef
    {
      Point3D p;
      int32_t seg_idx = -1;
      bool is_start = true;
    };

    std::vector<EndpointRef> endpoints;
    endpoints.reserve(indices.size() * 2);
    for (int32_t idx : indices)
    {
      const Line* l = lines[idx];
      if (!l) continue;
      endpoints.push_back({ l->start, idx, true });
      endpoints.push_back({ l->end, idx, false });
    }

    DSU endpoint_dsu(endpoints.size());
    std::unordered_map<EndpointKey, std::vector<int32_t>, EndpointKeyHash> endpoint_bucket;
    endpoint_bucket.reserve(endpoints.size());
    for (int32_t ei = 0; ei < static_cast<int32_t>(endpoints.size()); ++ei)
    {
      endpoint_bucket[quantize_point(endpoints[ei].p, distance_tolerance_)].push_back(ei);
    }

    // 聚类：将距离在 tolerance 内的端点归为同一个“节点”
    for (int32_t ei = 0; ei < static_cast<int32_t>(endpoints.size()); ++ei)
    {
      const EndpointKey base = quantize_point(endpoints[ei].p, distance_tolerance_);
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            EndpointKey k{ base.ix + dx, base.iy + dy, base.iz + dz };
            auto it = endpoint_bucket.find(k);
            if (it == endpoint_bucket.end()) continue;
            for (int32_t ej : it->second)
            {
              if (ej <= ei) continue;
              if (endpoints[ei].p.distance(endpoints[ej].p) <= distance_tolerance_)
              {
                endpoint_dsu.unite(ei, ej);
              }
            }
          }
        }
      }
    }

    // node_id -> incident segment indices
    std::unordered_map<int32_t, std::vector<int32_t>> node_to_segments;
    node_to_segments.reserve(indices.size() * 2);

    // seg_idx -> (start_node_id, end_node_id)
    std::unordered_map<int32_t, std::pair<int32_t, int32_t>> seg_nodes;
    seg_nodes.reserve(indices.size());

    std::unordered_map<int32_t, int32_t> seg_start_ei;
    seg_start_ei.reserve(indices.size());
    std::unordered_map<int32_t, int32_t> seg_end_ei;
    seg_end_ei.reserve(indices.size());

    for (int32_t ei = 0; ei < static_cast<int32_t>(endpoints.size()); ++ei)
    {
      const int32_t node = endpoint_dsu.find(ei);
      const int32_t seg = endpoints[ei].seg_idx;
      node_to_segments[node].push_back(seg);
      if (endpoints[ei].is_start)
      {
        seg_start_ei[seg] = ei;
      }
      else
      {
        seg_end_ei[seg] = ei;
      }
    }

    for (int32_t idx : indices)
    {
      auto it_s = seg_start_ei.find(idx);
      auto it_e = seg_end_ei.find(idx);
      if (it_s == seg_start_ei.end() || it_e == seg_end_ei.end()) continue;
      const int32_t ns = endpoint_dsu.find(it_s->second);
      const int32_t ne = endpoint_dsu.find(it_e->second);
      seg_nodes[idx] = { ns, ne };
    }

    std::unordered_set<int32_t> visited;
    visited.reserve(indices.size());

    auto traverse_chain = [&](int32_t start_node, int32_t start_seg) -> MergeGroup {
      MergeGroup group;

      int32_t current_node = start_node;
      int32_t current_seg = start_seg;
      bool reverse_current = false;

      // 确定 start_seg 的方向：让 segment 的“起点端”落在 current_node
      {
        const auto it = seg_nodes.find(current_seg);
        if (it != seg_nodes.end())
        {
          const auto& [ns, ne] = it->second;
          reverse_current = (ne == current_node);
        }
      }

      while (true)
      {
        if (visited.find(current_seg) != visited.end()) break;
        visited.insert(current_seg);

        const Line* l = lines[current_seg];
        if (!l) break;

        group.line_ids.push_back(l->id);
        group.need_reverse.push_back(reverse_current);

        // 走到该段的另一端
        int32_t next_node = -1;
        {
          const auto it = seg_nodes.find(current_seg);
          if (it == seg_nodes.end()) break;
          const auto& [ns, ne] = it->second;
          next_node = reverse_current ? ns : ne;
        }

        // 从 next_node 选择下一个未访问的 segment
        auto it_list = node_to_segments.find(next_node);
        if (it_list == node_to_segments.end()) break;

        int32_t next_seg = -1;
        for (int32_t cand : it_list->second)
        {
          if (visited.find(cand) == visited.end())
          {
            next_seg = cand;
            break;
          }
        }
        if (next_seg < 0) break;

        // 计算 next_seg 是否需要反转以衔接 next_node
        bool next_reverse = false;
        {
          const auto it = seg_nodes.find(next_seg);
          if (it != seg_nodes.end())
          {
            const auto& [ns, ne] = it->second;
            next_reverse = (ne == next_node);
          }
        }

        current_node = next_node;
        current_seg = next_seg;
        reverse_current = next_reverse;
      }

      // 计算合并后的起点/终点（按链的第一个/最后一个实际端点）
      if (!group.line_ids.empty())
      {
        const int32_t first_seg_idx = start_seg;
        const Line* first = lines[first_seg_idx];

        // 找最后一个 segment index（visited 内最末的 current_seg）
        // 由于 current_seg 在循环结束后为最后一次赋值，这里取最后一次插入的 segment id 反查 index。
        int32_t last_idx = -1;
        if (!group.line_ids.empty())
        {
          const int32_t last_id = group.line_ids.back();
          for (int32_t idx : indices)
          {
            if (lines[idx] && lines[idx]->id == last_id)
            {
              last_idx = idx;
              break;
            }
          }
        }

        const Line* last = (last_idx >= 0) ? lines[last_idx] : nullptr;

        if (first && last)
        {
          group.merged_start = group.need_reverse.front() ? first->end : first->start;
          group.merged_end = group.need_reverse.back() ? last->start : last->end;
        }
      }

      return group;
    };

    // 1) 先从度为1的端点启动（链式结构）
    for (const auto& [node, segs] : node_to_segments)
    {
      // 注意：node_to_segments[node] 可能包含重复 seg（同一 seg 在 start/end 都落到同一 node）
      std::unordered_set<int32_t> uniq(segs.begin(), segs.end());
      if (uniq.size() != 1) continue;
      const int32_t seg = *uniq.begin();
      if (visited.find(seg) != visited.end()) continue;
      result.push_back(traverse_chain(node, seg));
    }

    // 2) 剩余未访问段（可能是环或全度=2的结构）
    for (int32_t idx : indices)
    {
      if (visited.find(idx) != visited.end()) continue;
      auto it = seg_nodes.find(idx);
      if (it == seg_nodes.end()) continue;
      result.push_back(traverse_chain(it->second.first, idx));
    }
  }

  // 对于无法合并的（没有任何边）段，components 中会各自成组，仍会输出单段 group。
  // 保证输出顺序稳定：按 merged_start 的 x/y 进行简单排序（可选）。
  std::stable_sort(result.begin(), result.end(), [](const MergeGroup& a, const MergeGroup& b) {
    if (a.merged_start.x != b.merged_start.x) return a.merged_start.x < b.merged_start.x;
    return a.merged_start.y < b.merged_start.y;
  });

  return result;
}

}  // namespace path_planner
