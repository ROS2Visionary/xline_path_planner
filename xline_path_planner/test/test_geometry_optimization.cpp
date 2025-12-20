#include <gtest/gtest.h>
#include <cmath>

#include "xline_path_planner/geometry_preprocessor.hpp"
#include "xline_path_planner/collinear_merger.hpp"

namespace {

path_planner::Point3D pt(double x, double y, double z)
{
  path_planner::Point3D p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

}  // namespace

TEST(GeometryPreprocessorTest, SplitSimplePolyline)
{
  path_planner::Polyline poly;
  poly.id = 1;
  poly.vertices.push_back(pt(0.0, 0.0, 0.0));
  poly.vertices.push_back(pt(1.0, 0.0, 0.0));
  poly.vertices.push_back(pt(1.0, 1.0, 0.0));
  poly.vertices.push_back(pt(0.0, 1.0, 0.0));
  poly.closed = false;
  poly.update_geometry();

  path_planner::CADData cad;
  cad.path_lines.push_back(std::make_shared<path_planner::Polyline>(poly));

  path_planner::PathPlannerConfig cfg;
  cfg.split_polyline = true;
  cfg.preserve_polyline_info = true;

  auto out = path_planner::GeometryPreprocessor::preprocess(cad, cfg);
  ASSERT_EQ(out.path_lines.size(), 3u);
  for (const auto& l : out.path_lines)
  {
    ASSERT_TRUE(l);
    EXPECT_EQ(l->type, path_planner::GeometryType::LINE);
    auto seg = std::dynamic_pointer_cast<path_planner::LineSegment>(l);
    ASSERT_TRUE(seg);
    EXPECT_EQ(seg->parent_polyline_id, 1);
  }
}

TEST(GeometryPreprocessorTest, SplitClosedPolyline)
{
  path_planner::Polyline poly;
  poly.id = 2;
  poly.vertices.push_back(pt(0.0, 0.0, 0.0));
  poly.vertices.push_back(pt(1.0, 0.0, 0.0));
  poly.vertices.push_back(pt(1.0, 1.0, 0.0));
  poly.closed = true;
  poly.update_geometry();

  path_planner::CADData cad;
  cad.path_lines.push_back(std::make_shared<path_planner::Polyline>(poly));

  path_planner::PathPlannerConfig cfg;
  cfg.split_polyline = true;
  cfg.preserve_polyline_info = true;

  auto out = path_planner::GeometryPreprocessor::preprocess(cad, cfg);
  ASSERT_EQ(out.path_lines.size(), 3u);
}

TEST(CollinearMergerTest, MergeTwoCollinearLines)
{
  path_planner::Line a;
  a.id = 1;
  a.type = path_planner::GeometryType::LINE;
  a.start = pt(0.0, 0.0, 0.0);
  a.end = pt(1.0, 0.0, 0.0);
  a.length = a.start.distance(a.end);

  path_planner::Line b;
  b.id = 2;
  b.type = path_planner::GeometryType::LINE;
  b.start = pt(1.0, 0.0, 0.0);
  b.end = pt(2.0, 0.0, 0.0);
  b.length = b.start.distance(b.end);

  std::vector<const path_planner::Line*> lines = { &a, &b };
  path_planner::CollinearMerger merger(0.001, 0.001, 0.0);
  auto groups = merger.merge(lines);

  ASSERT_EQ(groups.size(), 1u);
  ASSERT_EQ(groups[0].line_ids.size(), 2u);
  const double s = groups[0].merged_start.x;
  const double e = groups[0].merged_end.x;
  EXPECT_TRUE((std::abs(s - 0.0) < 1e-6 && std::abs(e - 2.0) < 1e-6) ||
              (std::abs(s - 2.0) < 1e-6 && std::abs(e - 0.0) < 1e-6));
}

TEST(CollinearMergerTest, NoMergeForNonCollinear)
{
  path_planner::Line a;
  a.id = 1;
  a.type = path_planner::GeometryType::LINE;
  a.start = pt(0.0, 0.0, 0.0);
  a.end = pt(1.0, 0.0, 0.0);
  a.length = a.start.distance(a.end);

  path_planner::Line b;
  b.id = 2;
  b.type = path_planner::GeometryType::LINE;
  b.start = pt(1.0, 0.0, 0.0);
  b.end = pt(1.0, 1.0, 0.0);
  b.length = b.start.distance(b.end);

  std::vector<const path_planner::Line*> lines = { &a, &b };
  path_planner::CollinearMerger merger(0.001, 0.001, 0.0);
  auto groups = merger.merge(lines);

  ASSERT_EQ(groups.size(), 2u);
}

TEST(CollinearMergerTest, NoMergeWithGap)
{
  path_planner::Line a;
  a.id = 1;
  a.type = path_planner::GeometryType::LINE;
  a.start = pt(0.0, 0.0, 0.0);
  a.end = pt(1.0, 0.0, 0.0);
  a.length = a.start.distance(a.end);

  path_planner::Line b;
  b.id = 2;
  b.type = path_planner::GeometryType::LINE;
  b.start = pt(1.5, 0.0, 0.0);
  b.end = pt(2.0, 0.0, 0.0);
  b.length = b.start.distance(b.end);

  std::vector<const path_planner::Line*> lines = { &a, &b };
  path_planner::CollinearMerger merger(0.001, 0.001, 0.0);
  auto groups = merger.merge(lines);

  ASSERT_EQ(groups.size(), 2u);
}
