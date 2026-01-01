#include <gtest/gtest.h>

#include "xline_path_planner/output_formatter.hpp"

namespace path_planner
{

TEST(TransitionSplineExportTest, ExportTransitionAsSplineWithVertices)
{
  RouteSegment transition(RouteType::TRANSITION_PATH, /*id=*/ -1, PrinterType::CENTER_PRINTER,
    InkMode::SOLID);
  transition.execute_backward = true;
  transition.points = {
    Point3D{1.0, 2.0, 0.0},
    Point3D{1.1, 2.2, 0.0},
    Point3D{1.2, 2.4, 0.0},
  };

  OutputFormatter fmt;
  auto out = fmt.format_planned_paths_to_cad_json({transition}, "test");

  ASSERT_TRUE(out.contains("lines"));
  ASSERT_TRUE(out["lines"].is_array());
  ASSERT_EQ(out["lines"].size(), 1u);

  const auto & j = out["lines"][0];
  EXPECT_EQ(j["id"].get<int>(), 1000000);
  EXPECT_EQ(j["type"].get<std::string>(), "spline");
  EXPECT_EQ(j["layer"].get<std::string>(), "TRANSITION");
  EXPECT_TRUE(j.contains("vertices"));
  ASSERT_EQ(j["vertices"].size(), 3u);

  // meters -> mm
  EXPECT_DOUBLE_EQ(j["start"]["x"].get<double>(), 1000.0);
  EXPECT_DOUBLE_EQ(j["start"]["y"].get<double>(), 2000.0);
  EXPECT_DOUBLE_EQ(j["end"]["x"].get<double>(), 1200.0);
  EXPECT_DOUBLE_EQ(j["end"]["y"].get<double>(), 2400.0);

  EXPECT_TRUE(j["backward"].get<bool>());
  EXPECT_EQ(j["degree"].get<int>(), 3);
  EXPECT_FALSE(j["periodic"].get<bool>());
  EXPECT_FALSE(j["is_closed"].get<bool>());
  EXPECT_TRUE(j["control_points"].is_array());
  EXPECT_TRUE(j["knots"].is_array());
  EXPECT_TRUE(j["weights"].is_array());
}

}  // namespace path_planner
