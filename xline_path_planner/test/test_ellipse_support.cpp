#include <gtest/gtest.h>

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"

namespace path_planner
{

TEST(EllipseSupportTest, ParseGenerateBoundsAndPlan)
{
  const std::string json_text = R"json(
{
  "layers": [
    { "layer_id": 1, "name": "0", "color": "#007bff", "visible": true, "locked": false }
  ],
  "lines": [
    {
      "id": 1,
      "type": "ellipse",
      "line_type": "continuous",
      "thickness": 1.0,
      "hidden": false,
      "layer_id": 1,
      "color": "#007bff",
      "center": { "x": 3722.199059, "y": 4342.896461 },
      "major_axis": { "x": 5080.0, "y": 0.0 },
      "ratio": 0.25,
      "start_angle": 0.0,
      "end_angle": 360.0,
      "rotation": 0.0
    }
  ]
}
)json";

  CADParserConfig parser_cfg;
  parser_cfg.unit_conversion_factor = 1000.0;
  parser_cfg.auto_scale_coordinates = true;
  parser_cfg.angle_unit = AngleUnit::DEGREES;

  CADParser parser(parser_cfg);
  ASSERT_TRUE(parser.parse_from_string(json_text));

  const CADData& cad = parser.get_cad_data();
  ASSERT_EQ(cad.path_lines.size(), 1u);
  ASSERT_TRUE(cad.path_lines[0]);

  auto ellipse = std::dynamic_pointer_cast<Ellipse>(cad.path_lines[0]);
  ASSERT_TRUE(ellipse);

  EXPECT_NEAR(ellipse->center.x, 3722.199059 / 1000.0, 1e-9);
  EXPECT_NEAR(ellipse->center.y, 4342.896461 / 1000.0, 1e-9);
  EXPECT_NEAR(ellipse->major_radius(), 5080.0 / 1000.0, 1e-9);
  EXPECT_NEAR(ellipse->ratio, 0.25, 1e-12);
  EXPECT_NEAR(ellipse->minor_radius(), (5080.0 / 1000.0) * 0.25, 1e-9);
  EXPECT_NEAR(ellipse->start_angle, 0.0, 1e-12);
  EXPECT_NEAR(ellipse->end_angle, 2.0 * M_PI, 1e-9);

  GridMapConfig map_cfg;
  map_cfg.resolution = 0.05;
  map_cfg.padding = 0.1;
  GridMapGenerator grid(map_cfg);
  ASSERT_TRUE(grid.generate_from_cad(cad));

  double min_x, min_y, max_x, max_y;
  grid.get_map_bounds(min_x, min_y, max_x, max_y);

  const double a = ellipse->major_radius();
  const double b = ellipse->minor_radius();
  EXPECT_NEAR(min_x, ellipse->center.x - a - map_cfg.padding, 1e-6);
  EXPECT_NEAR(max_x, ellipse->center.x + a + map_cfg.padding, 1e-6);
  EXPECT_NEAR(min_y, ellipse->center.y - b - map_cfg.padding, 1e-6);
  EXPECT_NEAR(max_y, ellipse->center.y + b + map_cfg.padding, 1e-6);

  PathPlanner planner(PathPlannerConfig{});
  planner.set_grid_map(grid.get_grid_map(), &grid);

  PathOffsetConfig offsets;
  offsets.left_offset = -0.05;
  offsets.right_offset = 0.05;
  offsets.center_offset = 0.0;

  auto segments = planner.plan_paths(cad, offsets);
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_EQ(segments[0].type, RouteType::DRAWING_PATH);
  EXPECT_EQ(segments[0].line_id, 1);
  EXPECT_GT(segments[0].points.size(), 200u);
  EXPECT_LT(segments[0].points.front().distance(segments[0].points.back()), 1e-6);
}

}  // namespace path_planner
