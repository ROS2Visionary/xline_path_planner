#include <gtest/gtest.h>

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"

namespace path_planner
{

TEST(EllipseExtensionTest, PartialEllipseExtendsStartPoint)
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
      "center": { "x": 0.0, "y": 0.0 },
      "major_axis": { "x": 2000.0, "y": 0.0 },
      "ratio": 0.5,
      "start_angle": 0.0,
      "end_angle": 90.0,
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
  auto ellipse = std::dynamic_pointer_cast<Ellipse>(cad.path_lines[0]);
  ASSERT_TRUE(ellipse);

  GridMapConfig map_cfg;
  map_cfg.resolution = 0.05;
  map_cfg.padding = 0.1;
  GridMapGenerator grid(map_cfg);
  ASSERT_TRUE(grid.generate_from_cad(cad));

  PathPlannerConfig cfg;
  cfg.ellipse_extension_length = 0.4;
  cfg.ellipse_extension_max_angle = 90.0;
  PathPlanner planner(cfg);
  planner.set_grid_map(grid.get_grid_map(), &grid);

  PathOffsetConfig offsets;
  offsets.center_offset = 0.0;
  auto segments = planner.plan_paths(cad, offsets);

  ASSERT_EQ(segments.size(), 1u);
  EXPECT_EQ(segments[0].type, RouteType::DRAWING_PATH);
  ASSERT_GE(segments[0].points.size(), 2u);

  EXPECT_GT(segments[0].points.front().distance(ellipse->start), 1e-4);
  EXPECT_LT(segments[0].points.back().distance(ellipse->end), 1e-6);
}

}  // namespace path_planner

