#include <gtest/gtest.h>

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/output_formatter.hpp"
#include "xline_path_planner/path_planner.hpp"

namespace path_planner
{

TEST(SplineSupportTest, ParsePlanAndExportSpline)
{
  const std::string json_text = R"json(
{
  "layers": [
    { "layer_id": 1, "name": "0", "color": "#007bff", "visible": true, "locked": false }
  ],
  "lines": [
    {
      "id": 1,
      "type": "spline",
      "line_type": "continuous",
      "thickness": 1.0,
      "hidden": false,
      "layer_id": 1,
      "color": "#007bff",
      "degree": 3,
      "control_points": [],
      "knots": [],
      "weights": [],
      "periodic": false,
      "is_closed": false,
      "vertices": [
        { "x": 0.0, "y": 0.0 },
        { "x": 100.0, "y": 0.0 },
        { "x": 200.0, "y": 100.0 },
        { "x": 300.0, "y": 200.0 }
      ]
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
  auto spline = std::dynamic_pointer_cast<Spline>(cad.path_lines[0]);
  ASSERT_TRUE(spline);
  ASSERT_EQ(spline->vertices.size(), 4u);
  EXPECT_FALSE(spline->closed);

  GridMapConfig map_cfg;
  map_cfg.resolution = 0.05;
  map_cfg.padding = 0.1;
  GridMapGenerator grid(map_cfg);
  ASSERT_TRUE(grid.generate_from_cad(cad));

  PathPlanner planner(PathPlannerConfig{});
  planner.set_grid_map(grid.get_grid_map(), &grid);
  PathOffsetConfig offsets;
  offsets.center_offset = 0.0;
  auto segments = planner.plan_paths(cad, offsets);
  ASSERT_EQ(segments.size(), 1u);
  ASSERT_EQ(segments[0].line_id, 1);
  ASSERT_EQ(segments[0].points.size(), 4u);

  OutputFormatter fmt;
  auto out = fmt.format_planned_paths_to_cad_json(segments, cad, "test");
  ASSERT_TRUE(out.contains("lines"));
  ASSERT_TRUE(out["lines"].is_array());
  ASSERT_EQ(out["lines"].size(), 1u);
  EXPECT_EQ(out["lines"][0]["type"].get<std::string>(), "spline");
  ASSERT_TRUE(out["lines"][0].contains("vertices"));
  ASSERT_EQ(out["lines"][0]["vertices"].size(), 4u);
}

}  // namespace path_planner

