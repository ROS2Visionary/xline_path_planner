#include <gtest/gtest.h>

#include "xline_path_planner/cad_parser.hpp"
#include "xline_path_planner/grid_map_generator.hpp"
#include "xline_path_planner/path_planner.hpp"

namespace path_planner
{

TEST(TransitionConstraintTest, FromSplineEndpointForcesStraightTransition)
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
        { "x": 1000.0, "y": 0.0 },
        { "x": 2000.0, "y": 0.0 }
      ]
    },
    {
      "id": 2,
      "type": "line",
      "line_type": "continuous",
      "thickness": 1.0,
      "hidden": false,
      "layer_id": 1,
      "color": "#007bff",
      "start": { "x": 2000.0, "y": 1000.0 },
      "end": { "x": 3000.0, "y": 1000.0 }
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
  ASSERT_EQ(cad.path_lines.size(), 2u);

  GridMapConfig map_cfg;
  map_cfg.resolution = 0.05;
  map_cfg.padding = 0.1;
  GridMapGenerator grid(map_cfg);
  ASSERT_TRUE(grid.generate_from_cad(cad));

  PathPlanner planner(PathPlannerConfig{});
  planner.set_grid_map(grid.get_grid_map(), &grid);

  BezierTransitionConfig bezier_cfg;
  bezier_cfg.enabled = true;
  bezier_cfg.min_curve_distance = 0.0;
  bezier_cfg.min_angle_for_curve = 0.0;
  bezier_cfg.consider_backward = false;
  planner.set_bezier_config(bezier_cfg);

  PathOffsetConfig offsets;
  offsets.center_offset = 0.0;
  auto segments = planner.plan_paths(cad, offsets);

  ASSERT_EQ(segments.size(), 3u);
  EXPECT_EQ(segments[0].type, RouteType::DRAWING_PATH);
  EXPECT_EQ(segments[1].type, RouteType::TRANSITION_PATH);
  EXPECT_EQ(segments[2].type, RouteType::DRAWING_PATH);

  EXPECT_EQ(segments[1].points.size(), 2u);
}

}  // namespace path_planner

