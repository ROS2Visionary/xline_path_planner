#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "xline_path_planner/filename_utils.hpp"

TEST(FilenameUtils, PreservesChineseInPlannedOutputFilename)
{
  const std::filesystem::path out_dir = std::filesystem::u8path(u8"/tmp/out");
  const std::filesystem::path input_path = std::filesystem::u8path(u8"/tmp/中文文件.dxf");

  const auto out_path = path_planner::planned_json_output_path(out_dir, input_path);
  EXPECT_EQ(out_path.filename().u8string(), std::string(u8"planned_中文文件.json"));
}

TEST(FilenameUtils, ReplacesPathSeparatorsAndControls)
{
  EXPECT_EQ(path_planner::sanitize_filename_component_preserve_utf8("a/b"), "a_b");
  EXPECT_EQ(path_planner::sanitize_filename_component_preserve_utf8("a\\b"), "a_b");
  EXPECT_EQ(path_planner::sanitize_filename_component_preserve_utf8(std::string("a\nb")), "a_b");
}
