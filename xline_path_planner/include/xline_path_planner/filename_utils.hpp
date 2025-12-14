#pragma once

#include <filesystem>
#include <string>

namespace path_planner
{

inline std::string sanitize_filename_component_preserve_utf8(std::string name)
{
  if (name.empty())
  {
    return "request";
  }

  for (char& ch : name)
  {
    const unsigned char uch = static_cast<unsigned char>(ch);
    if (ch == '/' || ch == '\\' || ch == '\0' || uch < 0x20)
    {
      ch = '_';
    }
  }

  return name;
}

inline std::string planned_output_file_id_from_input_path(const std::filesystem::path & input_path)
{
  // On Linux, filesystem paths are typically UTF-8 byte sequences; keep non-ASCII bytes intact.
  return sanitize_filename_component_preserve_utf8(input_path.stem().u8string());
}

inline std::filesystem::path planned_json_output_path(const std::filesystem::path & out_dir,
                                                      const std::filesystem::path & input_path)
{
  const std::string file_id = planned_output_file_id_from_input_path(input_path);
  return out_dir / (std::string("planned_") + file_id + ".json");
}

}  // namespace path_planner
