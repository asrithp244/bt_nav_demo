#pragma once

#include <string>
#include <vector>

#include "bt_nav_eval/scenario_types.hpp"

namespace bt_nav_eval
{

/**
 * @brief Writes per-run evaluation reports from a vector of ScenarioMetrics.
 *
 * Outputs:
 *   - metrics.json         : machine-readable, one object per scenario
 *   - evaluation_report.md : human-readable pass/fail table + aggregate stats
 */
class ReportGenerator
{
public:
  ReportGenerator() = default;

  /**
   * @param metrics   Results from all completed scenarios.
   * @param out_dir   Directory to write output files into.
   */
  void generate(
    const std::vector<ScenarioMetrics> & metrics,
    const std::string & out_dir) const;

private:
  void write_json(
    const std::vector<ScenarioMetrics> & metrics,
    const std::string & path) const;

  void write_markdown(
    const std::vector<ScenarioMetrics> & metrics,
    const std::string & path) const;

  static std::string escape_json(const std::string & s);
};

}  // namespace bt_nav_eval
