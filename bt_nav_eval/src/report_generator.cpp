#include "bt_nav_eval/report_generator.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <ctime>

namespace bt_nav_eval
{

namespace
{

// ─── helpers ────────────────────────────────────────────────────────────────

std::string current_timestamp()
{
  std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", std::gmtime(&t));
  return buf;
}

std::string fmt(double v, int prec = 2)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(prec) << v;
  return ss.str();
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────

void ReportGenerator::generate(
  const std::vector<ScenarioMetrics> & metrics,
  const std::string & out_dir) const
{
  const std::string json_path = out_dir + "/metrics.json";
  const std::string md_path   = out_dir + "/evaluation_report.md";

  write_json(metrics, json_path);
  write_markdown(metrics, md_path);
}

// ─────────────────────────────────────────────────────────────────────────────
//  JSON writer
// ─────────────────────────────────────────────────────────────────────────────

void ReportGenerator::write_json(
  const std::vector<ScenarioMetrics> & metrics,
  const std::string & path) const
{
  std::ofstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open for writing: " + path);
  }

  f << "[\n";
  for (std::size_t i = 0; i < metrics.size(); ++i) {
    const auto & m = metrics[i];
    f << "  {\n"
      << "    \"id\": \""           << escape_json(m.id)          << "\",\n"
      << "    \"description\": \""  << escape_json(m.description)  << "\",\n"
      << "    \"result\": \""       << result_to_str(m.result)     << "\",\n"
      << "    \"time_to_goal_s\": " << fmt(m.time_to_goal_s)       << ",\n"
      << "    \"path_length_m\": "  << fmt(m.path_length_m)        << ",\n"
      << "    \"euclidean_dist_m\": " << fmt(m.euclidean_dist_m)   << ",\n"
      << "    \"path_efficiency\": " << fmt(m.path_efficiency, 3)  << ",\n"
      << "    \"replanning_count\": " << m.replanning_count        << ",\n"
      << "    \"min_clearance_m\": " << fmt(m.min_clearance_m)     << ",\n"
      << "    \"mean_velocity_ms\": " << fmt(m.mean_velocity_ms)   << ",\n"
      << "    \"std_velocity_ms\": " << fmt(m.std_velocity_ms)     << ",\n"
      << "    \"recovery_count\": " << m.recovery_count            << "\n"
      << "  }";
    if (i + 1 < metrics.size()) f << ",";
    f << "\n";
  }
  f << "]\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  Markdown writer
// ─────────────────────────────────────────────────────────────────────────────

void ReportGenerator::write_markdown(
  const std::vector<ScenarioMetrics> & metrics,
  const std::string & path) const
{
  std::ofstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open for writing: " + path);
  }

  // ── Header ────────────────────────────────────────────────────────────────
  f << "# bt_nav_eval — Closed-Loop Evaluation Report\n\n"
    << "**Generated:** " << current_timestamp() << "\n\n";

  // ── Aggregate stats ───────────────────────────────────────────────────────
  int success_count = 0, failure_count = 0, timeout_count = 0;
  double total_time = 0.0, total_eff = 0.0, total_clear = 0.0;
  int valid_eff = 0, valid_clear = 0;

  for (const auto & m : metrics) {
    if (m.result == ScenarioResult::SUCCESS) { ++success_count; total_time += m.time_to_goal_s; }
    else if (m.result == ScenarioResult::FAILURE) { ++failure_count; }
    else { ++timeout_count; }

    if (m.path_efficiency > 0.0) { total_eff += m.path_efficiency; ++valid_eff; }
    if (m.min_clearance_m > 0.0 && m.min_clearance_m < 100.0) {
      total_clear += m.min_clearance_m; ++valid_clear;
    }
  }

  const int total = static_cast<int>(metrics.size());
  const double success_rate = total > 0 ? (100.0 * success_count / total) : 0.0;
  const double mean_time    = success_count > 0 ? (total_time / success_count) : 0.0;
  const double mean_eff     = valid_eff > 0 ? (total_eff / valid_eff) : 0.0;
  const double mean_clear   = valid_clear > 0 ? (total_clear / valid_clear) : 0.0;

  f << "## Aggregate Summary\n\n"
    << "| Metric | Value |\n"
    << "|---|---|\n"
    << "| Total scenarios | " << total << " |\n"
    << "| **Success rate** | **" << fmt(success_rate, 1) << "%** |\n"
    << "| Success | " << success_count << " |\n"
    << "| Failure | " << failure_count << " |\n"
    << "| Timeout | " << timeout_count << " |\n"
    << "| Mean time to goal (success only) | " << fmt(mean_time) << " s |\n"
    << "| Mean path efficiency | " << fmt(mean_eff, 3) << " |\n"
    << "| Mean min clearance | " << fmt(mean_clear) << " m |\n"
    << "\n";

  // ── Per-scenario table ────────────────────────────────────────────────────
  f << "## Per-Scenario Results\n\n";
  f << "| ID | Description | Result | Time (s) | Path (m) | Efficiency | "
       "Min Clear (m) | Recoveries | Mean Vel (m/s) |\n";
  f << "|---|---|:---:|---:|---:|---:|---:|---:|---:|\n";

  for (const auto & m : metrics) {
    const char * badge = (m.result == ScenarioResult::SUCCESS) ? "✅ SUCCESS"
                       : (m.result == ScenarioResult::TIMEOUT) ? "⏱ TIMEOUT"
                                                                : "❌ FAILURE";
    const std::string clear_str =
      (m.min_clearance_m < 0.0 || m.min_clearance_m > 100.0)
        ? "N/A"
        : fmt(m.min_clearance_m);

    f << "| `" << m.id << "` "
      << "| " << m.description << " "
      << "| " << badge << " "
      << "| " << fmt(m.time_to_goal_s) << " "
      << "| " << fmt(m.path_length_m) << " "
      << "| " << fmt(m.path_efficiency, 3) << " "
      << "| " << clear_str << " "
      << "| " << m.recovery_count << " "
      << "| " << fmt(m.mean_velocity_ms) << " "
      << "|\n";
  }

  f << "\n";

  // ── Per-scenario notes ────────────────────────────────────────────────────
  f << "## Per-Scenario Notes\n\n";
  for (const auto & m : metrics) {
    f << "### `" << m.id << "` — " << m.description << "\n\n";
    f << "- **Result:** " << result_to_str(m.result) << "\n";
    f << "- **Time to goal:** " << fmt(m.time_to_goal_s) << " s\n";
    f << "- **Path length:** " << fmt(m.path_length_m) << " m  "
      << "(euclidean: " << fmt(m.euclidean_dist_m) << " m)\n";
    f << "- **Path efficiency:** " << fmt(m.path_efficiency, 3)
      << " (1.0 = straight-line optimal)\n";
    f << "- **Min clearance:** ";
    if (m.min_clearance_m < 0.0 || m.min_clearance_m > 100.0) {
      f << "N/A (no LiDAR data)\n";
    } else {
      f << fmt(m.min_clearance_m) << " m\n";
    }
    f << "- **Recovery activations:** " << m.recovery_count << "\n";
    f << "- **Velocity:** mean=" << fmt(m.mean_velocity_ms)
      << " m/s, std=" << fmt(m.std_velocity_ms) << " m/s\n";
    f << "\n";
  }

  // ── Footer ────────────────────────────────────────────────────────────────
  f << "---\n"
    << "*Generated by bt_nav_eval — "
       "closed-loop evaluation harness for Nav2 agents*\n";
}

// ─────────────────────────────────────────────────────────────────────────────

std::string ReportGenerator::escape_json(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '"')  { out += "\\\""; }
    else if (c == '\\') { out += "\\\\"; }
    else if (c == '\n') { out += "\\n"; }
    else if (c == '\r') { out += "\\r"; }
    else if (c == '\t') { out += "\\t"; }
    else { out += c; }
  }
  return out;
}

}  // namespace bt_nav_eval
