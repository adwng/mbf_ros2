#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace TUI {
const std::string RESET = "\033[0m";
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[1;34m";
const std::string CYAN = "\033[1;36m";
const std::string GRAY = "\033[90m";

class Dashboard {
 private:
  std::stringstream current_row_ss;
  std::vector<std::string> rows;
  bool first_item_in_row = true;
  inline static int prev_line_count;
  int last_line_count = 0;

 public:
  Dashboard() {}

  enum class LogLevel { INFO, WARN, ERROR };

  template <typename T>
  void add_field(const std::string& label, const T& value,
                 const std::string& color = RESET, int width = 0) {
    add_separator();
    current_row_ss << color << label << ": ";
    if (width > 0) current_row_ss << std::left << std::setw(width);
    current_row_ss << value << RESET;
  }
  template <typename VecT>
  void add_vector(const std::string& label, const VecT& vec,
                  const std::string& color = GREEN) {
    add_separator();
    current_row_ss << color << label << ": [";
    current_row_ss << std::fixed << std::setprecision(2);

    const auto n = std::size(vec);
    for (size_t i = 0; i < n; ++i) {
      current_row_ss << vec[i];
      if (i + 1 < n) current_row_ss << ", ";
    }

    current_row_ss << "]" << RESET;
  }
  void add_flags(const std::string& label,
                 const std::vector<std::pair<std::string, bool>>& flags) {
    add_separator();
    current_row_ss << label << ": [";
    for (const auto& flag : flags) {
      current_row_ss << (flag.second ? GREEN : GRAY) << flag.first << RESET
                     << " ";
    }
    current_row_ss << "]";
  }
  void new_row() {
    // Push current buffer to rows vector
    rows.push_back(current_row_ss.str());

    // Reset buffer for next row
    current_row_ss.str("");
    current_row_ss.clear();
    first_item_in_row = true;
  }
  void print() {
    // Step 0: erase previous dashboard
    if (last_line_count > 0) {
      std::cout << "\033[" << last_line_count << "A";
      for (int i = 0; i < last_line_count; ++i) {
        std::cout << "\033[2K"
                  << "\033[1B";
      }
      std::cout << "\033[" << last_line_count << "A";
    }

    // Step 1: print new dashboard./
    for (const auto& row : rows) {
      std::cout << row << "\n";
    }

    // Print log section
    if (!log_buffer.empty()) {
      // std::cout << TUI::GRAY << "---- logs ----" << TUI::RESET << "\n";
      for (const auto& log : log_buffer) {
        std::cout << log << "\n";
      }
    }
    std::cout << std::flush;

    // Step 2: update last_line_count for next frame
    last_line_count =
        rows.size() + (log_buffer.empty() ? 0 : log_buffer.size() + 1);

    // Step 3: clear current rows to avoid accumulation
    rows.clear();
    current_row_ss.str("");
    current_row_ss.clear();
    first_item_in_row = true;
  }

  static void log(const std::string& msg, LogLevel lvl = LogLevel::INFO) {
    std::stringstream ss;

    switch (lvl) {
      case LogLevel::INFO:
        ss << CYAN << "[INFO] ";
        break;
      case LogLevel::WARN:
        ss << YELLOW << "[WARN] ";
        break;
      case LogLevel::ERROR:
        ss << RED << "[ERR ] ";
        break;
    }

    ss << msg << RESET;

    log_buffer.push_back(ss.str());

    if (log_buffer.size() > MAX_LOG_LINES) {
      log_buffer.erase(log_buffer.begin());
    }
  }

 private:
  void add_separator() {
    if (!first_item_in_row) current_row_ss << " | ";
    first_item_in_row = false;
  }

  static constexpr size_t MAX_LOG_LINES = 6;
  inline static std::vector<std::string> log_buffer;
};
}  // namespace TUI