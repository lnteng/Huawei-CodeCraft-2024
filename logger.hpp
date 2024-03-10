#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
using namespace std;

// 日志级别
enum LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class Logger {
public:
    Logger(const std::string& filename) : log_file_(filename, std::ios::app) {
        if (!log_file_.is_open()) {
            std::cerr << "Error: Unable to open log file." << std::endl;
        }
    }

    ~Logger() {
        log_file_.close();
    }

    // 写入日志
    void log(LogLevel level, const std::string& message) {
        if (log_file_.is_open()) {
            std::time_t now = std::time(nullptr);
            struct tm* timeinfo = std::localtime(&now);

            log_file_ << "[" << getFormattedTime(timeinfo) << "] ";

            switch (level) {
                case DEBUG:
                    log_file_ << "[DEBUG] ";
                    break;
                case INFO:
                    log_file_ << "[INFO] ";
                    break;
                case WARNING:
                    log_file_ << "[WARNING] ";
                    break;
                case ERROR:
                    log_file_ << "[ERROR] ";
                    break;
                default:
                    break;
            }

            log_file_ << message << std::endl;
        }
    }

private:
    std::ofstream log_file_;

    // 格式化时间
    std::string getFormattedTime(struct tm* timeinfo) {
        std::ostringstream oss;
        oss << std::put_time(timeinfo, "%Y-%m-%d %X");
        return oss.str();
    }
};