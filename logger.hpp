#pragma once
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
using namespace std;

// 日志级别
enum LogLevel
{
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class Logger
{
public:
    Logger(const std::string &filename) : log_file_(filename, std::ios::trunc)
    {
        if (!log_file_.is_open())
        {
            std::cerr << "Error: Unable to open log file." << std::endl;
        }
    }

    ~Logger()
    {
        log_file_.close();
    }

    // 写入日志
    void log(LogLevel level, const std::string &message)
    {
        if (log_file_.is_open())
        {
            std::time_t now = std::time(nullptr);
            struct tm *timeinfo = std::localtime(&now);

            log_file_ << "[" << getFormattedTime(timeinfo) << "] ";

            switch (level)
            {
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

    void log(const std::string &message)
    {
        this->log(INFO, message);
    }

private:
    std::ofstream log_file_;

    // 格式化时间
    std::string getFormattedTime(struct tm *timeinfo)
    {
        std::ostringstream oss;
        oss << std::put_time(timeinfo, "%Y-%m-%d %X");
        return oss.str();
    }
};

// 辅助函数，用于递归处理可变参数
void replacePlaceholdersHelper(std::string &result, size_t &pos, int arg)
{
    std::stringstream ss;
    ss << arg;
    result.replace(pos, 2, ss.str());
}

// 递归处理可变参数
template <typename... Args>
void replacePlaceholdersHelper(std::string &result, size_t &pos, int arg, Args... args)
{
    replacePlaceholdersHelper(result, pos, arg);

    // 查找下一个占位符的位置
    pos = result.find("{}", pos + 1);

    // 递归处理下一个参数
    replacePlaceholdersHelper(result, pos, args...);
}

// 可变参数的主函数
template <typename... Args>
std::string formatString(const std::string &templateStr, Args... args)
{
    std::string result = templateStr;
    size_t pos = result.find("{}");

    // 调用辅助函数处理可变参数
    replacePlaceholdersHelper(result, pos, args...);

    return result;
}