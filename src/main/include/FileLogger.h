#pragma once

#include <fstream>
#include <iostream>

// Use the namespace you want
namespace tools
{

    enum class LogType
    {
        LOG_CRITICAL,
        LOG_ERROR,
        LOG_WARNING,
        LOG_NOTICE,
        LOG_INFO,
        LOG_DEBUG
    };

#define LL_DEBUG        tools::LogType::LOG_DEBUG,__LINE__, __FILE__, __FUNCTION__
#define LL_INFO         tools::LogType::LOG_INFO,__LINE__, __FILE__, __FUNCTION__
#define LL_NOTICE       tools::LogType::LOG_NOTICE,__LINE__, __FILE__, __FUNCTION__
#define LL_WARNING      tools::LogType::LOG_WARNING,__LINE__, __FILE__, __FUNCTION__
#define LL_ERROR        tools::LogType::LOG_ERROR,__LINE__, __FILE__, __FUNCTION__
#define LL_CRITICAL     tools::LogType::LOG_CRITICAL,__LINE__, __FILE__, __FUNCTION__

    class FileLogger
    {

    public:

        // ctor
        explicit FileLogger() : 
            numWarnings(0U),
            numErrors(0U),
            print_stdout(true),
            m_classname("")
        {
        }

        // ctor
        explicit FileLogger(const char *classname) : 
            numWarnings(0U),
            numErrors(0U),
            print_stdout(true),
            m_classname(classname)
        {
        }

        // ctor
        explicit FileLogger(const char *classname, const char *fname) : 
            numWarnings(0U),
            numErrors(0U),
            print_stdout(true),
            m_classname(classname)
        {
            open_file(fname);
        }

        // dtor
        ~FileLogger()
        {

            if (m_file.is_open())
            {
                m_file << std::endl
                       << std::endl;

                // Report number of errors and warnings
                m_file << numWarnings << " warnings" << std::endl;
                m_file << numErrors << " errors" << std::endl;

                m_file.close();
            } // if
        }

        void enable_stdout() { print_stdout = true; }

        void disable_stdout() { print_stdout = false; }

        bool open_file(const char *fname) 
        {
            if (fname == NULL)
            {
                return false;
            }
            m_file.open(fname);

            // Write the first lines
            if (m_file.is_open())
            {
                m_file << "My RIO Log File " << std::endl << std::endl;
                return true;
            }
            else
            {
                std::cout << "[ERROR] failed to open file: " << fname << std::endl;
                return false;
            }
        }

        char* timestamp(){
            time_t now;
            time(&now);

            strftime(m_timestamp, sizeof m_timestamp, "%F %TZ ", gmtime(&now));

            return m_timestamp;
        }

        void log(const LogType l_type, int lineno, const char *fileName, const char *fctName, const char *text) {
            *this << l_type;

            std::string data = "";

            // data += fileName;
            // data += "(" + std::to_string(lineno) + ") ";

            data += "[";
            
            if (!m_classname.empty())
            {
                data += m_classname + ".";
            }

            data += fctName;
            data += "(" + std::to_string(lineno) + ")";

            data += "] ";

            *this << data.c_str();
            *this << text << "\n";
        }

        void debug(const char *text) {
            *this << LogType::LOG_DEBUG;
            if (!m_classname.empty())
            {
                *this << "[" << m_classname.c_str() << "] ";

            }
            *this << text << "\n";
        }

        void info(const char *text) {
            *this << LogType::LOG_INFO;
            if (!m_classname.empty())
            {
                *this << "[" << m_classname.c_str() << "] ";

            }
            *this << text << "\n";
        }

        void notice(const char *text) {
            *this << LogType::LOG_NOTICE;
            if (!m_classname.empty())
            {
                *this << "[" << m_classname.c_str() << "] ";

            }
            *this << text << "\n";
        }

        void warning(const char *text) {
            *this << LogType::LOG_WARNING;
            if (!m_classname.empty())
            {
                *this << "[" << m_classname.c_str() << "] ";

            }
            *this << text << "\n";
        }

        void error(const char *text) {
            *this << LogType::LOG_ERROR;
            if (!m_classname.empty())
            {
                *this << "[" << m_classname.c_str() << "] ";

            }
            *this << text << "\n";
        }

        void critical(const char *text) {
            *this << LogType::LOG_CRITICAL;
            if (!m_classname.empty())
            {
                *this << "[" << m_classname.c_str() << "] ";

            }
            *this << text << "\n";
        }

        // Overload << operator using log type
        friend FileLogger &operator<<(FileLogger &logger, const LogType l_type)
        {

            if (logger.m_file.is_open())
            {
                logger.m_file << logger.timestamp();
            }
            
            if (logger.print_stdout)
            {
                std::cout << logger.timestamp();
            }

            switch (l_type)
            {
            case tools::LogType::LOG_ERROR:
                if (logger.m_file.is_open())
                {
                    logger.m_file << "[ERROR] ";
                }
                ++logger.numErrors;

                if (logger.print_stdout)
                {
                    std::cout << "[ERROR] ";
                }
                break;

            case tools::LogType::LOG_WARNING:
                if (logger.m_file.is_open())
                {
                    logger.m_file << "[WARNING] ";
                }
                ++logger.numWarnings;

                if (logger.print_stdout)
                {
                    std::cout << "[WARNING] ";
                }
                break;

            case tools::LogType::LOG_INFO:
            default:
                if (logger.m_file.is_open())
                {
                    logger.m_file << "[INFO] ";
                }

                if (logger.print_stdout)
                {
                    std::cout << "[INFO] ";
                }
                break;

            case tools::LogType::LOG_DEBUG:
                if (logger.m_file.is_open())
                {
                    logger.m_file << "[DEBUG] ";
                }

                if (logger.print_stdout)
                {
                    std::cout << "[INDEBUGFO] ";
                }
                break;
            }

            return logger;
        }

        // Overload << operator using C style strings
        // No need for std::string objects here
        friend FileLogger &operator<<(FileLogger &logger, const char *text)
        {

            if (logger.m_file.is_open())
            {
                logger.m_file << text;
            }

            if (logger.print_stdout)
            {
                std::cout << text;
            }

            return logger;
        }

        // Make it Non Copyable (or you can inherit from sf::NonCopyable if you want)
        FileLogger(const FileLogger &) = delete;
        FileLogger &operator=(const FileLogger &) = delete;

    private:
        std::ofstream m_file;

        unsigned int numWarnings;
        unsigned int numErrors;
        bool print_stdout;
        std::string m_classname;
        char m_timestamp[sizeof "2011-10-08T07:07:09Z    "];


    }; // class end

} // namespace
