/*
 * tool.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#ifndef _LOG_TOOL_H_
#define _LOG_TOOL_H_

#include <iostream>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/console.hpp>
using namespace std;

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;


///* log formatter:
//   * [TimeStamp] [ThreadId] [Severity Level] [Scope] Log message
//   */
//  auto fmtTimeStamp = boost::log::expressions::
//      format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f");
//  auto fmtThreadId = boost::log::expressions::
//      attr<boost::log::attributes::current_thread_id::value_type>("ThreadID");
//  auto fmtSeverity = boost::log::expressions::
//      attr<boost::log::trivial::severity_level>("Severity");
//  auto fmtScope = boost::log::expressions::format_named_scope("Scope",
//      boost::log::keywords::format = "%n(%f:%l)",
//      boost::log::keywords::iteration = boost::log::expressions::reverse,
//      boost::log::keywords::depth = 2);
//  boost::log::formatter logFmt =
//      boost::log::expressions::format("[%1%] (%2%) [%3%] [%4%] %5%")
//      % fmtTimeStamp % fmtThreadId % fmtSeverity % fmtScope
//      % boost::log::expressions::smessage;

  /* console sink */
//  auto consoleSink = logging::add_console_log(std::clog);
//  consoleSink->set_formatter(logFmt);

#define logInfo BOOST_LOG_TRIVIAL(info)
#define logDebug BOOST_LOG_TRIVIAL(debug)
#define logWarn BOOST_LOG_TRIVIAL(warning)
#define logError BOOST_LOG_TRIVIAL(error)

/**
 * 日志等级，支持宏定义
 * 0 debug 1 info
 */
//#define logLevel 0

enum severity_level {
	trace, debug, info, warning, error, fatal,
};

//BOOST_LOG_ATTRIBUTE_KEYWORD(line_id, "LineID", unsigned int)
//BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", severity_level)
//BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string)
//BOOST_LOG_ATTRIBUTE_KEYWORD(timestamp, "TimeStamp", boost::posix_time::ptime)


/**
 * info log filter
 */
inline void addInfoFilter() {
	logging::core::get()->set_filter(
			logging::trivial::severity >= logging::trivial::info);
}

inline void addDebugFilter() {
	logging::core::get()->set_filter(
			logging::trivial::severity >= logging::trivial::debug);
}
inline void addFilter(){
	if(logLevel == 0){
		logInfo << "level == debug";
		addDebugFilter();
	}else if (logLevel == 1){
		logInfo << "level == info";
		addInfoFilter();
	}
}
//boost::log::add_console_log(std::cout, boost::log::keywords::format = ">> %Message%");
inline void addFileLog(std::string file_path) {

	logging::add_common_attributes();

	logging::add_file_log(keywords::file_name = "/tmp/NSeNavigation%N.log",
			keywords::rotation_size = 10 * 1024 * 1024,		//rotate every 10 MB
			keywords::time_based_rotation = sinks::file::rotation_at_time_point(
					0, 0, 0),			//rotate at midnight
			keywords::format = (expr::stream << std::setw(8)
					<< std::setfill('0') << expr::attr<unsigned int>("LineID")
					<< "\t"
					<< expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%H:%M:%S.%f") << "\t: <"
					<< logging::trivial::severity << "> \t" << expr::smessage

					));
}
#endif
//	int main() {
//		BOOST_LOG_TRIVIAL(trace)<< "A trace severity message";
//		BOOST_LOG_TRIVIAL(debug)<< "A debug severity message";
//		BOOST_LOG_TRIVIAL(info)<< "An informational severity message";
//		BOOST_LOG_TRIVIAL(warning)<< "A warning severity message";
//		BOOST_LOG_TRIVIAL(error)<< "An error severity message";
//		BOOST_LOG_TRIVIAL(fatal)<< "A fatal severity message";
//	}

