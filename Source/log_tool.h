/*
 * tool.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#include <iostream>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/support/date_time.hpp>

using namespace std;

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;

#define logInfo BOOST_LOG_TRIVIAL(info)
#define logDebug BOOST_LOG_TRIVIAL(debug)
#define logWarn BOOST_LOG_TRIVIAL(warning)
#define logError BOOST_LOG_TRIVIAL(error)

/// 0 debug
/// 1 info
//#define logLevel 0

enum severity_level {
	trace, debug, info, warning, error, fatal,
};

BOOST_LOG_ATTRIBUTE_KEYWORD(line_id, "LineID", unsigned int)
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", severity_level)
BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(timestamp, "TimeStamp", boost::posix_time::ptime)


void addInfoFilter() {
	logging::core::get()->set_filter(
			logging::trivial::severity >= logging::trivial::info);
}

void addDebugFilter() {
	logging::core::get()->set_filter(
			logging::trivial::severity >= logging::trivial::debug);
}
void addFilter(){
	if(logLevel == 0){
		logInfo << "level == debug";
		addDebugFilter();
	}else if (logLevel == 1){
		logInfo << "level == info";
		addInfoFilter();
	}
}

void addFileLog(std::string file_name) {

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

//	int main() {
//		BOOST_LOG_TRIVIAL(trace)<< "A trace severity message";
//		BOOST_LOG_TRIVIAL(debug)<< "A debug severity message";
//		BOOST_LOG_TRIVIAL(info)<< "An informational severity message";
//		BOOST_LOG_TRIVIAL(warning)<< "A warning severity message";
//		BOOST_LOG_TRIVIAL(error)<< "An error severity message";
//		BOOST_LOG_TRIVIAL(fatal)<< "A fatal severity message";
//	}

