#ifndef UTILS_HPP
#define UTILS_HPP

#include <cassert>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <map>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

using namespace std;
/** json file parser, specific for amazon picking challenge */
class JSON{
private:
    /** process */
    void process( stringstream & ss ) {
        boost::property_tree::ptree pt;
        boost::property_tree::json_parser::read_json(ss, pt);
        BOOST_FOREACH(boost::property_tree::ptree::value_type &l1, pt.get_child("")) {
            if ( l1.first == "bin_contents" ) {
                BOOST_FOREACH(boost::property_tree::ptree::value_type &l2, l1.second){
                    pair< string, vector<string> > bin_content;
                    bin_content.first = l2.first.data();
                    BOOST_FOREACH( boost::property_tree::ptree::value_type & l3, l2.second ) {
                        bin_content.second.push_back( l3.second.get_value<string>() );
                    }
                    bin_contents_.insert( bin_content );
                }
            }
            else if ( l1.first == "work_order" ) {
                BOOST_FOREACH(boost::property_tree::ptree::value_type &l2, l1.second.get_child("")) {
                    pair<string, string> single_work_order;
                    BOOST_FOREACH( boost::property_tree::ptree::value_type & l3, l2.second ) {
                        if ( l3.first == "bin" )
                            single_work_order.first = l3.second.get_value<string>();
                        if ( l3.first == "item" )
                            single_work_order.second = l3.second.get_value<string>();
                    }
                    work_order_.push_back( single_work_order );
                }
            }
        }
    }

public:
    map<string, vector<string> > bin_contents_;
    vector< pair<string, string> > work_order_;

    /** default constructor */
    JSON() {}

    /** load json from disk */
    void from_file( const string & filename ) {
        ifstream in( filename.c_str() );
        stringstream ss;
        ss << in.rdbuf();
        from_sstream( ss );
    }

    /** load json from string */
    void from_sstream( stringstream & ss ) {
        process(ss);
    }

    friend ostream & operator << ( ostream & out, const JSON & json ) {
        typedef map< string, vector<string> >::const_iterator bin_content_iterator;
        out << "bin contents:\n";
        for ( bin_content_iterator iter = json.bin_contents_.begin(); iter != json.bin_contents_.end(); ++ iter ) {
            out << "\t" << iter->first << " : ";
            for ( size_t i = 0; i < iter->second.size(); ++ i )
                out << iter->second[i] << " ";
            out << "\n";
        }
        out << "work order:\n";
        for ( size_t i = 0; i < json.work_order_.size(); ++ i ) {
            out << "\t" << json.work_order_[i].first << " : " << json.work_order_[i].second << "\n";
        }
        return out;
    }
};


#endif // UTILS_HPP
