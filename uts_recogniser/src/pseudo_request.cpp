#include "include/json_parser.hpp"

#include <apc_msgs/TargetRequest.h>

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>

using namespace std;

// json configuration file
const string g_json_filename    = "../data/amazon.json";


int ms_sleep( unsigned long ms ) {
    struct timespec req = {0};
    time_t sec = (int)(ms/1000);
    ms -= sec*1000;
    req.tv_sec  = sec;
    req.tv_nsec = ms*1000000L;
    while ( nanosleep( &req, &req ) == -1 )
        continue;
    return 1;
}

int main( int argc, char ** argv ) {
    // default object list, named from berkeley object dataset
    string names[] = {
        "champion_copper_plus_spark_plug",
        "cheezit_big_original",
        "crayola_64_ct",
        "dove_beauty_bar",
        "elmers_washable_no_run_school_glue",
        "expo_dry_erase_board_eraser",
        "feline_greenies_dental_treats",
        "first_years_take_and_toss_straw_cups",
        "genuine_joe_plastic_stir_sticks",
        "highland_6539_self_stick_notes",
        "kong_air_dog_squeakair_tennis_ball",
        "kong_duck_dog_toy",
        "kong_sitting_frog_dog_toy",
        "kygen_squeakin_eggs_plush_puppies",
        "mark_twain_huckleberry_finn",
        "mead_index_cards",
        "mommys_helper_outlet_plugs",
        "munchkin_white_hot_duck_bath_toy",
        "one_with_nature_soap_dead_sea_mud",
        "oreo_mega_stuf",
        "paper_mate_12_count_mirado_black_warrior",
        "rollodex_mesh_collection_jumbo_pencil_cup",
        "safety_works_safety_glasses",
        "sharpie_accent_tank_style_highlighters",
        "stanley_66_052"
    };

    string uts_names[] = {
        "fruity_bites",
        "belvita_honey",
        "oldel_paso",
        "belvita_milk",
        "moroccan_beef",
        "sanitarium",
        "sultana_bran",
        "yoghurt_topps"
    };

    string tmp_names[] = {
        "dr_browns_bottle_rush",
        "expo_dry_erase_board_eraser",
        "genuine_joe_plastic_stir_sticks",
        "highland_6539_self_stick_notes",
        "laugh_out_loud_joke_book",
        "mark_twain_huckleberry_finn",
        "mead_index_cards",
        "mommys_helper_outlet_plugs",
        "paper_mate_12_count_mirado_black_warrior",
        "stanley_66_052"
    };

    // parse json files
    JSON json;
    json.from_file( g_json_filename );
    map<string, vector<string> >  bin_contents   = json.bin_contents_;
    vector<pair<string, string> > work_order     = json.work_order_;


    vector<string> object_names( uts_names, uts_names + sizeof(uts_names)/sizeof(string) );

    ros::init(argc, argv, "pseudo_request");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<apc_msgs::TargetRequest>("target_object_srv");
    apc_msgs::TargetRequest target_srv;
    int random_index;

    while ( ros::ok() ) {
        random_index = rand()%work_order.size();

        target_srv.request.ObjectIndex = random_index;
        target_srv.request.ObjectName  = work_order[random_index].second;
        vector<int8_t> removed_indices;
        target_srv.request.RemovedObjectIndices = removed_indices;
        ROS_INFO( "request object %d with name %s, sending ...", target_srv.request.ObjectIndex, target_srv.request.ObjectName.c_str() );
       if ( client.call( target_srv ) ) {
            ROS_INFO( "return status: %s", target_srv.response.Found? "true" : "false" );
        }
        else {
            ROS_ERROR( "Target object: %s, failed to call service target_object", target_srv.request.ObjectName.c_str() );
        }
        ms_sleep(50L);
    }
    return 0;
}


