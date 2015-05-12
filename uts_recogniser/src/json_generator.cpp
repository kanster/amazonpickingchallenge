/** json file generator
  */


#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

int main( int argc, char** argv ) {
    std::vector<std::string> bin_ids;
    std::vector<std::string> items;
    items.push_back( "expo_dry_erase_board_eraser" );
    items.push_back( "kong_duck_dog_toy" );
    items.push_back( "genuine_joe_plastic_stir_sticks" );
    items.push_back( "munchkin_white_hot_duck_bath_toy" );
    items.push_back( "mommys_helper_outlet_plugs" );
    items.push_back( "sharpie_accent_tank_style_highlighters" );
    items.push_back( "kong_air_dog_squeakair_tennis_ball" );
    items.push_back( "stanley_66_052" );
    items.push_back( "safety_works_safety_glasses" );
    items.push_back( "dr_browns_bottle_brush" );
    items.push_back( "laugh_out_loud_joke_book" );
    items.push_back( "paper_mate_12_count_mirado_black_warrior" );
    items.push_back( "feline_greenies_dental_treats" );
    items.push_back( "elmers_washable_no_run_school_glue" );
    items.push_back( "mead_index_cards" );
    items.push_back( "highland_6539_self_stick_notes" );
    items.push_back( "mark_twain_huckleberry_finn" );
    items.push_back( "kyjen_squeakin_eggs_plush_puppies" );
    items.push_back( "kong_sitting_frog_dog_toy" );

    boost::property_tree::ptree root;
    boost::property_tree::ptree wo_tree;
    boost::property_tree::ptree bc_tree;
    srand(time(NULL));
    for ( int i = 0; i < 12; ++ i ) {
        int n_items = rand()%3+1;
        std::vector<std::string> bin_items;
        for ( int j = 0; j < n_items; ++ j )
            bin_items.push_back( items[rand()%(items.size())] );
        std::string bin_id = "bin_" + std::string(1, static_cast<char>(i+65));
        // generate bin content
        boost::property_tree::ptree bc;
        for ( int j = 0; j < (int)bin_items.size(); ++ j ) {
            bc.push_back( std::make_pair("", boost::property_tree::ptree(bin_items[j])) );
        }
        bc_tree.put_child(bin_id, bc);
        // generator work order
        boost::property_tree::basic_ptree<std::string, std::string> wo;
        wo.put<std::string>("bin", bin_id);
        wo.put<std::string>("item", bin_items[rand()%bin_items.size()]);
        wo_tree.push_back(std::make_pair("", wo));
    }
    root.put_child( "bin_contents", bc_tree );
    root.put_child( "work_order", wo_tree );
    boost::property_tree::json_parser::write_json( std::string(argv[1]), root );
    return 0;
}
