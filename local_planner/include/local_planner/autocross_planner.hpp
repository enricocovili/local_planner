#ifndef AUTOCROSS_LOCAL_PLANNER_HPP
#define AUTOCROSS_LOCAL_PLANNER_HPP

#include <local_planner/generic_planner.hpp>


namespace local_planning {

class AutocrossPlanner : public GenericPlanner
{
private:
    std::string m_todo;
public:
    AutocrossPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) const override;
    void load_params() override;
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP