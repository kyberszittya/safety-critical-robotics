#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// Move base specific headers
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimpleMissionExecutionInterface
{
protected:
    std::vector<geometry_msgs::PoseStamped> goal_list;
    unsigned int goal_idx;
    move_base_msgs::MoveBaseGoal current_goal;
public:
    /**
     * Execute next goal
     * */
    virtual void executeGoal() = 0;

    /**
     * Select next goal from list
     * */
    virtual void selectNextGoal() = 0;

    /**
     * Add mission goal to list
     * */
    virtual void addGoal(geometry_msgs::PoseStamped p) = 0;
};


class SimpleMissionExecution: public SimpleMissionExecutionInterface
{
private:
    std::shared_ptr<MoveBaseClient> ac;
public:
    SimpleMissionExecution(){}

    bool init()
    {
        ac = std::make_shared<MoveBaseClient>("move_base", true);

    }

    virtual void addGoal(geometry_msgs::PoseStamped p)
    {
        goal_list.push_back(p);
    }

    virtual void selectNextGoal()
    {

    }

    virtual void executeGoal()
    {
        ac->sendGoal(current_goal);
        ac->waitForResult();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_action_move");
    return 0;
}