#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DummyTaskNode : public rclcpp::Node
{
public:
    DummyTaskNode(const std::string & task_id)
    : Node(task_id), 
      task_id_(task_id),
      execution_count_(0),
      random_generator_(std::random_device{}())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing task: %s", task_id_.c_str());
        
        // Create publisher for registration
        register_publisher_ = this->create_publisher<std_msgs::msg::String>("register", 10);
        
        // Create publisher for task completion notifications
        completion_publisher_ = this->create_publisher<std_msgs::msg::String>("task_complete", 10);
        
        // Subscribe to grant topic to receive time quantum allocations
        grant_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "grant", 10,
            std::bind(&DummyTaskNode::grant_callback, this, std::placeholders::_1));
        
        // Register with scheduler after a brief delay to ensure scheduler is ready
        register_timer_ = this->create_wall_timer(
            1s, 
            std::bind(&DummyTaskNode::register_with_scheduler, this)
        );
        
        // Periodic status update
        status_timer_ = this->create_wall_timer(
            15s, 
            std::bind(&DummyTaskNode::status_update, this)
        );
    }

private:
    void register_with_scheduler()
    {
        auto registration_msg = std_msgs::msg::String();
        registration_msg.data = task_id_;
        
        RCLCPP_INFO(this->get_logger(), "Registering with scheduler as '%s'", task_id_.c_str());
        register_publisher_->publish(registration_msg);
        
        // Cancel the registration timer after first registration
        register_timer_->cancel();
    }
    
    void grant_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == task_id_) {
            execution_count_++;
            RCLCPP_INFO(this->get_logger(), 
                       "Received time quantum #%d! Executing task...", 
                       execution_count_);
            
            // Record execution start time
            auto execution_start = std::chrono::steady_clock::now();
            
            // Simulate some work
            execute_dummy_work();
            
            // Record execution end time
            auto execution_end = std::chrono::steady_clock::now();
            auto execution_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                execution_end - execution_start).count();
            
            RCLCPP_INFO(this->get_logger(), "Task execution completed in %ldms", execution_duration);
            
            // Notify scheduler about completion
            notify_completion();
        }
    }
    
    void execute_dummy_work()
    {
        // Simulate different types of work with random duration
        std::uniform_int_distribution<int> work_type_dist(1, 3);
        std::uniform_int_distribution<int> duration_dist(50, 300);
        
        int work_type = work_type_dist(random_generator_);
        int work_duration = duration_dist(random_generator_);
        
        switch (work_type) {
            case 1:
                RCLCPP_INFO(this->get_logger(), "  -> Processing data simulation (%dms)", work_duration);
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "  -> Computing algorithm simulation (%dms)", work_duration);
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "  -> I/O operation simulation (%dms)", work_duration);
                break;
        }
        
        // Simulate work by sleeping
        std::this_thread::sleep_for(std::chrono::milliseconds(work_duration));
        
        // Simulate some computational work
        volatile int dummy_computation = 0;
        for (int i = 0; i < 50000; ++i) {
            dummy_computation += i * i;
        }
    }
    
    void notify_completion()
    {
        auto completion_msg = std_msgs::msg::String();
        completion_msg.data = task_id_;
        completion_publisher_->publish(completion_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Notified scheduler of completion");
    }
    
    void status_update()
    {
        RCLCPP_INFO(this->get_logger(), 
                   "[%s] Status: Alive | Executions: %d | Waiting for scheduler...", 
                   task_id_.c_str(), execution_count_);
    }
    
    // Node identification
    std::string task_id_;
    
    // Communication
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr register_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr completion_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grant_subscriber_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr register_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Task state
    int execution_count_;
    std::mt19937 random_generator_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::string task_name = "task_node";
    if (argc > 1) {
        task_name = argv[1];
    }
    
    // Add some uniqueness if no custom name provided
    if (task_name == "task_node") {
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        task_name += "_" + std::to_string(timestamp % 10000);
    }
    
    auto node = std::make_shared<DummyTaskNode>(task_name);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting dummy task: %s", task_name.c_str());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}