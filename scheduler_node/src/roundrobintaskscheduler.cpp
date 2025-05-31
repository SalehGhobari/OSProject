#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RoundRobinTaskScheduler : public rclcpp::Node
{
public:
    RoundRobinTaskScheduler(int time_quantum_ms = 500)
    : Node("RoundRobinTaskScheduler"), 
      current_index_(0), 
      time_quantum_(std::chrono::milliseconds(time_quantum_ms)),
      scheduler_started_(false)
    {
        // Declare parameter for time quantum
        this->declare_parameter("time_quantum_ms", time_quantum_ms);
        int param_quantum = this->get_parameter("time_quantum_ms").as_int();
        time_quantum_ = std::chrono::milliseconds(param_quantum);
        
        RCLCPP_INFO(this->get_logger(), "Round Robin Scheduler initialized with time quantum: %d ms", param_quantum);
        
        // Subscriber registration topic
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "register", 10,
            std::bind(&RoundRobinTaskScheduler::register_callback, this, std::placeholders::_1));
        
        // Publisher for granting time quantum to tasks
        grant_publisher_ = this->create_publisher<std_msgs::msg::String>("grant", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for task registrations...");
        
        // Start scheduler check timer (checks every 100ms if we should start scheduling)
        startup_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&RoundRobinTaskScheduler::startup_check, this)
        );
    }

private:
    void register_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string id = msg->data;
        if (unique_subscriber_ids_.insert(id).second) {
            subscribers_.push_back(id);
            RCLCPP_INFO(this->get_logger(), "Registered new task: %s (Total tasks: %zu)", 
                       id.c_str(), subscribers_.size());
            
            // If this is the first task and scheduler hasn't started, start it
            if (!scheduler_started_ && subscribers_.size() == 1) {
                start_scheduler();
            }
        }
    }
    
    void startup_check()
    {
        // This timer just ensures we don't miss starting the scheduler
        // The actual start happens in register_callback
        if (!subscribers_.empty() && !scheduler_started_) {
            start_scheduler();
        }
    }
    
    void start_scheduler()
    {
        if (scheduler_started_) return;
        
        scheduler_started_ = true;
        startup_timer_->cancel(); // Stop the startup check timer
        
        RCLCPP_INFO(this->get_logger(), "Starting round-robin scheduling with %zu tasks...", 
                   subscribers_.size());
        
        // Timer for round-robin scheduling
        scheduler_timer_ = this->create_wall_timer(
            time_quantum_, 
            std::bind(&RoundRobinTaskScheduler::schedule_next_task, this)
        );
    }
    
    void schedule_next_task()
    {
        if (subscribers_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No tasks to schedule.");
            return;
        }
        
        std::string current_task = subscribers_[current_index_];
        
        // Grant time quantum to current task
        auto grant_msg = std_msgs::msg::String();
        grant_msg.data = current_task;
        grant_publisher_->publish(grant_msg);
        
        RCLCPP_INFO(this->get_logger(), "Time quantum granted to: %s [%zu/%zu]", 
                   current_task.c_str(), current_index_ + 1, subscribers_.size());
        
        // Move to next task (round-robin)
        current_index_ = (current_index_ + 1) % subscribers_.size();
    }
    
    // Subscriber for task registration
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    
    // Publisher for granting time quantum
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grant_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr scheduler_timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    
    // Task management
    std::vector<std::string> subscribers_;
    std::unordered_set<std::string> unique_subscriber_ids_;
    
    // Scheduling state
    size_t current_index_;
    std::chrono::milliseconds time_quantum_;
    bool scheduler_started_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    int time_quantum = 500; // Default 500ms
    if (argc > 1) {
        try {
            time_quantum = std::stoi(argv[1]);
            if (time_quantum <= 0) {
                std::cerr << "Time quantum must be positive. Using default 500ms." << std::endl;
                time_quantum = 500;
            }
        } catch (const std::exception& e) {
            std::cerr << "Invalid time quantum argument. Using default 500ms." << std::endl;
            time_quantum = 500;
        }
    }
    
    auto scheduler = std::make_shared<RoundRobinTaskScheduler>(time_quantum);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Round Robin Scheduler...");
    rclcpp::spin(scheduler);
    rclcpp::shutdown();
    return 0;
}