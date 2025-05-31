#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <unordered_set>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class FCFSTaskScheduler : public rclcpp::Node
{
public:
    FCFSTaskScheduler()
    : Node("FCFSTaskScheduler"), 
      current_task_executing_(false),
      scheduler_started_(false)
    {
        RCLCPP_INFO(this->get_logger(), "FCFS (First-Come-First-Served) Scheduler initialized");
        
        // Subscriber for task registration
        register_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "register", 10,
            std::bind(&FCFSTaskScheduler::register_callback, this, std::placeholders::_1));
        
        // Subscriber for task completion notifications
        complete_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "task_complete", 10,
            std::bind(&FCFSTaskScheduler::complete_callback, this, std::placeholders::_1));
        
        // Publisher for granting execution to tasks
        grant_publisher_ = this->create_publisher<std_msgs::msg::String>("grant", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for task registrations...");
        
        // Timer to check for tasks to schedule
        scheduler_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&FCFSTaskScheduler::schedule_next_task, this)
        );
        
        // Status timer for logging
        status_timer_ = this->create_wall_timer(
            5s,
            std::bind(&FCFSTaskScheduler::print_status, this)
        );
    }

private:
    void register_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        std::string task_id = msg->data;
        
        // Check if task already registered
        if (registered_tasks_.find(task_id) != registered_tasks_.end()) {
            RCLCPP_WARN(this->get_logger(), "Task %s already registered, ignoring duplicate registration", 
                       task_id.c_str());
            return;
        }
        
        // Add to registered tasks and ready queue
        registered_tasks_.insert(task_id);
        ready_queue_.push(task_id);
        
        RCLCPP_INFO(this->get_logger(), "📝 Registered task: %s (Queue position: %zu)", 
                   task_id.c_str(), ready_queue_.size());
        
        if (!scheduler_started_) {
            scheduler_started_ = true;
            RCLCPP_INFO(this->get_logger(), "🚀 FCFS Scheduler started!");
        }
    }
    
    void complete_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        std::string completed_task = msg->data;
        
        if (current_executing_task_ == completed_task) {
            RCLCPP_INFO(this->get_logger(), "✅ Task %s completed execution", completed_task.c_str());
            current_task_executing_ = false;
            current_executing_task_.clear();
            
            // Update statistics
            completed_tasks_.insert(completed_task);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received completion from unexpected task: %s", 
                       completed_task.c_str());
        }
    }
    
    void schedule_next_task()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // If no task is currently executing and we have tasks in queue
        if (!current_task_executing_ && !ready_queue_.empty()) {
            std::string next_task = ready_queue_.front();
            ready_queue_.pop();
            
            // Start executing the next task
            current_task_executing_ = true;
            current_executing_task_ = next_task;
            
            RCLCPP_INFO(this->get_logger(), "🎯 Granting execution to task: %s (Queue remaining: %zu)", 
                       next_task.c_str(), ready_queue_.size());
            
            // Send grant message
            auto grant_msg = std_msgs::msg::String();
            grant_msg.data = next_task;
            grant_publisher_->publish(grant_msg);
        }
    }
    
    void print_status()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (scheduler_started_) {
            RCLCPP_INFO(this->get_logger(), 
                       "📊 FCFS Status - Registered: %zu | Queue: %zu | Executing: %s | Completed: %zu",
                       registered_tasks_.size(),
                       ready_queue_.size(),
                       current_task_executing_ ? current_executing_task_.c_str() : "None",
                       completed_tasks_.size());
        }
    }
    
    // Communication
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr complete_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grant_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr scheduler_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Task management
    std::queue<std::string> ready_queue_;
    std::unordered_set<std::string> registered_tasks_;
    std::unordered_set<std::string> completed_tasks_;
    
    // Execution state
    bool current_task_executing_;
    std::string current_executing_task_;
    bool scheduler_started_;
    
    // Thread safety
    std::mutex queue_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto scheduler = std::make_shared<FCFSTaskScheduler>();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting FCFS Task Scheduler...");
    rclcpp::spin(scheduler);
    rclcpp::shutdown();
    return 0;
}