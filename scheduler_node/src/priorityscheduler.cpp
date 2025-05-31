#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct TaskInfo {
    std::string task_id;
    int priority;
    std::chrono::steady_clock::time_point registration_time;
    
    // For priority queue - higher priority values have higher precedence
    // If priorities are equal, earlier registration time has precedence
    bool operator<(const TaskInfo& other) const {
        if (priority != other.priority) {
            return priority < other.priority; // Higher priority value = higher precedence
        }
        return registration_time > other.registration_time; // Earlier time = higher precedence
    }
};

class PriorityTaskScheduler : public rclcpp::Node
{
public:
    PriorityTaskScheduler()
    : Node("PriorityTaskScheduler"), 
      scheduler_started_(false),
      current_executing_task_(""),
      task_executing_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Priority Scheduler initialized");
        
        // Subscriber for task registration (format: "task_id:priority")
        register_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "register", 10,
            std::bind(&PriorityTaskScheduler::register_callback, this, std::placeholders::_1));
        
        // Subscriber for task completion notifications
        complete_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "task_complete", 10,
            std::bind(&PriorityTaskScheduler::complete_callback, this, std::placeholders::_1));
        
        // Publisher for granting execution permission to tasks
        grant_publisher_ = this->create_publisher<std_msgs::msg::String>("grant", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for task registrations...");
        
        // Start scheduler check timer (checks every 100ms if we should start scheduling)
        startup_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&PriorityTaskScheduler::startup_check, this)
        );
    }

private:
    void register_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Parse message format: "task_id:priority"
        std::string data = msg->data;
        size_t colon_pos = data.find(':');
        
        if (colon_pos == std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "Invalid registration format: %s (expected task_id:priority)", data.c_str());
            return;
        }
        
        std::string task_id = data.substr(0, colon_pos);
        std::string priority_str = data.substr(colon_pos + 1);
        
        int priority;
        try {
            priority = std::stoi(priority_str);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Invalid priority value: %s", priority_str.c_str());
            return;
        }
        
        // Check if task already registered
        if (registered_tasks_.find(task_id) != registered_tasks_.end()) {
            RCLCPP_DEBUG(this->get_logger(), "Task %s already registered", task_id.c_str());
            return;
        }
        
        // Register the task
        TaskInfo task_info;
        task_info.task_id = task_id;
        task_info.priority = priority;
        task_info.registration_time = std::chrono::steady_clock::now();
        
        registered_tasks_[task_id] = task_info;
        task_queue_.push(task_info);
        
        RCLCPP_INFO(this->get_logger(), "Registered task: %s with priority: %d (Total tasks: %zu)", 
                   task_id.c_str(), priority, registered_tasks_.size());
        
        // If scheduler hasn't started and no task is executing, start scheduling
        if (!scheduler_started_ && !task_executing_) {
            start_scheduler();
        }
    }
    
    void complete_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string completed_task = msg->data;
        
        RCLCPP_INFO(this->get_logger(), "Task completed: %s", completed_task.c_str());
        
        // Mark that no task is currently executing
        if (current_executing_task_ == completed_task) {
            current_executing_task_ = "";
            task_executing_ = false;
            
            // Schedule next task immediately
            schedule_next_task();
        }
    }
    
    void startup_check()
    {
        // This timer ensures we start scheduling when tasks are available
        if (!task_queue_.empty() && !task_executing_ && !scheduler_started_) {
            start_scheduler();
        }
    }
    
    void start_scheduler()
    {
        if (scheduler_started_) return;
        
        scheduler_started_ = true;
        startup_timer_->cancel(); // Stop the startup check timer
        
        RCLCPP_INFO(this->get_logger(), "Starting priority-based scheduling with %zu tasks...", 
                   registered_tasks_.size());
        
        // Schedule the first task immediately
        schedule_next_task();
        
        // Timer for periodic scheduling check (in case of missed events)
        scheduler_timer_ = this->create_wall_timer(
            1s, 
            std::bind(&PriorityTaskScheduler::periodic_schedule_check, this)
        );
    }
    
    void schedule_next_task()
    {
        if (task_executing_) {
            RCLCPP_DEBUG(this->get_logger(), "Task currently executing, waiting for completion");
            return;
        }
        
        if (task_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No tasks in queue to schedule");
            return;
        }
        
        // Get highest priority task
        TaskInfo next_task = task_queue_.top();
        task_queue_.pop();
        
        // Verify task is still registered (in case it was removed)
        if (registered_tasks_.find(next_task.task_id) == registered_tasks_.end()) {
            RCLCPP_DEBUG(this->get_logger(), "Task %s no longer registered, skipping", next_task.task_id.c_str());
            schedule_next_task(); // Try next task
            return;
        }
        
        // Grant execution to the highest priority task
        current_executing_task_ = next_task.task_id;
        task_executing_ = true;
        
        auto grant_msg = std_msgs::msg::String();
        grant_msg.data = next_task.task_id;
        grant_publisher_->publish(grant_msg);
        
        RCLCPP_INFO(this->get_logger(), "Granted execution to task: %s (Priority: %d, Queue size: %zu)", 
                   next_task.task_id.c_str(), next_task.priority, task_queue_.size());
    }
    
    void periodic_schedule_check()
    {
        // Periodic check to ensure scheduling continues
        if (!task_executing_ && !task_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Periodic check: scheduling next task");
            schedule_next_task();
        }
        
        // Log current status
        if (task_executing_) {
            RCLCPP_DEBUG(this->get_logger(), "Currently executing: %s, Queue size: %zu", 
                        current_executing_task_.c_str(), task_queue_.size());
        } else if (task_queue_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No tasks in queue, waiting for registrations");
        }
    }
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr complete_subscriber_;
    
    // Publisher for granting execution permission
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grant_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr scheduler_timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    
    // Task management
    std::unordered_map<std::string, TaskInfo> registered_tasks_;
    std::priority_queue<TaskInfo> task_queue_;
    
    // Scheduling state
    bool scheduler_started_;
    std::string current_executing_task_;
    bool task_executing_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto scheduler = std::make_shared<PriorityTaskScheduler>();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Priority Scheduler...");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Tasks should register with format: 'task_id:priority'");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Higher priority numbers have higher precedence");
    
    rclcpp::spin(scheduler);
    rclcpp::shutdown();
    return 0;
}