#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <mutex>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct FCFSTaskMetrics {
    std::chrono::steady_clock::time_point arrival_time;
    std::chrono::steady_clock::time_point execution_start_time;
    std::chrono::steady_clock::time_point execution_end_time;
    bool execution_started = false;
    bool execution_completed = false;
    int queue_position = 0;
};

class FCFSTaskScheduler : public rclcpp::Node
{
public:
    FCFSTaskScheduler()
    : Node("FCFSTaskScheduler"), 
    task_counter_(0),
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
            10s,
            std::bind(&FCFSTaskScheduler::print_status, this)
        );
        
        // Metrics reporting timer
        metrics_timer_ = this->create_wall_timer(
            30s,
            std::bind(&FCFSTaskScheduler::report_metrics, this)
        );
    }

private:
    void register_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        std::string task_id = msg->data;
        auto now = std::chrono::steady_clock::now();
        
        // Check if task already registered
        if (registered_tasks_.find(task_id) != registered_tasks_.end()) {
            RCLCPP_WARN(this->get_logger(), "Task %s already registered, ignoring duplicate registration", 
                       task_id.c_str());
            return;
        }
        
        // Initialize metrics for this task
        FCFSTaskMetrics metrics;
        metrics.arrival_time = now;
        metrics.queue_position = ++task_counter_;
        task_metrics_[task_id] = metrics;
        
        // Add to registered tasks and ready queue
        registered_tasks_.insert(task_id);
        ready_queue_.push(task_id);
        
        RCLCPP_INFO(this->get_logger(), "Registered task: %s (Queue position: %d, Total in queue: %zu)", 
                   task_id.c_str(), metrics.queue_position, ready_queue_.size());
        
        if (!scheduler_started_) {
            scheduler_started_ = true;
            RCLCPP_INFO(this->get_logger(), "FCFS Scheduler started!");
        }
    }
    
    void complete_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        std::string completed_task = msg->data;
        auto now = std::chrono::steady_clock::now();
        
        if (current_executing_task_ == completed_task) {
            RCLCPP_INFO(this->get_logger(), "Task %s completed execution", completed_task.c_str());
            
            // Update metrics
            if (task_metrics_.find(completed_task) != task_metrics_.end()) {
                task_metrics_[completed_task].execution_end_time = now;
                task_metrics_[completed_task].execution_completed = true;
            }
            
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
            
            auto now = std::chrono::steady_clock::now();
            
            // Update metrics
            if (task_metrics_.find(next_task) != task_metrics_.end()) {
                task_metrics_[next_task].execution_start_time = now;
                task_metrics_[next_task].execution_started = true;
            }
            
            // Start executing the next task
            current_task_executing_ = true;
            current_executing_task_ = next_task;
            
            RCLCPP_INFO(this->get_logger(), "Granting execution to task: %s (Queue remaining: %zu)", 
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
                       "FCFS Status - Registered: %zu | Queue: %zu | Executing: %s | Completed: %zu",
                       registered_tasks_.size(),
                       ready_queue_.size(),
                       current_task_executing_ ? current_executing_task_.c_str() : "None",
                       completed_tasks_.size());
        }
    }
    
    void report_metrics()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (task_metrics_.empty()) {
            return;
        }
        
        std::vector<double> waiting_times;
        std::vector<double> turnaround_times;
        std::vector<double> response_times;
        
        auto current_time = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "=== FCFS PERFORMANCE METRICS REPORT ===");
        
        for (const auto& [task_id, metrics] : task_metrics_) {
            if (!metrics.execution_started) {
                // Task hasn't been scheduled yet
                auto waiting_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - metrics.arrival_time).count();
                
                RCLCPP_INFO(this->get_logger(), 
                           "Task %s: Position=%d, Still waiting (%.1fms so far)",
                           task_id.c_str(), metrics.queue_position, (double)waiting_time_ms);
                continue;
            }
            
            // Response time: time from arrival to start of execution
            auto response_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                metrics.execution_start_time - metrics.arrival_time).count();
            response_times.push_back(response_time_ms);
            
            // Waiting time: same as response time in FCFS (no preemption)
            waiting_times.push_back(response_time_ms);
            
            // Turnaround time: time from arrival to completion
            double turnaround_time_ms;
            if (metrics.execution_completed) {
                turnaround_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    metrics.execution_end_time - metrics.arrival_time).count();
            } else {
                // Task is still executing
                turnaround_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - metrics.arrival_time).count();
            }
            turnaround_times.push_back(turnaround_time_ms);
            
            // Execution time (if completed)
            double execution_time_ms = 0;
            if (metrics.execution_completed) {
                execution_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    metrics.execution_end_time - metrics.execution_start_time).count();
            }
            
            std::string status = metrics.execution_completed ? "Completed" : "Executing";
            
            RCLCPP_INFO(this->get_logger(), 
                       "Task %s: Position=%d, Status=%s, Response=%.1fms, Waiting=%.1fms, Turnaround=%.1fms, Execution=%.1fms",
                       task_id.c_str(), metrics.queue_position, status.c_str(),
                       (double)response_time_ms, (double)response_time_ms, 
                       turnaround_time_ms, execution_time_ms);
        }
        
        // Calculate and report averages
        if (!response_times.empty()) {
            double avg_response = std::accumulate(response_times.begin(), response_times.end(), 0.0) / response_times.size();
            double avg_waiting = std::accumulate(waiting_times.begin(), waiting_times.end(), 0.0) / waiting_times.size();
            double avg_turnaround = std::accumulate(turnaround_times.begin(), turnaround_times.end(), 0.0) / turnaround_times.size();
            
            RCLCPP_INFO(this->get_logger(), "AVERAGES: Response=%.1fms, Waiting=%.1fms, Turnaround=%.1fms",
                       avg_response, avg_waiting, avg_turnaround);
        }
        
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }
    
    // Communication
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr complete_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grant_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr scheduler_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;
    
    // Task management
    std::queue<std::string> ready_queue_;
    std::unordered_set<std::string> registered_tasks_;
    std::unordered_set<std::string> completed_tasks_;
    
    // Metrics tracking
    std::unordered_map<std::string, FCFSTaskMetrics> task_metrics_;
    int task_counter_;
    
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