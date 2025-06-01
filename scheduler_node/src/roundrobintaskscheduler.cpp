#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct TaskMetrics {
    std::chrono::steady_clock::time_point arrival_time;
    std::chrono::steady_clock::time_point first_execution_time;
    std::chrono::steady_clock::time_point last_execution_time;
    std::vector<std::chrono::steady_clock::time_point> execution_starts;
    std::vector<std::chrono::steady_clock::time_point> execution_ends;
    int execution_count = 0;
    bool first_execution_received = false;
};

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
        
        // Subscriber for task completion notifications
        completion_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "task_complete", 10,
            std::bind(&RoundRobinTaskScheduler::completion_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Waiting for task registrations...");
        
        // Start scheduler check timer (checks every 100ms if we should start scheduling)
        startup_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&RoundRobinTaskScheduler::startup_check, this)
        );
        
        // Metrics reporting timer (every 30 seconds)
        metrics_timer_ = this->create_wall_timer(
            30s,
            std::bind(&RoundRobinTaskScheduler::report_metrics, this)
        );
    }

private:
    void register_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string id = msg->data;
        if (unique_subscriber_ids_.insert(id).second) {
            subscribers_.push_back(id);
            
            // Initialize metrics for this task
            TaskMetrics metrics;
            metrics.arrival_time = std::chrono::steady_clock::now();
            task_metrics_[id] = metrics;
            
            RCLCPP_INFO(this->get_logger(), "Registered new task: %s (Total tasks: %zu)", 
                       id.c_str(), subscribers_.size());
            
            // If this is the first task and scheduler hasn't started, start it
            if (!scheduler_started_ && subscribers_.size() == 1) {
                start_scheduler();
            }
        }
    }
    
    void completion_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string task_id = msg->data;
        auto now = std::chrono::steady_clock::now();
        
        if (task_metrics_.find(task_id) != task_metrics_.end()) {
            task_metrics_[task_id].execution_ends.push_back(now);
            task_metrics_[task_id].last_execution_time = now;
            
            RCLCPP_DEBUG(this->get_logger(), "Task %s completed execution", task_id.c_str());
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
        auto now = std::chrono::steady_clock::now();
        
        // Update metrics for this task
        if (task_metrics_.find(current_task) != task_metrics_.end()) {
            auto& metrics = task_metrics_[current_task];
            
            if (!metrics.first_execution_received) {
                metrics.first_execution_time = now;
                metrics.first_execution_received = true;
            }
            
            metrics.execution_starts.push_back(now);
            metrics.execution_count++;
        }
        
        // Grant time quantum to current task
        auto grant_msg = std_msgs::msg::String();
        grant_msg.data = current_task;
        grant_publisher_->publish(grant_msg);
        
        RCLCPP_INFO(this->get_logger(), "Time quantum granted to: %s [%zu/%zu]", 
                   current_task.c_str(), current_index_ + 1, subscribers_.size());
        
        // Move to next task (round-robin)
        current_index_ = (current_index_ + 1) % subscribers_.size();
    }
    
    void report_metrics()
    {
        if (task_metrics_.empty()) {
            return;
        }
        
        std::vector<double> waiting_times;
        std::vector<double> turnaround_times;
        std::vector<double> response_times;
        
        auto current_time = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "=== PERFORMANCE METRICS REPORT ===");
        
        for (const auto& [task_id, metrics] : task_metrics_) {
            if (!metrics.first_execution_received) {
                continue; // Skip tasks that haven't been scheduled yet
            }
            
            // Response time: time from arrival to first execution
            auto response_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                metrics.first_execution_time - metrics.arrival_time).count();
            response_times.push_back(response_time_ms);
            
            // Calculate waiting time (total time spent waiting between executions)
            double total_waiting_time_ms = response_time_ms; // Initial wait time
            
            for (size_t i = 1; i < metrics.execution_starts.size(); ++i) {
                if (i - 1 < metrics.execution_ends.size()) {
                    auto wait_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        metrics.execution_starts[i] - metrics.execution_ends[i-1]).count();
                    total_waiting_time_ms += wait_time;
                }
            }
            
            waiting_times.push_back(total_waiting_time_ms);
            
            // Turnaround time: total time from arrival to last completion
            double turnaround_time_ms;
            if (!metrics.execution_ends.empty()) {
                turnaround_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    metrics.execution_ends.back() - metrics.arrival_time).count();
            } else {
                // If no completion recorded, use current time
                turnaround_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - metrics.arrival_time).count();
            }
            turnaround_times.push_back(turnaround_time_ms);
            
            RCLCPP_INFO(this->get_logger(), 
                       "Task %s: Executions=%d, Response=%.1fms, Waiting=%.1fms, Turnaround=%.1fms",
                       task_id.c_str(), metrics.execution_count, 
                       (double)response_time_ms, total_waiting_time_ms, turnaround_time_ms);
        }
        
        if (!waiting_times.empty()) {
            double avg_waiting = std::accumulate(waiting_times.begin(), waiting_times.end(), 0.0) / waiting_times.size();
            double avg_turnaround = std::accumulate(turnaround_times.begin(), turnaround_times.end(), 0.0) / turnaround_times.size();
            double avg_response = std::accumulate(response_times.begin(), response_times.end(), 0.0) / response_times.size();
            
            RCLCPP_INFO(this->get_logger(), "AVERAGES: Response=%.1fms, Waiting=%.1fms, Turnaround=%.1fms",
                       avg_response, avg_waiting, avg_turnaround);
        }
        
        RCLCPP_INFO(this->get_logger(), "=====================================");
    }
    
    // Subscriber for task registration
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    
    // Subscriber for task completion notifications
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr completion_subscriber_;
    
    // Publisher for granting time quantum
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grant_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr scheduler_timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;
    
    // Task management
    std::vector<std::string> subscribers_;
    std::unordered_set<std::string> unique_subscriber_ids_;
    
    // Metrics tracking
    std::unordered_map<std::string, TaskMetrics> task_metrics_;
    
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