#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class FCFSDummyTask : public rclcpp::Node
{
public:
    FCFSDummyTask(const std::string & task_id, int execution_time_ms = 0)
    : Node(task_id), 
      task_id_(task_id),
      execution_count_(0),
      is_executing_(false),
      custom_execution_time_(execution_time_ms),
      random_generator_(std::random_device{}())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing FCFS task: %s", task_id_.c_str());
        
        if (custom_execution_time_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Custom execution time: %d ms", custom_execution_time_);
        }
        
        // Publisher for registration
        register_publisher_ = this->create_publisher<std_msgs::msg::String>("register", 10);
        
        // Publisher for task completion notification
        complete_publisher_ = this->create_publisher<std_msgs::msg::String>("task_complete", 10);
        
        // Subscribe to grant topic to receive execution permission
        grant_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "grant", 10,
            std::bind(&FCFSDummyTask::grant_callback, this, std::placeholders::_1));
        
        // Register with scheduler after a brief delay
        register_timer_ = this->create_wall_timer(
            1s, 
            std::bind(&FCFSDummyTask::register_with_scheduler, this)
        );
        
        // Periodic status update (only when not executing)
        status_timer_ = this->create_wall_timer(
            12s, 
            std::bind(&FCFSDummyTask::status_update, this)
        );
    }

private:
    void register_with_scheduler()
    {
        auto registration_msg = std_msgs::msg::String();
        registration_msg.data = task_id_;
        
        RCLCPP_INFO(this->get_logger(), "Registering with FCFS scheduler as '%s'", task_id_.c_str());
        register_publisher_->publish(registration_msg);
        
        // Keep registering periodically until we get scheduled (in case scheduler starts later)
        register_timer_->reset();
    }
    
    void grant_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == task_id_ && !is_executing_) {
            execution_count_++;
            is_executing_ = true;
            
            // Cancel registration timer since we're now active
            register_timer_->cancel();
            
            RCLCPP_INFO(this->get_logger(), 
                       "Granted execution permission! Starting task execution #%d...", 
                       execution_count_);
            
            // Execute the task in a separate thread to avoid blocking ROS callbacks
            std::thread execution_thread(&FCFSDummyTask::execute_task, this);
            execution_thread.detach();
        }
    }
    
    void execute_task()
    {
        auto start_time = std::chrono::steady_clock::now();
        
        // Determine execution time
        int execution_time_ms;
        if (custom_execution_time_ > 0) {
            execution_time_ms = custom_execution_time_;
        } else {
            // Random execution time between 2-8 seconds
            std::uniform_int_distribution<int> duration_dist(2000, 8000);
            execution_time_ms = duration_dist(random_generator_);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Executing task for %d ms...", execution_time_ms);
        
        // Simulate different phases of work
        int phases = 4;
        int phase_duration = execution_time_ms / phases;
        
        for (int phase = 1; phase <= phases; ++phase) {
            // Simulate work phase
            std::this_thread::sleep_for(std::chrono::milliseconds(phase_duration));
            
            // Simulate some CPU work
            simulate_cpu_work();
            
            RCLCPP_INFO(this->get_logger(), 
                       "  Phase %d/%d completed", phase, phases);
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto actual_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Task execution completed! (Actual time: %ld ms)", 
                   actual_duration.count());
        
        // Notify scheduler of completion
        auto complete_msg = std_msgs::msg::String();
        complete_msg.data = task_id_;
        complete_publisher_->publish(complete_msg);
        
        is_executing_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Sent completion notification to scheduler");
    }
    
    void simulate_cpu_work()
    {
        // Simulate CPU-intensive work
        volatile long dummy_computation = 0;
        for (int i = 0; i < 100000; ++i) {
            dummy_computation += i * i + static_cast<long>(std::sqrt(i));
        }
    }
    
    void status_update()
    {
        if (!is_executing_) {
            RCLCPP_INFO(this->get_logger(), 
                       "[%s] Waiting for scheduler | Executions completed: %d", 
                       task_id_.c_str(), execution_count_);
        }
    }
    
    // Node identification
    std::string task_id_;
    
    // Communication
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr register_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr complete_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grant_subscriber_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr register_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Task state
    int execution_count_;
    bool is_executing_;
    int custom_execution_time_;
    std::mt19937 random_generator_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::string task_name = "fcfs_task";
    int execution_time = 0; // 0 means random time
    
    if (argc > 1) {
        task_name = argv[1];
    }
    if (argc > 2) {
        try {
            execution_time = std::stoi(argv[2]);
            if (execution_time < 0) execution_time = 0;
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Invalid execution time, using random");
            execution_time = 0;
        }
    }
    
    // Add some uniqueness if default name
    if (task_name == "fcfs_task") {
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        task_name += "_" + std::to_string(timestamp % 10000);
    }
    
    auto node = std::make_shared<FCFSDummyTask>(task_name, execution_time);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting FCFS dummy task: %s", task_name.c_str());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}