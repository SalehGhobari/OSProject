#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PriorityDummyTask : public rclcpp::Node
{
public:
    PriorityDummyTask(const std::string & task_id, int priority, int execution_time_ms = 0)
    : Node(task_id), 
      task_id_(task_id),
      priority_(priority),
      execution_count_(0),
      is_executing_(false),
      custom_execution_time_(execution_time_ms),
      random_generator_(std::random_device{}())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Priority task: %s with priority: %d", 
                   task_id_.c_str(), priority_);
        
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
            std::bind(&PriorityDummyTask::grant_callback, this, std::placeholders::_1));
        
        // Register with scheduler after a brief delay
        register_timer_ = this->create_wall_timer(
            1s, 
            std::bind(&PriorityDummyTask::register_with_scheduler, this)
        );
        
        // Periodic status update (only when not executing)
        status_timer_ = this->create_wall_timer(
            10s, 
            std::bind(&PriorityDummyTask::status_update, this)
        );
    }

private:
    void register_with_scheduler()
    {
        auto registration_msg = std_msgs::msg::String();
        // Format: "task_id:priority"
        registration_msg.data = task_id_ + ":" + std::to_string(priority_);
        
        RCLCPP_INFO(this->get_logger(), "📝 Registering with Priority scheduler as '%s' (Priority: %d)", 
                   task_id_.c_str(), priority_);
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
                       "🚀 [Priority: %d] Granted execution permission! Starting task execution #%d...", 
                       priority_, execution_count_);
            
            // Execute the task in a separate thread to avoid blocking ROS callbacks
            std::thread execution_thread(&PriorityDummyTask::execute_task, this);
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
            // Random execution time between 3-7 seconds for priority tasks
            std::uniform_int_distribution<int> duration_dist(3000, 7000);
            execution_time_ms = duration_dist(random_generator_);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "⚙️  [Priority: %d] Executing task for %d ms...", 
                   priority_, execution_time_ms);
        
        // Simulate different phases of work with priority-aware messaging
        int phases = 5;
        int phase_duration = execution_time_ms / phases;
        
        for (int phase = 1; phase <= phases; ++phase) {
            // Simulate work phase
            std::this_thread::sleep_for(std::chrono::milliseconds(phase_duration));
            
            // Simulate some CPU work (more work for higher priority tasks)
            simulate_cpu_work();
            
            RCLCPP_INFO(this->get_logger(), 
                       "  📈 [P%d] Phase %d/%d completed", 
                       priority_, phase, phases);
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto actual_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(this->get_logger(), 
                   "✅ [Priority: %d] Task execution completed! (Actual time: %ld ms)", 
                   priority_, actual_duration.count());
        
        // Notify scheduler of completion
        auto complete_msg = std_msgs::msg::String();
        complete_msg.data = task_id_;
        complete_publisher_->publish(complete_msg);
        
        is_executing_ = false;
        
        RCLCPP_INFO(this->get_logger(), "📤 [Priority: %d] Sent completion notification to scheduler", priority_);
        
        // Re-register for next execution cycle after a delay
        auto reregister_delay = std::chrono::milliseconds(1000 + (rand() % 2000)); // 1-3 seconds
        reregister_timer_ = this->create_wall_timer(
            reregister_delay,
            [this]() {
                RCLCPP_INFO(this->get_logger(), "🔄 Re-registering for next execution cycle...");
                register_timer_ = this->create_wall_timer(1s, std::bind(&PriorityDummyTask::register_with_scheduler, this));
                reregister_timer_->cancel();
            }
        );
    }
    
    void simulate_cpu_work()
    {
        // Simulate CPU-intensive work - higher priority tasks do more computation
        volatile long dummy_computation = 0;
        int work_amount = 50000 + (priority_ * 25000); // More work for higher priority
        
        for (int i = 0; i < work_amount; ++i) {
            dummy_computation += i * i + std::sqrt(i);
        }
    }
    
    void status_update()
    {
        if (!is_executing_) {
            RCLCPP_INFO(this->get_logger(), 
                       "💤 [%s | P%d] Waiting for scheduler | Executions completed: %d", 
                       task_id_.c_str(), priority_, execution_count_);
        }
    }
    
    // Node identification
    std::string task_id_;
    int priority_;
    
    // Communication
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr register_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr complete_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grant_subscriber_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr register_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr reregister_timer_;
    
    // Task state
    int execution_count_;
    bool is_executing_;
    int custom_execution_time_;
    std::mt19937 random_generator_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::string task_name = "priority_task";
    int priority = 1; // Default priority
    int execution_time = 0; // 0 means random time
    
    if (argc > 1) {
        task_name = argv[1];
    }
    if (argc > 2) {
        try {
            priority = std::stoi(argv[2]);
            if (priority < 0) priority = 1;
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Invalid priority, using default priority 1");
            priority = 1;
        }
    }
    if (argc > 3) {
        try {
            execution_time = std::stoi(argv[3]);
            if (execution_time < 0) execution_time = 0;
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Invalid execution time, using random");
            execution_time = 0;
        }
    }
    
    // Add some uniqueness if default name
    if (task_name == "priority_task") {
        auto now = std::chrono::steady_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        task_name += "_p" + std::to_string(priority) + "_" + std::to_string(timestamp % 10000);
    }
    
    auto node = std::make_shared<PriorityDummyTask>(task_name, priority, execution_time);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Priority dummy task: %s (Priority: %d)", 
               task_name.c_str(), priority);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}