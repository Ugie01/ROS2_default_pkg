// ROS2 및 액션 관련 헤더 포함
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/fibonacci.hpp"

// 타입 앨리어스로 복잡한 타입 이름 간소화
using Fibonacci = custom_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

// 액션 서버 노드 클래스
class FibonacciServer : public rclcpp::Node {
public:
    FibonacciServer() : Node("fibonacci_server") {
        // 액션 서버 생성 및 콜백 함수 연결
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",  // 액션 이름
            std::bind(&FibonacciServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),   // Goal UUID와 Goal 
            std::bind(&FibonacciServer::handle_cancel, this, std::placeholders::_1),                        // GoalHandle
            std::bind(&FibonacciServer::handle_accepted, this, std::placeholders::_1)                       // GoalHandle
        );
    }

private:
    //// [콜백 함수 3종] ////
    
    // 1. 목표 수신 처리
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Fibonacci::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // 목표 수락
    }

    // 2. 취소 요청 처리
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;  // 취소 수락
    }

    // 3. 목표 실행 시작 (멀티스레드 처리)
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{std::bind(&FibonacciServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // 실제 피보나치 수열 생성 로직
    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        // 피보나치 초기값 설정
        int a = 0, b = 1;
        feedback->partial.push_back(a);
        feedback->partial.push_back(b);

        // 수열 생성 루프
        for(int i=1; i <= goal->order; ++i) {
            // 취소 요청 확인
            if(goal_handle->is_canceling()) {
                result->sequence = feedback->partial;
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }

            // 피보나치 수열 계산 및 피드백 발행
            feedback->partial.push_back(feedback->partial[i] + feedback->partial[i-1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(get_logger(), "Publish feedback");

            // 0.5초 대기 (시뮬레이션을 위한 딜레이)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // 최종 결과 반환
        result->sequence = feedback->partial;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    }

    // 액션 서버 인스턴스
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // ROS2 초기화
    auto node = std::make_shared<FibonacciServer>();  // 노드 생성
    rclcpp::spin(node);  // 이벤트 루프 실행
    rclcpp::shutdown();  // ROS2 종료
    return 0;
}
