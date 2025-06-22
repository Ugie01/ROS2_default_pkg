// ROS2 및 액션 관련 헤더 포함
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/fibonacci.hpp"

using Fibonacci = custom_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

// 액션 클라이언트 노드 클래스
class FibonacciClient : public rclcpp::Node {
public:
    FibonacciClient() : Node("fibonacci_client") {
        // 액션 클라이언트 생성 (서비스명: "fibonacci")
        client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    }

    // 액션 목표 전송 메서드
    void send_goal(int order) {
        // 액션 서버가 준비될 때까지 대기 (최대 1초)
        if(!client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        // 액션 목표 객체 생성 및 값 설정
        auto goal = Fibonacci::Goal();
        goal.order = order;  // 피보나치 수열 길이 설정

        // 콜백 옵션 설정
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        
        // 피드백 콜백 (진행 상황 수신 시 실행)
        send_goal_options.feedback_callback = 
            [this](GoalHandle::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
                RCLCPP_INFO(get_logger(), "Next number: %d", feedback->partial.back());
            };
        
        // 결과 콜백 (작업 완료 시 실행)
        send_goal_options.result_callback = 
            [this](const GoalHandle::WrappedResult & result) {
                if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Final sequence:");
                    for(auto num : result.result->sequence) {
                        RCLCPP_INFO(get_logger(), "%d", num);  // 최종 결과 출력
                    }
                }
            };

        // 비동기 방식으로 목표 전송 (논블로킹)
        client->async_send_goal(goal, send_goal_options);
    }

private:
    // 액션 클라이언트 스마트 포인터
    rclcpp_action::Client<Fibonacci>::SharedPtr client;
};

// 메인 함수
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // ROS2 초기화
    
    auto client_node = std::make_shared<FibonacciClient>();  // 노드 인스턴스 생성
    client_node->send_goal(10);  // 10번째 피보나치 수열 요청
    
    rclcpp::spin(client_node);  // 이벤트 루프 실행 (콜백 처리)
    rclcpp::shutdown();  // ROS2 종료 처리
    return 0;
}
