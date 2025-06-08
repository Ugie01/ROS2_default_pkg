#include "rclcpp/rclcpp.hpp"
// 서비스 인터페이스 헤더 포함 (두 정수를 더하는 서비스)
#include "example_interfaces/srv/add_two_ints.hpp"

// 서비스 서버 클래스 선언 (Node 상속)
class ServieServer : public rclcpp::Node {
public:
    ServieServer() : Node("servie_server") {  // 노드 이름 지정
        // 노드 시작 로그 출력
        RCLCPP_INFO(get_logger(), "ServiceServer Start!");

        // 서비스 생성 부분
        service = create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",  // 서비스 이름
            // 람다 함수로 요청 처리 콜백 정의 (캡처 리스트 [this]로 클래스 멤버 접근)
            [this](
                const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
            ) {
                // 요청 처리: a + b 계산
                response->sum = request->a + request->b;
                
                // 로그 출력 (요청 값)
                RCLCPP_INFO(get_logger(), "Request: %ld + %ld", request->a, request->b);
                // 로그 출력 (응답 값)
                RCLCPP_INFO(get_logger(), "Response: %ld", response->sum);
            });
    }

private:
    // AddTwoInts 서비스 타입의 스마트 포인터 (자동 메모리 관리)
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;
};

// 메인 함수
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // ROS2 초기화
    auto node = std::make_shared<ServieServer>();  // 노드 인스턴스 생성
    rclcpp::spin(node);  // 노드 실행 (이벤트 루프 진입)
    rclcpp::shutdown();  // ROS2 종료 처리
    return 0;
}
