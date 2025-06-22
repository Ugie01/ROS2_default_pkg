#include "rclcpp/rclcpp.hpp"                       // ROS2 C++ 클라이언트 라이브러리 헤더
#include "example_interfaces/srv/add_two_ints.hpp" // 두 정수를 더하는 서비스 타입 헤더
using namespace std::chrono_literals;              // 1s 등 시간 단위 리터럴 사용

// 서비스 클라이언트 클래스 선언 (rclcpp::Node 상속)
class ServiceClient : public rclcpp::Node {
public:
    ServiceClient() : Node("add_two_ints_client") { // 노드 이름 지정
        RCLCPP_INFO(get_logger(), "ServiceClient Start!"); // 노드 시작 로그

        // 서비스 클라이언트 생성 (서비스 이름: add_two_ints)
        client = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        // 서버가 준비될 때까지 1초마다 대기 및 안내 메시지 출력
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for service...");
        }
        RCLCPP_INFO(get_logger(), "Connect Server");

        // ROS2가 정상 동작 중인 동안 반복
        while(rclcpp::ok()){
            RCLCPP_INFO(get_logger(), "두 수를 입력해주세요.");
            
            // 서비스 요청 객체 생성
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            std::cout << "[add_two_ints_client] 입력 값: ";
            std::cin >> request->a >> request->b; // 사용자로부터 두 정수 입력받음

            // 비동기 요청 전송 및 응답 대기
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == 
                rclcpp::FutureReturnCode::SUCCESS) {
                // 응답 성공 시 결과 출력
                RCLCPP_INFO(get_logger(), "Result: %ld", future.get()->sum);
            } else {
                // 응답 실패 시 에러 출력
                RCLCPP_ERROR(get_logger(), "Service call failed");
            }
        }
    }

private:
    // 서비스 클라이언트 스마트 포인터(자동 메모리 관리)
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
};

// 메인 함수
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                      // ROS2 시스템 초기화
    auto node = std::make_shared<ServiceClient>(); // 서비스 클라이언트 노드 생성 및 실행
    rclcpp::shutdown();                            // ROS2 시스템 종료
    return 0;
}
