#include "rclcpp/rclcpp.hpp"             // ROS2 C++ 클라이언트 라이브러리 헤더
#include "std_msgs/msg/string.hpp"       // 표준 String 메시지 타입 헤더

// Sub 클래스 선언, rclcpp::Node를 상속받아 ROS2 노드로 동작
class Sub : public rclcpp::Node {
public:
    // 생성자: 노드 이름을 "Sub"로 지정하고, 시작 메시지 출력
    Sub() : Node("Sub") {
        RCLCPP_INFO(get_logger(), "Subscriber Start!"); // 시작 로그

        // 구독자 생성 토픽, 큐 크기, 콜백 함수
        subscription = create_subscription<std_msgs::msg::String>(
            "/topic",
            10,
            // 람다 함수로 콜백 처리
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "Received: '%s'", msg->data.c_str());
            }
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

// 프로그램 진입점(main 함수)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                        // ROS2 통신 초기화
    auto node = std::make_shared<Sub>();             // Sub 객체 생성 (shared_ptr)
    rclcpp::spin(node);                              // 노드 실행(콜백 함수 대기)
    rclcpp::shutdown();                              // ROS2 종료 처리
    return 0;                                        // 프로그램 종료
}