#include "rclcpp/rclcpp.hpp"             // ROS2 C++ 클라이언트 라이브러리 헤더
#include "std_msgs/msg/string.hpp"       // 표준 String 메시지 타입 헤더
using namespace std::chrono_literals;    // 1s, 500ms 등 시간 단위 사용을 위해 필요

// Pub 클래스 선언, rclcpp::Node를 상속받아 ROS2 노드로 동작
class Pub : public rclcpp::Node {
public:
    // 생성자: 노드 이름을 "Pub"로 지정
    Pub() : Node("Pub") {
        RCLCPP_INFO(get_logger(), "Publisher Start!"); // 시작 로그

        // 발행자 생성 토픽, 큐 크기
        publisher = create_publisher<std_msgs::msg::String>("/topic", 10);

        // 1초 주기 타이머 설정
        timer = create_wall_timer(1s,
            [this]() {  // 람다 함수로 콜백 처리
                auto msg = std_msgs::msg::String();
                msg.data = "Ugie01 Blog " + std::to_string(count++);
                publisher->publish(msg);
                RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
            }
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    int count = 0;

};

// 프로그램 진입점(main 함수)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                        // ROS2 통신 초기화
    auto node = std::make_shared<Pub>();             // Pub 객체 생성 (shared_ptr)
    rclcpp::spin(node);                              // 노드 실행(콜백 함수 대기)
    rclcpp::shutdown();                              // ROS2 종료 처리
    return 0;                                        // 프로그램 종료
}