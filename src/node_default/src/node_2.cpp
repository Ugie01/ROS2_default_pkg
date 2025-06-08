#include "rclcpp/rclcpp.hpp"             // ROS2 C++ 클라이언트 라이브러리 헤더
#include "std_msgs/msg/string.hpp"       // 표준 String 메시지 타입 헤더

// Node_2 클래스 선언, rclcpp::Node를 상속받아 ROS2 노드로 동작
class Node_2 : public rclcpp::Node {
public:
    // 생성자: 노드 이름을 "Node_2"로 지정하고, 시작 메시지 출력
    Node_2() : Node("Node_2") {
        RCLCPP_INFO(get_logger(), "Node_2 Start!"); // 로그로 노드 시작 알림
    }

private:

};

// 프로그램 진입점(main 함수)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                        // ROS2 통신 초기화
    auto node = std::make_shared<Node_2>();          // Node_2 객체 생성 (shared_ptr)
    rclcpp::spin(node);                              // 노드 실행(콜백 함수 대기)
    rclcpp::shutdown();                              // ROS2 종료 처리
    return 0;                                        // 프로그램 종료
}