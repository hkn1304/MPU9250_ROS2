#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "mpu9250_publisher/mpu9250.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MPU9250Publisher : public rclcpp::Node
{
public:
  MPU9250Publisher() : Node("mpu9250_publisher")//, mpu9250_(1, 0x68) commented out due to declare parameter approach
  {

    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<int>("i2c_address", 0x68);

    int i2c_bus = this->get_parameter("i2c_bus").as_int();
    int i2c_address = this->get_parameter("i2c_address").as_int();

    mpu9250_ = std::make_shared<MPU9250>(i2c_bus,i2c_address); 
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/sixdof", 10);
    imu_timer_ = this->create_wall_timer( 10ms, std::bind(&MPU9250Publisher::publish_imu_data, this));
    
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag",10);
    mag_timer_ = this->create_wall_timer(10ms,std::bind(&MPU9250Publisher::publish_mag_data,this));

    mpu9250_->readAccelData(/*accelX, accelY, accelZ*/);
    (*mpu9250_).readGyroData(/*gyroX, gyroY, gyroZ*/);
    mpu9250_->readMagnetometerData(/*magX, magY, magZ*/);
  }

private:
  void publish_imu_data()
  {
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->get_clock()->now();
    message.linear_acceleration.x =  mpu9250_->getAccX();
    message.linear_acceleration.y =  (*mpu9250_).getAccY();
    message.linear_acceleration.z =  mpu9250_->getAccZ();

    message.angular_velocity.x = mpu9250_->getGyroX();
    message.angular_velocity.y = (*mpu9250_).getGyroY();
    message.angular_velocity.z = mpu9250_->getGyroZ();

    imu_publisher_->publish(message);
  }

  void publish_mag_data()
  {
    auto magneto = sensor_msgs::msg::MagneticField();
    magneto.header.stamp = this->get_clock()->now();
    magneto.magnetic_field.x = mpu9250_->getMagX();
    magneto.magnetic_field.y = (*mpu9250_).getMagY();
    magneto.magnetic_field.z = mpu9250_->getMagZ();    
    
    mag_publisher_->publish(magneto);
  }

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr mag_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  size_t count_;
  //MPU9250 mpu9250_; commented out due to paramater approach, emplaced with sharedPtr
  std::shared_ptr<MPU9250> mpu9250_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU9250Publisher>());
  rclcpp::shutdown();
  return 0;
}
