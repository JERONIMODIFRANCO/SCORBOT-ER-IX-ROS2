// Nodo para realizar el loggeo en un archivo de los estados de las juntas (desde Gazebo) 
// y del plan ejecutado mediante la función "planear y ejecutar" del plugin de MoveIt2 en RViz

#include <functional>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using std::placeholders::_1;
int juntas = 0;
int tiempo = 0;
double last_time;
int plan_num = 1;
int primero = 0;
int ultimo = 0;
double actual_time_;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("logger")
  {
    subscription_dt = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "display_planned_path", 10, std::bind(&MinimalSubscriber::trayectory_callback, this, std::placeholders::_1));
    subscription_jt = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&MinimalSubscriber::jt_callback, this, std::placeholders::_1));
    subscription_clock = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "clock_gz", 10, std::bind(&MinimalSubscriber::clock_callback, this, std::placeholders::_1));
  }


private:

  // Creo dos subscriptores a los tópicos DisplayTrajectory y a JointState para conocer tanto el plan como el 
  // estado de las juntas durante todo el transcurso del mismo

  void trayectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) const
  {
    
    std::string nombre_archivo = "/home/scorbot/Loggeo/Gazebo/Plan/Plan_" + std::to_string(plan_num) + ".txt";
    
    if (!msg->trajectory.empty()) {
      // Accede al primer punto de la trayectoria planificada
      // auto& first_trajectory_point = msg->trajectory[0].joint_trajectory.points[0];

      // Accede a los valores de las juntas en el primer punto
      // auto& joint_values = first_trajectory_point.positions;

      // Imprime los valores de las juntas
      // for (size_t i = 0; i < joint_values.size(); ++i) {
      //     RCLCPP_INFO(this->get_logger(), "Joint %zu value: %f", i, joint_values[i]);
      // }
            

      // std::ofstream plan(nombre_archivo, std::ios::app); // Crea un objeto ofstream y abre el archivo sin borrar lo anterior
      std::ofstream plan(nombre_archivo); // Crea un objeto ofstream y abre el archivo borrando lo anterior
      const char* separador = ","; 
      if (plan.is_open()) { // Verifica que el archivo se haya abierto correctamente
        plan << "Tiempo,J1,J2,J3,J4,J5"<< std::endl;
        for (const auto& trajectory : msg->trajectory) {
          for (const auto& point : trajectory.joint_trajectory.points) {
            double time_stamp = point.time_from_start.sec + static_cast<double>(point.time_from_start.nanosec) * 1e-09;
          
            // Imprime el tiempo del punto actual
            // RCLCPP_INFO(this->get_logger(), "Point timestamp: %.3fs", time_stamp);
            // Imprime los valores de las juntas en el punto actual
            // RCLCPP_INFO(this->get_logger(), "Tiempo: %.3fs\n J1: %f, J2: %f, J3: %f, J4: %f, J5: %f", time_stamp, 
            // point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4]);
            plan << time_stamp << separador << point.positions[0] << separador << point.positions[1] << separador << point.positions[2] << separador << point.positions[3] << separador << point.positions[4] << std::endl; // Escribe el dato en el archivo
          }
        }
        plan.close(); // Cierra el archivo
        // std::cout << "Dato guardado en el archivo." << std::endl;
      } else {
        std::cout << "No se pudo abrir el archivo." << std::endl;
      }
    
      auto& last_trajectory = msg->trajectory.back();
      if (!last_trajectory.joint_trajectory.points.empty()) {
        auto& last_point = last_trajectory.joint_trajectory.points.back();
        last_time = static_cast<double>(last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-09);
        // Ahora last_time contiene el tiempo del último punto de la última trayectoria
      }

      // Crea un nuevo nodo subscriptor para saber que se publica en el /joint_states
      RCLCPP_INFO(this->get_logger(), "Plan N° %d almacenado. Empezando el loggeo", plan_num );
      tiempo = 1;
      primero = 1;
    }
    

  }
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_dt;

  void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr clock_gz) const
  {
    auto variable_tiempo = clock_gz->clock;
    static auto start_time_ = variable_tiempo.sec + (variable_tiempo.nanosec / 1e9);
    if(tiempo){
      tiempo = 0;
      if(primero){
        start_time_ = variable_tiempo.sec + (variable_tiempo.nanosec / 1e9); // Inicia el contador de tiempo
      }
      actual_time_ = variable_tiempo.sec + (variable_tiempo.nanosec / 1e9) - start_time_; 
      juntas = 1;

      if(actual_time_ > last_time *1.2){
        ultimo = 1;
      }
    }
  }
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_clock;

  void jt_callback(const sensor_msgs::msg::JointState::SharedPtr joints) const
  {
    std::string nombre_archivo = "/home/scorbot/Loggeo/Gazebo/Posiciones/EjecucionS_" + std::to_string(plan_num) + ".txt";
    if(juntas){
      juntas = 0;
      std::ofstream ejecucion(nombre_archivo, std::ios::app); // Crea un objeto ofstream y abre el archivo datos.txt
      if(primero){
        ejecucion << "Tiempo,J1,J2,J3,J4,J5"<< std::endl;
        primero = 0;
      }

      std::vector<std::string> joint_names = joints->name;
      std::vector<double> joint_positions = joints->position;

      if (ejecucion.is_open()) { // Verifica que el archivo se haya abierto correctamente
        const char* separador = ",";
        ejecucion << actual_time_ << separador << joint_positions[0] << separador << joint_positions[1] 
        << separador << joint_positions[2] << separador << joint_positions[3] << separador << joint_positions[4] 
        << separador << joint_positions[5] << std::endl; // Escribe el dato en el archivo
        ejecucion.close(); // Cierra el archivo
      } else {
        std::cout << "No se pudo abrir el archivo." << std::endl;
      }

      tiempo = 1;
      

      if(ultimo){
        ultimo = 0;
        primero = 1;
        tiempo = 0;
        RCLCPP_INFO(this->get_logger(), "Loggeo N° %d completado", plan_num );
        plan_num +=1;
      }
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_jt;
  
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
