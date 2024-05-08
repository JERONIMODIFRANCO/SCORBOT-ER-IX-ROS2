// Nodo para realizar el loggeo en un archivo de los estados de las juntas y las corrientes
// desde el robot real y del plan ejecutado mediante la función "planear y ejecutar" 
// del plugin de MoveIt2 en RViz

// Todos los datos se almacenarán en archivos dentro de "/home/scorbot/Loggeo/Real"
#include <functional>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
int escuchando = 0;
int primero_jt = 1;
int primero_cor = 1;
double last_time;
int plan_num = 1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("logger")
  {
    subscription_dt = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "display_planned_path", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    subscription_jt = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&MinimalSubscriber::topic_jt, this, std::placeholders::_1));
    subscription_corr = this->create_subscription<std_msgs::msg::String>(
      "corriente_juntas", 10, std::bind(&MinimalSubscriber::topic_corr, this, std::placeholders::_1));
  }

  

 

private:

  // Creo dos subscriptores a los tópicos DisplayTrajectory y a JointState para conocer tanto el plan como el 
  // estado de las juntas durante todo el transcurso del mismo

  void topic_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) const
  {
    
    std::string nombre_archivo = "/home/scorbot/Loggeo/Real/Plan/Plan_" + std::to_string(plan_num) + ".txt";
    
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
    }
    
    // Crea un nuevo nodo subscriptor para saber que se publica en el /joint_states
    RCLCPP_INFO(this->get_logger(), "Plan N° %d almacenado. Empezando el loggeo", plan_num );
    escuchando = 1;
    plan_num +=1;
  }
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_dt;

  void topic_jt(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    static auto start_time_ = rclcpp::Clock().now();
    std::string nombre_archivo = "/home/scorbot/Loggeo/Real/Posiciones/EjecucionR_" + std::to_string(plan_num-1) + ".txt";
    if(escuchando){
      std::ofstream ejecucion(nombre_archivo, std::ios::app); // Crea un objeto ofstream y abre el archivo datos.txt
      if(primero_jt){
        start_time_ = rclcpp::Clock().now(); // Inicia el contador de tiempo
        ejecucion << "Tiempo,J1,J2,J3,J4,J5"<< std::endl;
        primero_jt = 0;
      }

      std::vector<std::string> joint_names = msg->name;
      std::vector<double> joint_positions = msg->position;
      double time_stamp = rclcpp::Clock().now().seconds() - start_time_.seconds();

      if (ejecucion.is_open()) { // Verifica que el archivo se haya abierto correctamente
        const char* separador = ",";
        ejecucion << time_stamp << separador << joint_positions[0] << separador << joint_positions[1] 
        << separador << joint_positions[2] << separador << joint_positions[3] << separador << joint_positions[4] 
        << separador << joint_positions[5] << std::endl; // Escribe el dato en el archivo
        ejecucion.close(); // Cierra el archivo
      } else {
        std::cout << "No se pudo abrir el archivo." << std::endl;
      }

    }
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_jt;

  void topic_corr(const std_msgs::msg::String::SharedPtr msg) const
  {
    static auto start_time_ = rclcpp::Clock().now();
    std::string nombre_archivo = "/home/scorbot/Loggeo/Real/Corrientes/Corrientes_" + std::to_string(plan_num-1) + ".txt";
    if(escuchando){
      std::ofstream corrientes(nombre_archivo, std::ios::app); // Crea un objeto ofstream y abre el archivo datos.txt
      if(primero_cor){
        start_time_ = rclcpp::Clock().now(); // Inicia el contador de tiempo
        corrientes << "Tiempo,C1,C2,C3,C4,C5,C6"<< std::endl;
        primero_cor = 0;
      }

      
      std_msgs::msg::String mensaje;
      mensaje.data = msg->data;
      double time_stamp = rclcpp::Clock().now().seconds() - start_time_.seconds();

      if (corrientes.is_open()) { // Verifica que el archivo se haya abierto correctamente
        const char* separador = ",";
        corrientes << time_stamp << separador << mensaje.data << std::endl; // Escribe el dato en el archivo
        corrientes.close(); // Cierra el archivo
      } else {
        std::cout << "No se pudo abrir el archivo." << std::endl;
      }
      

      if(time_stamp > last_time *1.5){
        primero_jt = 1;
        primero_cor = 1;
        escuchando = 0;
        RCLCPP_INFO(this->get_logger(), "Loggeo N° %d completado", plan_num-1 );
      }
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_corr;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}