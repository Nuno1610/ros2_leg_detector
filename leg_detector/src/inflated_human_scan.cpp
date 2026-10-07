#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <leg_detector_msgs/msg/person_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <eigen3/Eigen/Dense>
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/subscription_options.hpp"
#include <limits> // Para utilizar el infinito.

class InflatedHumanScanNode : public rclcpp::Node
{
public:
    InflatedHumanScanNode()
        : Node("inflated_human_scan_node")
    {
        // Recupera y asigna los parámetros, gestionando los posibles errores.
        this->declare_parameter("inflation_radius", rclcpp::ParameterValue(1.0));
        if (!this->get_parameter("inflation_radius", inflation_r))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve 'inflation_radius' parameter. Setting to default 1.0.");
            inflation_r = 1.0;
        }
        if (inflation_r <= 0.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid 'inflation_radius' parameter. Must be positive. Setting to default 1.0.");
            inflation_r = 1.0;
        }

        RCLCPP_INFO(this->get_logger(), "Inflation radius: %f", inflation_r);

        // Inicializa los suscriptores, gestionando los posibles errores.
        try
        {
            scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());
            people_tracked_sub_ = std::make_shared<message_filters::Subscriber<leg_detector_msgs::msg::PersonArray>>(this, "people_tracked", rclcpp::SensorDataQoS().get_rmw_qos_profile());

            sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, leg_detector_msgs::msg::PersonArray>>(*scan_sub_, *people_tracked_sub_, 200);

            // Registra un callback sincronizado.
            sync_->registerCallback(std::bind(&InflatedHumanScanNode::inflated_human_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error initializing subscribers or synchronizer: %s", e.what());
            rclcpp::shutdown();
        }

        // Publica en el tópico inflated_human_scan.
        ihs_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("inflated_human_scan", 20);
    }

private:
    // Suscriptores con filtros de mensajes.
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_sub_;
    std::shared_ptr<message_filters::Subscriber<leg_detector_msgs::msg::PersonArray>> people_tracked_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, leg_detector_msgs::msg::PersonArray>> sync_;

    // Publicador.
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr ihs_pub_;

    // Parámetros y otras variables.
    float angle_min;
    float angle_max;
    float angle_inc;
    double inflation_r;
    sensor_msgs::msg::LaserScan updated_human_scan_;
    std::string scan_topic_;

    // Función callback.
    void inflated_human_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan, const leg_detector_msgs::msg::PersonArray::ConstSharedPtr &people_tracked)
    {
        // Obtiene los parámetros del escaneo.
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;

        // Inicializa el escaneo humano a partir del escaneo láser recibido.
        updated_human_scan_ = *scan;

        for (const auto &person : people_tracked->people)
        {
            float xH = person.pose.position.x;
            float yH = person.pose.position.y;
            float dH = sqrt(xH * xH + yH * yH);

            // Llama a la función que genera un escaneo con el radio inflado.
            if (dH > inflation_r)
            {
                try
                {
                    inflate_human_position(xH, yH);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error inflating human position: %s", e.what());
                }
            }
        }

        // Publica el escaneo láser actualizado.
        ihs_pub_->publish(updated_human_scan_);
    }

    // Genera varios puntos alrededor de una persona seguida, incorporando el radio de inflación.
    void inflate_human_position(float xH, float yH)
    {
        Eigen::VectorXf temp_ranges;
        Eigen::VectorXf temp_angles;
        Eigen::VectorXi temp_ind;

        // Derivaciones necesarias para calcular los valores de ranges[].
        float dH = sqrt(xH * xH + yH * yH);
        if (dH <= inflation_r)
        {
            RCLCPP_WARN(this->get_logger(), "Distance to human is less than or equal to inflation radius. Skipping inflation.");
            return;
        }
        float angle = atan2(yH, xH);
        float theta_tangent = asin(inflation_r / dH);

        if (angle_inc <= 0.0f)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid angle increment (<= 0). Cannot proceed with inflation.");
            return;
        }

        // Calcula la distancia al círculo para distintos ángulos.
        Eigen::ArrayXf delta_theta = Eigen::VectorXf::LinSpaced(static_cast<int>(floor(2 * theta_tangent / angle_inc)), -theta_tangent, theta_tangent);
        temp_ranges = dH * delta_theta.cos() - (inflation_r * inflation_r - dH * dH * (delta_theta.sin()).square()).sqrt();
        temp_angles = angle + delta_theta;
        temp_ind = ((temp_angles.array() - angle_min) / angle_inc).cast<int>();

        // Actualiza el escaneo con los nuevos valores de ranges[].
        for (int i = 0; i < temp_ind.size(); i++)
        {
            // Comprueba que temp_ind está dentro del rango válido.
            if (temp_ind[i] < 0 || temp_ind[i] >= updated_human_scan_.ranges.size())
            {
                RCLCPP_WARN(this->get_logger(), "Index out of range during inflation. Skipping.");
                continue;
            }

            // Garantiza que los rangos sean válidos.
            if (temp_ranges[i] < updated_human_scan_.range_min)
                temp_ranges[i] = updated_human_scan_.range_min;
            else if (temp_ranges[i] > updated_human_scan_.range_max)
                temp_ranges[i] = updated_human_scan_.range_max;

            if (updated_human_scan_.ranges[temp_ind(i)] > temp_ranges(i))
                updated_human_scan_.ranges[temp_ind(i)] = temp_ranges(i);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InflatedHumanScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
