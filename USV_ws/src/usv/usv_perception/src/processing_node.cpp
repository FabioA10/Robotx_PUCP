#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <algorithm>

using std::placeholders::_1;

namespace usv_perception
{

struct Deteccion {
  int x, y, w, h, id_clase, estado, ttl;
};

class EvasionProcessor {
public:
  std::vector<Deteccion> historial_detecciones;

  std::pair<int, int> obtener_limites_pista(int y, int h_img, int w_img) {
    int y_horizonte = static_cast<int>(h_img * 0.45);
    int x_top_izq = static_cast<int>(w_img * 0.40);
    int x_top_der = static_cast<int>(w_img * 0.60);
    int x_bot_izq = static_cast<int>(w_img * 0.05);
    int x_bot_der = static_cast<int>(w_img * 0.95);

    if (y < y_horizonte) return {-1, -1};

    double factor = static_cast<double>(y - y_horizonte) / (h_img - y_horizonte);
    int limite_izq = static_cast<int>(x_top_izq + (x_bot_izq - x_top_izq) * factor);
    int limite_der = static_cast<int>(x_top_der + (x_bot_der - x_top_der) * factor);
    return {limite_izq, limite_der};
  }

  std::vector<Deteccion> extraer_candidatos_por_color(const cv::Mat& mask, int id_clase, int y_horizonte) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<Deteccion> candidatos;
    for (const auto& cnt : contours) {
      if (cv::contourArea(cnt) > 15.0) {
        cv::Rect b = cv::boundingRect(cnt);
        int y_base = b.y + b.height;
        if (y_base < y_horizonte) continue;
        candidatos.push_back({b.x, b.y, b.width, b.height, id_clase, 0, y_base});
      }
    }
    return candidatos;
  }

  std::vector<Deteccion> agrupar_bloques_cercanos(const std::vector<Deteccion>& candidatos, double max_dist = 50.0) {
    if (candidatos.empty()) return {};
    std::vector<Deteccion> agrupados;
    std::vector<bool> usados(candidatos.size(), false);
    for (size_t i = 0; i < candidatos.size(); ++i) {
      if (usados[i]) continue;
      int x_min = candidatos[i].x;
      int y_min = candidatos[i].y;
      int x_max = candidatos[i].x + candidatos[i].w;
      int y_max = candidatos[i].y + candidatos[i].h;
      for (size_t j = i + 1; j < candidatos.size(); ++j) {
        if (usados[j] || candidatos[j].id_clase != candidatos[i].id_clase) continue;
        double dist = std::hypot(
          (candidatos[i].x + candidatos[i].w / 2.0) - (candidatos[j].x + candidatos[j].w / 2.0),
          (candidatos[i].y + candidatos[i].h / 2.0) - (candidatos[j].y + candidatos[j].h / 2.0)
        );
        if (dist < max_dist) {
          x_min = std::min(x_min, candidatos[j].x);
          y_min = std::min(y_min, candidatos[j].y);
          x_max = std::max(x_max, candidatos[j].x + candidatos[j].w);
          y_max = std::max(y_max, candidatos[j].y + candidatos[j].h);
          usados[j] = true;
        }
      }
      agrupados.push_back({x_min, y_min, x_max - x_min, y_max - y_min, candidatos[i].id_clase, 0, y_max});
      usados[i] = true;
    }
    return agrupados;
  }

  struct ProcessResult {
    std::vector<Deteccion> detecciones;
    std::string accion;
    float alarma_num;
    cv::Mat hud;
  };

  ProcessResult procesar_frame(const cv::Mat& frame_bgr, bool modo_laboratorio) {
    cv::Mat img;
    cv::resize(frame_bgr, img, cv::Size(640, 360), 0, 0, cv::INTER_LINEAR);
    int h_img = img.rows;
    int w_img = img.cols;
    int y_horizonte = static_cast<int>(h_img * 0.45);

    cv::Mat blurred;
    cv::GaussianBlur(img, blurred, cv::Size(5, 5), 0);
    cv::Mat hsv, gray;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);

    cv::Mat mask_agua;
    cv::inRange(hsv, cv::Scalar(90, 80, 30), cv::Scalar(135, 255, 255), mask_agua);
    cv::Mat mask_sin_agua;
    cv::bitwise_not(mask_agua, mask_sin_agua);

    std::vector<cv::Mat> hsv_planes, bgr_planes;
    cv::split(hsv, hsv_planes);
    cv::split(blurred, bgr_planes);
    cv::Mat v = hsv_planes[2];
    cv::Mat b = bgr_planes[0], g = bgr_planes[1], r = bgr_planes[2];

    cv::Mat mask_v;
    cv::threshold(v, mask_v, 40, 255, cv::THRESH_BINARY);
    int umbral_croma = modo_laboratorio ? 60 : 20;

    cv::Mat max_gb;
    cv::max(g, b, max_gb);
    cv::Mat dom_rojo;
    cv::subtract(r, max_gb, dom_rojo);
    cv::Mat mask_r_chroma;
    cv::threshold(dom_rojo, mask_r_chroma, umbral_croma, 255, cv::THRESH_BINARY);
    cv::Mat mask_red;
    cv::bitwise_and(mask_r_chroma, mask_v, mask_red);

    cv::Mat max_rb;
    cv::max(r, b, max_rb);
    cv::Mat dom_verde;
    cv::subtract(g, max_rb, dom_verde);
    cv::Mat mask_g_chroma;
    cv::threshold(dom_verde, mask_g_chroma, umbral_croma, 255, cv::THRESH_BINARY);
    cv::Mat mask_green;
    cv::bitwise_and(mask_g_chroma, mask_v, mask_green);

    if (!modo_laboratorio) {
      cv::bitwise_and(mask_red, mask_sin_agua, mask_red);
      cv::bitwise_and(mask_green, mask_sin_agua, mask_green);
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask_red, mask_red, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask_green, mask_green, cv::MORPH_CLOSE, kernel);

    auto cand_r = extraer_candidatos_por_color(mask_red, 1, y_horizonte);
    auto cand_g = extraer_candidatos_por_color(mask_green, 2, y_horizonte);
    std::vector<Deteccion> cand_all = cand_r;
    cand_all.insert(cand_all.end(), cand_g.begin(), cand_g.end());
    auto candidatos_fusionados = agrupar_bloques_cercanos(cand_all, 55.0);

    cv::Mat edges_raw;
    cv::Canny(gray, edges_raw, 20, 75);
    std::vector<Deteccion> objetos_actuales;

    for (const auto& c : candidatos_fusionados) {
      int x_b = c.x, y_b = c.y, w_b = c.w, h_b = c.h, id_clase = c.id_clase;
      int y_base = y_b + h_b;
      double aspect_ratio = static_cast<double>(h_b) / w_b;
      if (aspect_ratio > 2.2) {
        h_b = static_cast<int>(w_b * 1.6);
        y_base = y_b + h_b;
      }
      
      if (w_b < 5 || h_b < 5) continue;
      
      cv::Rect roi(x_b, y_b, w_b, h_b);
      roi = roi & cv::Rect(0, 0, w_img, h_img);
      if (roi.area() == 0) continue;
      
      cv::Mat roi_edges = edges_raw(roi);
      int min_edges = (id_clase == 2) ? 6 : 12;
      if (cv::countNonZero(roi_edges) < min_edges) continue;

      auto limites = obtener_limites_pista(y_base, h_img, w_img);
      int estado_objeto = 0;
      if (limites.first != -1 && limites.second != -1) {
        int centro_x = x_b + w_b / 2;
        if (centro_x >= limites.first && centro_x <= limites.second) {
          double factor_proximidad = static_cast<double>(y_base - y_horizonte) / (h_img - y_horizonte);
          estado_objeto = (factor_proximidad >= 0.45) ? 2 : 1;
        }
      }
      objetos_actuales.push_back({x_b, y_b, w_b, h_b, id_clase, estado_objeto, 8});
    }

    for (auto& act : objetos_actuales) {
      bool encontrado = false;
      for (auto& prev : historial_detecciones) {
        double dist = std::hypot((act.x + act.w/2.0) - (prev.x + prev.w/2.0),
                                 (act.y + act.h/2.0) - (prev.y + prev.h/2.0));
        double max_match_dist = (prev.w > 45) ? 55.0 : 35.0;
        if (dist < max_match_dist) {
          prev.x = act.x; prev.y = act.y; prev.w = act.w; prev.h = act.h;
          prev.estado = act.estado; prev.id_clase = act.id_clase; prev.ttl = act.ttl;
          encontrado = true;
          break;
        }
      }
      if (!encontrado) historial_detecciones.push_back(act);
    }

    std::vector<Deteccion> objetos_finales;
    bool critico = false, lejano = false;
    double lado_obstaculo = 0.0;

    for (auto it = historial_detecciones.begin(); it != historial_detecciones.end();) {
      it->ttl -= 1;
      if (it->ttl <= 0) {
        it = historial_detecciones.erase(it);
      } else {
        objetos_finales.push_back(*it);
        if (it->estado == 2) {
          critico = true;
          auto lims = obtener_limites_pista(it->y + it->h, h_img, w_img);
          if (lims.first != -1) {
            lado_obstaculo = (it->x + it->w/2.0) - (lims.first + (lims.second - lims.first)/2.0);
          }
        } else if (it->estado == 1) {
          lejano = true;
        }
        ++it;
      }
    }

    ProcessResult res;
    res.hud = img.clone();
    res.detecciones = objetos_finales;

    if (critico) {
      res.accion = (lado_obstaculo < 0) ? "EVASION: PIVOTE DERECHO" : "EVASION: PIVOTE IZQUIERDO";
      res.alarma_num = (lado_obstaculo < 0) ? 1.0f : 2.0f;
    } else if (lejano) {
      res.accion = "ALERTA: FRENADO/RETROCESO TACTICO";
      res.alarma_num = 3.0f;
    } else {
      res.accion = "CANAL LIBRE: AVANCE GPS";
      res.alarma_num = 0.0f;
    }

    return res;
  }
};

class ProcessingNode : public rclcpp::Node
{
public:
  explicit ProcessingNode(const rclcpp::NodeOptions & options)
  : Node("processing_node", options)
  {
    this->declare_parameter("modo_laboratorio", true);
    this->declare_parameter("ip_base", "192.168.2.1");
    this->declare_parameter("puerto_udp", 5000);

    bool modo_lab = this->get_parameter("modo_laboratorio").as_bool();
    std::string ip_base = this->get_parameter("ip_base").as_string();
    int puerto = this->get_parameter("puerto_udp").as_int();

    pub_alarma_ = this->create_publisher<std_msgs::msg::Float32>("/robotx/alarma_frontal", 10);
    
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/robotx/camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        this->image_callback(std::move(msg));
      });

    std::string gst_cmd = "gst-launch-1.0 -q fdsrc ! rawvideoparse use-sink-caps=false format=bgr width=640 height=360 framerate=30/1 ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast intra-refresh=true ! rtph264pay config-interval=1 pt=96 ! udpsink host=" + ip_base + " port=" + std::to_string(puerto) + " sync=false";
    
    out_video_ = popen(gst_cmd.c_str(), "w");
    if (!out_video_) {
      RCLCPP_ERROR(this->get_logger(), "[-] Error starting GStreamer CLI");
    } else {
      RCLCPP_INFO(this->get_logger(), "[+] Transmisión UDP iniciada hacia %s:%d", ip_base.c_str(), puerto);
    }

    RCLCPP_INFO(this->get_logger(), "[+] Nodo USV Perception Activo. Entorno: %s", modo_lab ? "LABORATORIO SECO" : "ACUATICO/MAR");
  }

  ~ProcessingNode() override
  {
    if (out_video_) {
      pclose(out_video_);
    }
  }

private:
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    bool modo_lab = this->get_parameter("modo_laboratorio").as_bool();
    auto res = processor_.procesar_frame(cv_ptr->image, modo_lab);

    int h_img = res.hud.rows;
    int w_img = res.hud.cols;
    int y_horizonte = static_cast<int>(h_img * 0.45);
    int y_mitad_pista = y_horizonte + static_cast<int>((h_img - y_horizonte) * 0.50);

    auto lims = processor_.obtener_limites_pista(y_mitad_pista, h_img, w_img);
    if (lims.first != -1 && lims.second != -1) {
      std::vector<cv::Point> pts_alerta = {
        cv::Point(w_img * 0.40, y_horizonte), cv::Point(w_img * 0.60, y_horizonte),
        cv::Point(lims.second, y_mitad_pista), cv::Point(lims.first, y_mitad_pista)
      };
      std::vector<cv::Point> pts_criticos = {
        cv::Point(lims.first, y_mitad_pista), cv::Point(lims.second, y_mitad_pista),
        cv::Point(w_img * 0.95, h_img), cv::Point(w_img * 0.05, h_img)
      };

      cv::Mat overlay = res.hud.clone();
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{pts_alerta}, cv::Scalar(0, 255, 255));
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{pts_criticos}, cv::Scalar(0, 0, 255));
      cv::addWeighted(overlay, 0.15, res.hud, 0.85, 0, res.hud);
    }

    for (const auto& det : res.detecciones) {
      cv::Scalar color_hud = (det.estado == 2) ? cv::Scalar(0, 0, 255) :
                             (det.estado == 1) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);
      std::string label = (det.id_clase == 1) ? "BOYA ROJA" : "BOYA VERDE";
      cv::Scalar color_tag = (det.id_clase == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);

      cv::rectangle(res.hud, cv::Rect(det.x, det.y, det.w, det.h), color_hud, 2);
      cv::putText(res.hud, label, cv::Point(det.x, det.y - 7), cv::FONT_HERSHEY_SIMPLEX, 0.45, color_tag, 2);
    }

    cv::rectangle(res.hud, cv::Rect(0, 0, w_img, 45), cv::Scalar(15, 15, 15), -1);
    cv::putText(res.hud, "ESTADO USV: " + res.accion, cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 2);

    std_msgs::msg::Float32 msg_alarma;
    msg_alarma.data = res.alarma_num;
    pub_alarma_->publish(msg_alarma);

    if (out_video_) {
      fwrite(res.hud.data, 1, res.hud.total() * res.hud.elemSize(), out_video_);
      fflush(out_video_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_alarma_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  EvasionProcessor processor_;
  FILE* out_video_ = nullptr;
};

}  // namespace usv_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usv_perception::ProcessingNode)
