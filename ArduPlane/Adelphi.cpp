#include "Adelphi.h"
#include "Plane.h"

// Construtor
Adelphi::Adelphi()
{
  release_esp32_data.id = 0x69;
}

// Destrutor
Adelphi::~Adelphi()
{
}

void Adelphi::trigger_emergency()
{
  this->emergency_active = true;
  GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "[Adelphi] Emergencia via MAVLink acionada!");
}

void Adelphi::on_emergency_actived()
{
  if (!this->emergency_active)
  {
    return;
  }

  if (this->emergency_msg_count % 30 == 0)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "[Adelphi] Condicao de emergencia ativa!");
  }

  if (plane.get_mode() != plane.mode_manual.mode_number())
  {
    plane.set_mode(plane.mode_manual, ModeReason::SCRIPTING);
  }

  plane.channel_rudder->set_override(plane.channel_rudder->get_radio_max(), AP_HAL::millis());
  this->emergency_msg_count++;
}

// Inicializa a comunicação com o ESP32
void Adelphi::init()
{
  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Inicializando e buscando modulo de alijamento UART...");

  bool found_release_esp32 = false;
  for (uint8_t i = 1; i <= 3; i++) {
    found_release_esp32 = probe_release_bus();
    if (found_release_esp32) {
      break;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Tentativa %d/3: Modulo UART nao encontrado. Tentando novamente em 1s...", i);
    
    if (i < 3) {
      hal.scheduler->delay(1000); // Espera 1 segundo
    }
  }

  if (found_release_esp32)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Sucesso! Modulo UART reconhecido.");
  }
  else
  {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[Adelphi] Falha critica: Modulo de alijamento UART NAO reconhecido apos 3 tentativas.");
  }
}

// Chamado em em Plane::scheduler_tasks (Plane.cpp) - 10Hz
void Adelphi::update()
{
  this->on_emergency_actived();

  if (release_esp32_uart != nullptr) {
      release_esp32_timer();
  }

  // Se não tiver fixado o GPS, aguarda
  if (!this->has_fixed_once && AP::gps().status() < AP_GPS::GPS_Status::GPS_OK_FIX_3D)
  {
    // A cada 30 ciclos de 10Hz, envia uma mensagem para o GCS (0.33 Hz)
    if (this->waiting_gps_fix % 30 == 0)
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Aguardando GPS fixar...");
      this->waiting_gps_fix = 0;
    }
    this->waiting_gps_fix++;
    return;
  }

  // Quando o GPS fixar, salva a posição inicial
  if (!this->has_fixed_once)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] GPS fixado.");
    this->has_fixed_once = true;

    this->home = AP::gps().location();
    this->home_alt = plane.relative_altitude;

    AP::adelphi().set_status(STATUS::ATTACHED);
  }

  if (this->prepared_to_release && plane.get_mode() == plane.mode_stabilize.mode_number() && this->release_esp32_data.ardupilot_release_confirmation == PlanadorInterfaceFields::NO && AP_HAL::millis() - this->prepared_to_release_time > 2000)
  {
    if (this->release_esp32_data.pilot_called_release == PlanadorInterfaceFields::YES)
    {
      auto mission = AP::mission();

      AP_Mission::Mission_Command home_cmd;
      AP_Mission::Mission_Command land;
      AP_Mission::Mission_Command flare;

      if (mission->num_commands() > 0)
      {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Editando missao");
        mission->read_cmd_from_storage(0, home_cmd);
        mission->read_cmd_from_storage(mission->num_commands() - 1, land);

        Vector2D land_point = {((float)land.content.location.lat) / 1.0e7f, ((float)land.content.location.lng) / 1.0e7f};

        Vector2D land_point_xy = latLonToCartesian(land_point.x, land_point.y, land_point.x, land_point.y);

        const Location loc = AP::gps().location();

        auto points = calculate(land_point.x, land_point.y, ((float)loc.lat) / 1.0e7f, ((float)loc.lng) / 1.0e7f, AP::ahrs().get_yaw(), 0.5, 0.05, 5, 5, 5000);

        if (!points.empty())
        {
          GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Calculando flare");
          auto flare_point_xy = findApproachPoint(land_point_xy, points, 125);

          auto flare_point = cartesianToLatLon(flare_point_xy.x, flare_point_xy.y, land_point.x, land_point.y);

          mission->clear();

          mission->add_cmd(home_cmd);
          flare.id = MAV_CMD_NAV_WAYPOINT;
          flare.p1 = 0;
          flare.content.location = Location{
              (int)(flare_point.x * 1e7),
              (int)(flare_point.y * 1e7),
              2500,
              Location::AltFrame::ABOVE_HOME};
          mission->add_cmd(flare);
          mission->add_cmd(land);

          mission->set_current_cmd(0);
        }
      }

      this->release_esp32_data.ardupilot_release_confirmation = PlanadorInterfaceFields::YES;
      this->should_write_to_release_esp32 += 5;
      // change mode to AUTO
      plane.set_mode(plane.mode_auto, ModeReason::SCRIPTING);
      AP::adelphi().set_status(STATUS::DEPLOYED);
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Alijamento liberado");
    }
    else
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Alijamento abortado");
      this->prepared_to_release = false;
      plane.set_mode(plane.mode_manual, ModeReason::SCRIPTING);
    }
  }

  // Considerar cancelamento o cancelamento para mudar o status se preciso.
  if (this->release_esp32_data.pilot_called_release == PlanadorInterfaceFields::YES && !this->prepared_to_release)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Preparando alijamento...");
    this->prepared_to_release = true;
    this->prepared_to_release_time = AP_HAL::millis();
    // change mode to STABILIZE
    plane.set_mode(plane.mode_stabilize, ModeReason::SCRIPTING);
  }

  // Se passar muito tempo depois de preparado e o modo voltar para o manual, significa que o planador pouso externamente
  if (this->prepared_to_release && plane.get_mode() == plane.mode_manual.mode_number() && AP_HAL::millis() - this->prepared_to_release_time > 5000 && !this->hasLanded)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Pouso concluido");
    AP::adelphi().set_status(STATUS::LANDED);
    this->hasLanded = true;
  }
}

void Adelphi::on_land()
{
  GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Pouso concluido");
  AP::adelphi().set_status(STATUS::LANDED);
}

#pragma region ESP32 UART Communication
void Adelphi::release_esp32_timer()
{
  static uint32_t last_send = 0;
  if (release_esp32_uart != nullptr && (should_write_to_release_esp32 > 0 || AP_HAL::millis() - last_send > 1000))
  {
    if (should_write_to_release_esp32 > 0) should_write_to_release_esp32--;
    last_send = AP_HAL::millis();
    
    release_esp32_data.ardupilot_millis = AP_HAL::millis();
    uint8_t checksum = calcChecksum((uint8_t *)(&release_esp32_data), sizeof(PlanadorInterfacePacket) - sizeof(uint32_t));
    release_esp32_data.checksum = checksum;
    release_esp32_uart->write((uint8_t *)(&release_esp32_data), sizeof(PlanadorInterfacePacket));
  }
  // read uart buffer
  release_esp32_read();
}

bool Adelphi::release_esp32_read()
{
  if (release_esp32_uart == nullptr) return false;

  bool got_packet = false;
  static uint8_t buf[sizeof(PlanadorInterfacePacket)];
  static uint8_t buf_idx = 0;

  while (release_esp32_uart->available() > 0) {
      uint8_t b = release_esp32_uart->read();
      buf[buf_idx++] = b;

      if (buf_idx == 1 && buf[0] != 0x55) { // First byte of header 0xAA55 in little endian
          buf_idx = 0;
      }
      else if (buf_idx == 2 && buf[1] != 0xAA) {
          buf_idx = 0;
      }
      else if (buf_idx == sizeof(PlanadorInterfacePacket)) {
          memcpy(&release_esp32_data_temp, buf, sizeof(PlanadorInterfacePacket));
          buf_idx = 0;
          
          uint32_t received_checksum = release_esp32_data_temp.checksum;
          uint32_t calculated_checksum = calcChecksum((uint8_t *)(&release_esp32_data_temp), sizeof(PlanadorInterfacePacket) - sizeof(uint32_t));

          if (release_esp32_data_temp.id == 0x69 && received_checksum == calculated_checksum)
          {
              release_esp32_data.pilot_called_release = release_esp32_data_temp.pilot_called_release;
              release_esp32_last_read_t = AP_HAL::millis();
              got_packet = true;
          }
      }
  }
  return got_packet;
}

bool Adelphi::probe_release_bus()
{
  release_esp32_uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Adelphi, 0);
  if (!release_esp32_uart)
  {
    return false;
  }
  
  release_esp32_uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Adelphi, 0));

  // Send a packet to trigger response
  uint8_t checksum = calcChecksum((uint8_t *)(&release_esp32_data), sizeof(PlanadorInterfacePacket) - sizeof(uint32_t));
  release_esp32_data.checksum = checksum;
  release_esp32_uart->write((uint8_t *)(&release_esp32_data), sizeof(PlanadorInterfacePacket));

  bool found = false;
  for (int i = 0; i < 100; i++) // 100 * 10ms = 1s timeout
  {
    bool success = release_esp32_read();
    if (success)
    {
      found = true;
      break;
    }
    hal.scheduler->delay(10);
  }

  return found;
}

uint8_t calcChecksum(uint8_t *buffer, uint8_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++)
  {
    checksum ^= buffer[i];
  }
  return checksum;
}
#pragma endregion ESP32 UART Communication

#pragma region Path Calculation Functions
// Convert latitude/longitude to Cartesian coordinates relative to a reference point
Vector2D latLonToCartesian(float lat, float lon, float lat_ref, float lon_ref)
{
  // Convert degrees to radians
  float lat_rad = lat * M_PI / 180.0;
  float lon_rad = lon * M_PI / 180.0;
  float lat_ref_rad = lat_ref * M_PI / 180.0;
  float lon_ref_rad = lon_ref * M_PI / 180.0;

  // Calculate x and y in meters
  float x = EARTH_RADIUS * (lon_rad - lon_ref_rad) * cosf((float)((lat_rad + lat_ref_rad) / 2.0f));
  float y = EARTH_RADIUS * (lat_rad - lat_ref_rad);

  return {x, y};
}

// Convert Cartesian coordinates to latitude/longitude relative to a reference point
Vector2D cartesianToLatLon(float x, float y, float lat_ref, float lon_ref)
{
  float lat_ref_rad = lat_ref * M_PI / 180.0;
  float lon_ref_rad = lon_ref * M_PI / 180.0;

  // Calculate latitude and longitude in radians
  float lat_rad = lat_ref_rad + (y / EARTH_RADIUS);
  float lon_rad = lon_ref_rad + (x / (EARTH_RADIUS * cosf((float)(lat_rad + lat_ref_rad) / 2.0f)));

  // Convert radians back to degrees
  float lat = lat_rad * 180.0 / M_PI;
  float lon = lon_rad * 180.0 / M_PI;

  return {lat, lon};
}

// Convert heading (in degrees) to a 2D directional vector
Vector2D headingToVector(float heading)
{
  float heading_rad = heading * M_PI / 180.0;
  float x = sinf(heading_rad);
  float y = cosf(heading_rad);
  return {x, y};
}

// Calculate path points towards the target point
std::vector<Vector2D> calculate(float target_lat, float target_lon, float current_lat, float current_lon, float initial_heading, float speed, float steer_factor, float max_bank, float max_dist, int max_iterations)
{
  max_bank = max_bank * M_PI / 180.0;

  // Convert target and current latitude/longitude to Cartesian coordinates
  Vector2D target_point = latLonToCartesian(target_lat, target_lon, target_lat, target_lon);
  Vector2D current_point = latLonToCartesian(current_lat, current_lon, target_lat, target_lon);

  // Calculate initial forward vector based on heading
  Vector2D forward_vector = headingToVector(initial_heading).normalized();

  std::vector<Vector2D> points;
  points.push_back(current_point);

  int walk_towards_target_iter = 0;
  bool converged = true;

  while ((target_point - current_point).magnitude() > max_dist)
  {
    walk_towards_target_iter++;

    Vector2D desired_direction = (target_point - current_point).normalized();
    Vector2D steer = desired_direction - forward_vector;
    forward_vector = (forward_vector + steer * steer_factor).normalized();

    current_point = current_point + forward_vector * speed;
    points.push_back(current_point);

    if (walk_towards_target_iter > max_iterations)
    {
      converged = false;
      break;
    }
    else
    {
      converged = true;
    }
  }

  if (converged)
  {
    return points;
  }
  else
  {
    points.clear();
    return points;
  }
}

Vector2D findApproachPoint(const Vector2D &target_point, const std::vector<Vector2D> &points, float approach_distance_from_target)
{
  Vector2D approach_point = target_point;

  for (const auto &point : points)
  {
    float dist_to_target_new = (point - target_point).magnitude();
    float dist_to_target_cur = (approach_point - target_point).magnitude();

    if (dist_to_target_new < approach_distance_from_target && dist_to_target_new > dist_to_target_cur)
    {
      approach_point = point;
    }
  }

  return approach_point;
}
#pragma endregion Path Calculation Functions