#include "Adelphi.h"
#include "Plane.h"

// Construtor
const AP_Param::GroupInfo Adelphi::var_info[] = {

    // @Param: GLIDES
    // @DisplayName: Adelphi Glide Slope Ratio (L/D)
    // @Description: Rampa de planeio final (razao L/D de solo) para dimensionamento do flare/aproximacao
    // @Range: 1.0 10.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GLIDES", 1, Adelphi, glide_slope, 3.8f),

    // @Param: RFACTOR
    // @DisplayName: Adelphi Flare Altitude Ratio Factor
    // @Description: Fracao da altitude atual de soltura reservada para a rampa final de flare (ex: 0.35 = 35%)
    // @Range: 0.10 0.80
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RFACTOR", 2, Adelphi, r_factor, 0.35f),

    AP_GROUPEND
};

Adelphi::Adelphi()
{
  AP_Param::setup_object_defaults(this, var_info);
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

// Chamado em Plane::scheduler_tasks (Plane.cpp) - 10Hz
void Adelphi::update()
{
  this->on_emergency_actived();

  // 1. Leitura imediata do buffer RX da UART antes de processar a maquina de estados
  if (release_esp32_uart != nullptr) {
    release_esp32_read();
  }

  // Timeout de recepcao UART do ESP32 (1500 ms)
  if (AP_HAL::millis() - this->release_esp32_last_read_t > 1500)
  {
    this->release_esp32_data.pilot_called_release = PlanadorInterfaceFields::NO;
  }

  // 2. Monitoramento continuo de armamento (executa SEMPRE, mesmo antes do GPS 3D Fix)
  const bool is_armed = plane.arming.is_armed();
  static bool was_armed = false;

  // Se o planador estiver DESARMADO em qualquer momento:
  // 1. A confirmacao de liberacao do FC e a flag de requisicao sao imediatamente limpas (NO)
  // 2. Qualquer processo de liberacao/preparacao em andamento e abortado e timers zerados
  // 3. Se estava na janela de preparacao em FBWA, reverte o modo para MANUAL
  // 4. Status retorna para ATTACHED (se GPS fixado) ou UNINITIALIZED
  if (!is_armed)
  {
    if (this->prepared_to_release)
    {
      GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Alijamento abortado imediatamente: Planador DESARMADO!");
      if (plane.get_mode() == plane.mode_fbwa.mode_number())
      {
        plane.set_mode(plane.mode_manual, ModeReason::SCRIPTING);
      }
    }

    if (this->release_esp32_data.ardupilot_release_confirmation != PlanadorInterfaceFields::NO)
    {
      this->release_esp32_data.ardupilot_release_confirmation = PlanadorInterfaceFields::NO;
      this->should_write_to_release_esp32 = 5;
    }

    this->release_esp32_data.pilot_called_release = PlanadorInterfaceFields::NO;
    this->release_esp32_data.is_armed = 0;
    this->prepared_to_release = false;
    this->prepared_to_release_time = 0;
    this->hasLanded = false;

    if (AP::adelphi().get_status() == STATUS::DEPLOYED || AP::adelphi().get_status() == STATUS::LANDED)
    {
      AP::adelphi().set_status(this->has_fixed_once ? STATUS::ATTACHED : STATUS::UNINITIALIZED);
    }
  }

  if (was_armed && !is_armed)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Desarme detectado. Resetando maquina de estados e propagando via UART.");
    this->should_write_to_release_esp32 = 5;
  }
  was_armed = is_armed;

  // Se não tiver fixado o GPS, aguarda (mas mantém o envio da telemetria/armamento na UART)
  if (!this->has_fixed_once && AP::gps().status() < AP_GPS::GPS_Status::GPS_OK_FIX_3D)
  {
    if (this->waiting_gps_fix % 30 == 0)
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Aguardando GPS fixar...");
      this->waiting_gps_fix = 0;
    }
    this->waiting_gps_fix++;

    if (release_esp32_uart != nullptr) {
      release_esp32_timer();
    }
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

  // Se estiver preparando para alijar, mas o piloto cancelou (ou link UART caiu), aborta imediatamente
  if (this->prepared_to_release && this->release_esp32_data.ardupilot_release_confirmation == PlanadorInterfaceFields::NO)
  {
    if (!is_armed || this->release_esp32_data.pilot_called_release != PlanadorInterfaceFields::YES)
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Alijamento abortado: solicitacao desfeita ou desarmado");
      this->prepared_to_release = false;
      this->prepared_to_release_time = 0;
      if (plane.get_mode() == plane.mode_fbwa.mode_number())
      {
        plane.set_mode(plane.mode_manual, ModeReason::SCRIPTING);
      }
      this->should_write_to_release_esp32 = 5;
    }
  }

  // Inicio da preparacao: OBRIGATORIAMENTE requer estar ARMADO
  // O recalculo da trajetoria e antecipado para este exato instante (quando entra em FBWA),
  // projetando a posicao futura de soltura para 2.0s a frente somando a velocidade de solo (groundspeed_vector).
  if (is_armed && this->release_esp32_data.pilot_called_release == PlanadorInterfaceFields::YES && !this->prepared_to_release && this->release_esp32_data.ardupilot_release_confirmation == PlanadorInterfaceFields::NO)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Preparando alijamento (FBWA)...");
    this->prepared_to_release = true;
    this->prepared_to_release_time = AP_HAL::millis();
    // change mode to FBWA
    plane.set_mode(plane.mode_fbwa, ModeReason::SCRIPTING);
    this->should_write_to_release_esp32 = 5;

    auto mission = AP::mission();
    AP_Mission::Mission_Command home_cmd;
    AP_Mission::Mission_Command land;
    AP_Mission::Mission_Command flare = {};

    if (mission != nullptr && mission->num_commands() > 0)
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Editando missao (V2 Dinamico em FBWA)");
      mission->read_cmd_from_storage(0, home_cmd);
      mission->read_cmd_from_storage(mission->num_commands() - 1, land);

      // 1. Posicao e altitude atuais
      Location predicted_loc;
      if (!AP::ahrs().get_location(predicted_loc)) {
        predicted_loc = AP::gps().location();
      }

      // 2. Extrapolar posicao para 2.0 segundos a frente (momento da soltura) usando o vetor de velocidade de solo
      Vector2f ground_vel = AP::ahrs().groundspeed_vector();
      if (ground_vel.is_zero() && AP::gps().status() >= AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
        const Vector3f &gps_vel = AP::gps().velocity();
        ground_vel.x = gps_vel.x;
        ground_vel.y = gps_vel.y;
      }

      const float PREPARATION_TIME_SEC = 2.0f;
      float ofs_north = ground_vel.x * PREPARATION_TIME_SEC;
      float ofs_east = ground_vel.y * PREPARATION_TIME_SEC;
      predicted_loc.offset(ofs_north, ofs_east);

      // Altura atual acima do Home (em metros)
      float current_alt_m = 50.0f;
      float pos_d;
      AP::ahrs().get_relative_position_D_home(pos_d);
      current_alt_m = -pos_d;
      if (current_alt_m <= 0.0f) {
        current_alt_m = plane.relative_altitude;
      }
      if (current_alt_m <= 0.0f) {
        current_alt_m = 50.0f;
      }

      // 3. Distancia estimada da soltura futura (+2s) ate o ponto de pouso
      float total_dist_m = predicted_loc.get_distance(land.content.location);

      // 4. Dimensionamento dinamico da rampa final baseado no L/D_solo e % de altitude configurados via parametros
      float r_factor_val = constrain_float(this->r_factor.get(), 0.10f, 0.80f);
      float flare_wp_alt_m = constrain_float(current_alt_m * r_factor_val, 15.0f, 30.0f);

      const float GLIDE_RATIO_FINAL = (this->glide_slope.get() > 0.5f) ? this->glide_slope.get() : 3.8f;
      float approach_dist_m = flare_wp_alt_m * GLIDE_RATIO_FINAL;

      // Garante que o WP1 nunca fique mais longe que 70% da distancia total da soltura estimada
      if (approach_dist_m > total_dist_m * 0.70f) {
        approach_dist_m = total_dist_m * 0.70f;
        flare_wp_alt_m = approach_dist_m / GLIDE_RATIO_FINAL;
      }

      Vector2D land_point = {((float)land.content.location.lat) / 1.0e7f, ((float)land.content.location.lng) / 1.0e7f};
      Vector2D land_point_xy = latLonToCartesian(land_point.x, land_point.y, land_point.x, land_point.y);

      auto points = calculate(
          land_point.x, land_point.y,
          ((float)predicted_loc.lat) / 1.0e7f, ((float)predicted_loc.lng) / 1.0e7f,
          degrees(AP::ahrs().get_yaw()),
          0.5f, 0.01f, 5.0f, 5.0f, 5000
      );

      if (!points.empty())
      {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Flare previsto (+2s): dist=%.1fm alt=%.1fm L/D=%.1f",
                      (double)approach_dist_m, (double)flare_wp_alt_m, (double)GLIDE_RATIO_FINAL);

        auto flare_point_xy = findApproachPoint(land_point_xy, points, approach_dist_m);
        auto flare_point = cartesianToLatLon(flare_point_xy.x, flare_point_xy.y, land_point.x, land_point.y);

        mission->clear();

        mission->add_cmd(home_cmd);

        flare.id = MAV_CMD_NAV_WAYPOINT;
        flare.p1 = 0;
        flare.content.location = Location{
            (int32_t)(flare_point.x * 1e7f),
            (int32_t)(flare_point.y * 1e7f),
            (int32_t)(flare_wp_alt_m * 100.0f), // Convertido para cm
            Location::AltFrame::ABOVE_HOME};

        mission->add_cmd(flare);
        mission->add_cmd(land);

        // Inicia diretamente no comando 1 (WP1 / flare)
        mission->set_current_cmd(1);
      }
    }
  }

  // Liberacao efetiva: requer OBRIGATORIAMENTE estar ARMADO, preparado, modo FBWA/STABILIZE,
  // confirmacao pendente (NO) e 2000 ms decorridos desde a preparacao
  if (is_armed && this->prepared_to_release && (plane.get_mode() == plane.mode_fbwa.mode_number() || plane.get_mode() == plane.mode_stabilize.mode_number()) && this->release_esp32_data.ardupilot_release_confirmation == PlanadorInterfaceFields::NO && AP_HAL::millis() - this->prepared_to_release_time > 2000)
  {
    if (this->release_esp32_data.pilot_called_release == PlanadorInterfaceFields::YES)
    {
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

  // Se passar muito tempo depois de preparado e o modo voltar para o manual, significa que o planador pousou externamente
  if (is_armed && this->prepared_to_release && plane.get_mode() == plane.mode_manual.mode_number() && AP_HAL::millis() - this->prepared_to_release_time > 5000 && !this->hasLanded)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Pouso concluido");
    AP::adelphi().set_status(STATUS::LANDED);
    this->hasLanded = true;
    this->should_write_to_release_esp32 = 5;
  }

  // 3. Transmissao imediata para o ESP32 apos todas as atualizacoes de estado do ciclo
  if (release_esp32_uart != nullptr) {
    release_esp32_timer();
  }
}

void Adelphi::on_land()
{
  GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Pouso concluido");
  AP::adelphi().set_status(STATUS::LANDED);
  this->release_esp32_data.ardupilot_release_confirmation = PlanadorInterfaceFields::NO;
  this->release_esp32_data.pilot_called_release = PlanadorInterfaceFields::NO;
  this->prepared_to_release = false;
  this->prepared_to_release_time = 0;
  this->hasLanded = true;
  this->should_write_to_release_esp32 = 5;
}

#pragma region ESP32 UART Communication
void Adelphi::release_esp32_timer()
{
  static uint32_t last_send = 0;
  if (release_esp32_uart != nullptr && (should_write_to_release_esp32 > 0 || AP_HAL::millis() - last_send >= 100))
  {
    if (should_write_to_release_esp32 > 0) should_write_to_release_esp32--;
    last_send = AP_HAL::millis();
    
    const bool is_armed = plane.arming.is_armed();
    if (!is_armed) {
      release_esp32_data.ardupilot_release_confirmation = PlanadorInterfaceFields::NO;
      release_esp32_data.pilot_called_release = PlanadorInterfaceFields::NO;
    }

    release_esp32_data.ardupilot_millis = AP_HAL::millis();
    release_esp32_data.flight_mode = plane.get_mode();
    release_esp32_data.adelphi_status = (uint8_t)AP::adelphi().get_status();
    release_esp32_data.is_armed = is_armed ? 1 : 0;
    release_esp32_data.gps_fix = (uint8_t)AP::gps().status();
    release_esp32_data.satellites = AP::gps().num_sats();
    release_esp32_data.altitude_cm = (int32_t)(plane.relative_altitude * 100.0f);

    if (AP::battery().num_instances() > 0) {
      release_esp32_data.battery_voltage_mv = (uint16_t)(AP::battery().voltage(0) * 1000.0f);
    } else {
      release_esp32_data.battery_voltage_mv = 0;
    }

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
          buf_idx = (b == 0x55) ? 1 : 0;
          if (buf_idx == 1) buf[0] = 0x55;
      }
      else if (buf_idx == 3 && buf[2] != 0x69) {
          buf_idx = (b == 0x55) ? 1 : 0;
          if (buf_idx == 1) buf[0] = 0x55;
      }
      else if (buf_idx == sizeof(PlanadorInterfacePacket)) {
          memcpy(&release_esp32_data_temp, buf, sizeof(PlanadorInterfacePacket));
          buf_idx = 0;
          
          uint32_t received_checksum = release_esp32_data_temp.checksum;
          uint32_t calculated_checksum = calcChecksum((uint8_t *)(&release_esp32_data_temp), sizeof(PlanadorInterfacePacket) - sizeof(uint32_t));

          if (release_esp32_data_temp.id == 0x69 && received_checksum == calculated_checksum)
          {
              release_esp32_data.pilot_called_release = plane.arming.is_armed()
                  ? release_esp32_data_temp.pilot_called_release
                  : PlanadorInterfaceFields::NO;
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
// NOTA [CRÍTICA-02]: Esta função espera `heading` em GRAUS (converte para radianos multiplicando por M_PI / 180.0),
// porém na chamada de `calculate(...)` em Adelphi::update() está sendo passado `AP::ahrs().get_yaw()`, que retorna
// o yaw em RADIANOS ([0, 2*PI) / [-PI, PI]). Para usar a proa real em graus, deve-se passar `degrees(AP::ahrs().get_yaw())`
// ou remover a conversão `* M_PI / 180.0` abaixo.
Vector2D headingToVector(float heading)
{
  float heading_rad = heading * M_PI / 180.0;
  float x = sinf(heading_rad);
  float y = cosf(heading_rad);
  return {x, y};
}

// Calculate path points towards the target point
// NOTA [CRÍTICA-02]: `initial_heading` é repassado diretamente para `headingToVector(initial_heading)`.
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