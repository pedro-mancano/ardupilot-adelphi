#include "Adelphi.h"
#include "Plane.h"

// Construtor
Adelphi::Adelphi()
{
}

// Destrutor
Adelphi::~Adelphi()
{
  // Fecha o arquivo
  AP::FS().close(this->file);
}

// Inicializa o arquivo de log
void Adelphi::init()
{
  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Inicializando...");

  int log_count = this->log_count();
  char log_filename[32]{0};
  if (log_count == 0)
  {
    hal.util->snprintf(log_filename, 32, "%s%s", ADELPHI_LOG_FILE_NAME, ADELPHI_LOG_FILE_EXT);
  }
  else
  {
    hal.util->snprintf(log_filename, 32, "%s_%d%s", ADELPHI_LOG_FILE_NAME, log_count, ADELPHI_LOG_FILE_EXT);
  }

  EXPECT_DELAY_MS(3000);
  this->file =
      AP::FS().open(log_filename, O_WRONLY | O_CREAT | O_TRUNC);

  AP::FS().write(this->file, this->header, strlen(this->header));
  AP::FS().fsync(this->file);

  GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Inicializado.");

  this->writebuf.set_size(4096);
  this->writebuf.clear();
  this->_write_offset = 0;

  if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Adelphi::io_thread, void), "adelphi_log_io", 4096, AP_HAL::Scheduler::PRIORITY_IO, 1))
  {
    AP_HAL::panic("Failed to start Adelphi IO thread");
  }

  // Inicializar conexão com o ESP32
  bool found_esp32 = false;

  static const uint8_t addresses[] = {0x69};

  FOREACH_I2C_EXTERNAL(bus)
  {
    for (uint8_t addr : addresses)
    {
      if (probe_bus(bus, addr))
      {
        found_esp32 = true;
        goto exit_sensor_loop;
      }
    }
  }
  FOREACH_I2C_INTERNAL(bus)
  {
    for (uint8_t addr : addresses)
    {
      if (probe_bus(bus, addr))
      {
        found_esp32 = true;
        goto exit_sensor_loop;
      }
    }
  }

exit_sensor_loop:

  if (!found_esp32)
  {

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ESP32Planador[Adelphi]: not found");
  }
  else
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESP32Planador[Adelphi]: Found bus %u addr 0x%02x", esp32_device->bus_num(), esp32_device->get_bus_address());
    // drop to 2 retries for runtime (better performance)
    esp32_device->set_retries(2);
    esp32_device->register_periodic_callback(20000,
                                             FUNCTOR_BIND_MEMBER(&Adelphi::esp32_timer, void));
  }
}

void Adelphi::io_timer()
{
  uint32_t last_run_us = AP_HAL::micros();

  while (true)
  {
    uint32_t now = AP_HAL::micros();

    uint32_t delay = 250U;
    if (now - last_run_us < 1000)
    {
      delay = MAX(1000 - (now - last_run_us), delay);
    }
    hal.scheduler->delay_microseconds(delay);
    last_run_us = AP_HAL::micros();

    uint32_t nbytes = this->writebuf.available();

    if (nbytes == 0)
    {
      continue;
    }

    uint32_t size;
    const uint8_t *head = this->writebuf.readptr(size);
    nbytes = MIN(nbytes, size);

    if ((nbytes + _write_offset) % 512 != 0)
    {
      uint32_t ofs = (nbytes + _write_offset) % 512;
      if (ofs < nbytes)
      {
        nbytes -= ofs;
      }
    }

    if (nbytes > 0)
    {
      this->sem.take(1);
      ssize_t nwritten = AP::FS().write(this->file, head, nbytes);
      AP::FS().fsync(this->file);
      this->sem.give();
      this->writebuf.advance(nwritten);
      _write_offset += nwritten;
    }
  }
}

void Adelphi::io_thread()
{
  this->io_timer();
}

// Atualiza o arquivo de log
// Chamado em em Plane::scheduler_tasks (Plane.cpp) - 10Hz
void Adelphi::update()
{
  // Se não tiver fixado o GPS, aguarda
  if (!this->has_fixed_once && AP::gps().status() < AP_GPS::GPS_Status::GPS_OK_FIX_3D)
  {
    // A cada 10 ciclos de 10Hz, envia uma mensagem para o GCS (1 Hz)
    if (this->waiting_gps_fix % 10 == 0)
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

    // Calcula o tempo base com base no tempo do GPS (UTC)
    // Aplica % 86400000 para obter o tempo do dia em milisegundos
    // Subtrai o tempo atual em milisegundos e divide por 1000 para obter o tempo em segundos
    // Subtrai 10800 para converter de UTC para BRT
    this->base_time = (AP::gps().time_week_ms() % 86400000 - AP_HAL::millis()) / 1000.0 - 10800.0;

    AP::adelphi().set_status(STATUS::ATTACHED);
  }

  // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] In release condition: %lu, should alert plane of release: %d, ardupilot release confirmation: %d, id: %d", this->esp32_data.in_release_condition, this->esp32_data.should_alert_plane_of_release, this->esp32_data.ardupilot_release_confirmation, this->esp32_data.id);

  if (this->prepared && plane.get_mode() == plane.mode_stabilize.mode_number() && this->esp32_data.ardupilot_release_confirmation == 0 && AP_HAL::millis() - this->prepared_time > 2000)
  {
    if (this->esp32_data.in_release_condition == 1)
    {
      auto mission = AP::mission();

      AP_Mission::Mission_Command home_cmd;
      AP_Mission::Mission_Command land;
      AP_Mission::Mission_Command flare;

      if (mission->num_commands() > 0)
      {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Editando missao");
        mission->read_cmd_from_storage(0, home_cmd);
        mission->read_cmd_from_storage(mission->num_commands() - 1, land);

        Vector2D land_point = {((float)land.content.location.lat) / 1.0e7f, ((float)land.content.location.lng) / 1.0e7f};

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Latitude: %f, Longitude: %f", land_point.x, land_point.y);

        Vector2D land_point_xy = latLonToCartesian(land_point.x, land_point.y, land_point.x, land_point.y);

        const Location loc = AP::gps().location();

        auto points = calculate(land_point.x, land_point.y, ((float)loc.lat) / 1.0e7f, ((float)loc.lng) / 1.0e7f, AP::ahrs().get_yaw(), 0.5, 0.05, 5, 5, 5000);

        if (!points.empty())
        {
          GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Calculando flare");
          auto flare_point_xy = findApproachPoint(land_point_xy, points, 51);

          auto flare_point = cartesianToLatLon(flare_point_xy.x, flare_point_xy.y, land_point.x, land_point.y);

          mission->clear();

          mission->add_cmd(home_cmd);
          flare.id = MAV_CMD_NAV_WAYPOINT;
          flare.p1 = 0;
          flare.content.location = Location{
              (int)(flare_point.x * 1e7),
              (int)(flare_point.y * 1e7),
              1500,
              Location::AltFrame::ABSOLUTE};
          mission->add_cmd(flare);
          mission->add_cmd(land);

          mission->set_current_cmd(0);
        }
        else
        {
          // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Pontos vazios missao");
        }
      }

      this->esp32_data.ardupilot_release_confirmation = this->esp32_data.id;
      this->should_write_to_esp32 = true;
      // change mode to AUTO
      plane.set_mode(plane.mode_auto, ModeReason::SCRIPTING);
      AP::adelphi().set_status(STATUS::DEPLOYED);
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Alijamento liberado");
    }
    else
    {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Alijamento abortado");
      this->prepared = false;
      plane.set_mode(plane.mode_manual, ModeReason::SCRIPTING);
    }
  }

  // Consderar cancelamento o cancelmaneto para mudar o status se preciso.

  if (this->esp32_data.in_release_condition == 1 && !this->prepared)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Preparando alijamento...");
    this->prepared = true;
    this->prepared_time = AP_HAL::millis();
    // change mode to STABILIZE
    plane.set_mode(plane.mode_stabilize, ModeReason::SCRIPTING);
  }

  const double now = this->base_time + (AP_HAL::millis() / 1000.0);

  const Location loc = AP::gps().location();
  const Vector3f dist3d = loc.get_distance_NED(this->home);
  const float alt = plane.relative_altitude - this->home_alt;

  // Euler angles in radians to degrees
  float roll = wrap_360(AP::ahrs().get_roll() * RAD_TO_DEGf);
  float pitch = wrap_360(AP::ahrs().get_pitch() * RAD_TO_DEGf);
  float yaw = wrap_360(AP::ahrs().get_yaw() * RAD_TO_DEGf);

  const float aoa = AP::ahrs().getAOA();
  const float aos = AP::ahrs().getSSA();

  uint16_t aileron_pwm = 0;
  SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, aileron_pwm);
  uint16_t elevator_pwm = 0;
  SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, elevator_pwm);
  uint16_t rudder_pwm = 0;
  SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, rudder_pwm);

  const float aileron = ((float)aileron_pwm) * 0.06f - 90.0f;
  const float elevator = ((float)elevator_pwm) * 0.0635575823338f - 91.2121469838f;
  const float rudder = ((float)rudder_pwm) * -0.0448554091f + 68.6076596501f;

  // Aguardar armar para começar a gravar
  if (!this->has_armed && plane.arming.is_armed())
  {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[Adelphi] Iniciando...");
    this->has_armed = true;
  }

  // Se desarmar, para de gravar
  if (this->has_armed && !plane.arming.is_armed())
  {
    this->prepared = false;
    this->has_armed = false;
    return;
  }

  // Se não tiver armado, não grava
  if (!this->has_armed)
  {
    return;
  }

  char buf[128];
  // "Tempo\tXGPS\tYGPS\tZGPS\tELEV\tAIL\tRUD\tTHETA\tPHI\tPSI\tStatus\tAOA\tAOS\n";
  hal.util->snprintf((char *)buf, 128, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%f\t%f\n",
                     now,                             // Tempo
                     dist3d.x,                        // XGPS
                     dist3d.y,                        // YGPS
                     alt,                             // ZGPS
                     elevator,                        // ELEV
                     aileron,                         // AIL
                     rudder,                          // RUD
                     pitch,                           // THETA
                     roll,                            // PHI
                     yaw,                             // PSI
                     (int)AP::adelphi().get_status(), // Status
                     aoa,                             // AOA
                     aos                              // AOS
  );

  this->writeBlock((uint8_t *)buf, strlen(buf));
}

void Adelphi::writeBlock(const uint8_t *pBuffer, uint16_t size)
{
  // Maybe our buffer doesn't have enough space
  uint32_t space = this->writebuf.space();

  // If the buffer is full, drop the data
  if (space < size)
  {
    this->dropped_count++;
    return;
  }

  this->writebuf.write(pBuffer, size);
}

int Adelphi::log_count()
{
  auto *d = AP::FS().opendir(".");
  if (d == nullptr)
  {
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "[Adelphi] Failed to open home dir");
    return -1;
  }

  uint32_t count = 0;

  EXPECT_DELAY_MS(2000);

  for (auto de = AP::FS().readdir(d); de; de = AP::FS().readdir(d))
  {
    EXPECT_DELAY_MS(100);

    uint32_t size = strlen(de->d_name);

    if (size <= 4)
    {
      continue;
    }

    // Check if de->d_name starsWith ADELPHI_LOG_FILE_NAME
    if (strncmp(de->d_name, ADELPHI_LOG_FILE_NAME, strlen(ADELPHI_LOG_FILE_NAME)) == 0)
    {
      count++;
    }
  }

  AP::FS().closedir(d);

  return count;
}

void Adelphi::esp32_timer()
{
  if (should_write_to_esp32)
  {
    uint8_t checksum = calcChecksum((uint8_t *)(&esp32_data), sizeof(DataFromPlanador) - sizeof(uint32_t));
    esp32_data.checksum = checksum;
    esp32_device->transfer((uint8_t *)(&esp32_data), sizeof(DataFromPlanador), nullptr, 0);
    should_write_to_esp32 = false;
  }
  // read i2c buffer on 0x69
  esp32_read();
}

bool Adelphi::esp32_read()
{

  if (esp32_device->read((uint8_t *)(&esp32_data_temp), sizeof(DataFromPlanador)))
  {
    // if (esp32_data_temp.checksum != calcChecksum((uint8_t *)(&esp32_data_temp), sizeof(DataFromPlanador) - sizeof(uint32_t)))
    //{
    //   GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ESP32Planador[Adelphi]: Checksum error, expected %d got %d", calcChecksum((uint8_t *)(&esp32_data_temp), sizeof(DataFromPlanador) - sizeof(uint32_t)), esp32_data_temp.checksum);
    //   return false;
    // }
    if (esp32_data_temp.id == 0x69)
    {
      esp32_data = esp32_data_temp;
      esp32_last_read_t = AP_HAL::millis();
      return true;
    }
  }
  return false;
}

bool Adelphi::probe_bus(uint8_t bus, uint8_t address)
{
  esp32_device = hal.i2c_mgr->get_device(bus, address);
  if (!esp32_device)
  {
    return false;
  }

  WITH_SEMAPHORE(esp32_device->get_semaphore());

  esp32_device->read((uint8_t *)(&esp32_data), sizeof(DataFromPlanador));
  // lots of retries during probe
  esp32_device->set_retries(10);

  bool found = false;
  for (int i = 0; i < 10; i++)
  {
    bool success = esp32_read();
    if (success)
    {
      found = true;
      break;
    }
  }

  return found;
}

void Adelphi::on_land()
{
  GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[Adelphi] Pouso concluido");
  AP::adelphi().set_status(STATUS::LANDED);
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