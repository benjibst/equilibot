#include "tmc5240.hpp"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include <cmath>
#include <algorithm>
#include <cstring>
constexpr auto kTag = "tmc5240";
namespace tmc5240
{

    TMC5240::TMC5240(SpiBus &spi_bus, gpio_num_t en_pin, gpio_num_t cs_pin, int device_id, float rref_kohm, float motor_current_rms)
        : spi_bus(spi_bus), dev_id(device_id), rref_kohm(rref_kohm), motor_current_rms(motor_current_rms)
    {
        // maximum ifs and rref according to datasheet
        assert(rref_kohm >= 12 && rref_kohm <= 60);
        assert(motor_current_rms < 2.12f);

        SpiDeviceConfig devconfig = {
            .mode = SpiDeviceConfig::Mode::CPOL1_CPHA1,
            .cs = cs_pin,
            .clk_speed_hz = SPI_MASTER_FREQ_10M,
        };
        spi_bus.add_device(device_id, devconfig);
        gpio_config_t en_conf{
            .pin_bit_mask = (1ULL << static_cast<uint64_t>(en_pin)),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&en_conf));
        ESP_ERROR_CHECK(gpio_set_level(en_pin, 0));

        ReadDatagram data;
        ESP_ERROR_CHECK(read_reg<IOIN>(data));
        uint32_t payload = data.payload();
        ESP_LOGI(__FILE__, "TMC5240 Silicon Revision: %x", IOIN::silicon_rv.read_from(payload));
        ESP_LOGI(__FILE__, "TMC5240 Version: %x", IOIN::version.read_from(payload));

        ESP_ERROR_CHECK(config_current());
        ESP_ERROR_CHECK(config_motion_ctrl_vel_mode());
        ESP_ERROR_CHECK(config_stealthchop2());
        ESP_ERROR_CHECK(stealthchop2_auto_tune());
        // ESP_ERROR_CHECK(config_spreadcycle());
    }

    esp_err_t TMC5240::set_velocity(float rpm)
    {
        constexpr int microstep_res = 256;
        constexpr int steps_per_rot = 200;
        constexpr float microsteps_per_rpm = microstep_res * steps_per_rot / 60.0f;
        int32_t vel = rpm * microsteps_per_rpm;
        const uint32_t vel_abs = std::clamp<uint32_t>(std::abs(vel), 0ul, (1ul << 23) - 1ul);
        if (vel >= 0)
        {
            write(RAMPMODE().set_field(RAMPMODE::rampmode, 1));
        }
        else
        {
            write(RAMPMODE().set_field(RAMPMODE::rampmode, 2));
        }
        write(VMAX().set_field(VMAX::vmax, vel_abs));
        return ESP_OK;
    }

    constexpr float to_rms(float i)
    {
        return i / M_SQRT2;
    }
    float ifs_rms(float kifs, float rref)
    {
        return to_rms(kifs) / rref;
    }
    std::pair<uint32_t, float> min_drv_conf(float mot_curr_rms, float rref_kohm)
    {
        float ifs_rms_;
        if (mot_curr_rms < (ifs_rms_ = ifs_rms(11.75, rref_kohm)))
            return {0b00, ifs_rms_};
        if (mot_curr_rms < (ifs_rms_ = ifs_rms(24, rref_kohm)))
            return {0b01, ifs_rms_};
        if (mot_curr_rms < (ifs_rms_ = ifs_rms(36, rref_kohm)))
            return {0b10, ifs_rms_};
        __builtin_unreachable();
    }

    esp_err_t TMC5240::config_current()
    {

        auto [drv_conf, ifs_rms] = min_drv_conf(motor_current_rms, rref_kohm);

        write(DRV_CONF().set_field(DRV_CONF::current_range, drv_conf));

        uint32_t globalscaler = motor_current_rms * 256 / ifs_rms;

        write(GLOBAL_SCALER().set_field(GLOBAL_SCALER::globalscaler, globalscaler));

        write(GCONF().set_field(GCONF::en_pwm_mode, 0));

        ESP_LOGI(__FILE__, "Current configuration parameters: Rref=%fkOhms  Irms=%fA", rref_kohm, motor_current_rms);
        ESP_LOGI(__FILE__, "Current range: %d -> Ifs: %f", drv_conf, ifs_rms);
        ESP_LOGI(__FILE__, "Globalscaler: %d/256", globalscaler);
        return ESP_OK;
    }

    esp_err_t TMC5240::config_stealthchop2()
    {

        ReadDatagram rx_data;

        update_field<GCONF>(GCONF::en_pwm_mode, 1);

        read_reg<PWMCONF>(rx_data);
        PWMCONF pwmconf{rx_data.payload()};
        pwmconf.set_field(PWMCONF::pwm_autoscale, 1)
            .set_field(PWMCONF::pwm_autograd, 1)
            .set_field(PWMCONF::pwm_meas_sd_enable, 1)
            .set_field(PWMCONF::pwm_freq, 0b00);
        write(pwmconf);

        read_reg<CHOPCONF>(rx_data);
        CHOPCONF chopconf{rx_data.payload()};
        chopconf.set_field(CHOPCONF::toff, 3)
            .set_field(CHOPCONF::tbl, 2)
            .set_field(CHOPCONF::hstrt_tfd210, 4)
            .set_field(CHOPCONF::hend_offset, 0);
        write(chopconf);

        return ESP_OK;
    }

    esp_err_t TMC5240::stealthchop2_auto_tune()
    {

        ReadDatagram rx_data;

        // AT#2 needs enough acceleration to quickly reach medium velocity.
        write(AMAX().set_field(AMAX::amax, 50000));

        read_reg<PWM_AUTO>(rx_data);
        uint32_t data = PWM_AUTO::pwm_grad_auto.read_from(rx_data.payload());
        ESP_LOGI(__FILE__, "PWM_GRAD_AUTO Initially: %d", data);

        // set speed to standstill and wait 200ms (at least 130ms according to datasheet)
        set_velocity(0);

        vTaskDelay(pdMS_TO_TICKS(200));
        read_reg<PWM_AUTO>(rx_data);
        const uint32_t pwm_ofs_auto = PWM_AUTO::pwm_ofs_auto.read_from(rx_data.payload());

        read_reg<DRV_STATUS>(rx_data);
        ESP_LOGI(__FILE__,
                 "AT#1 status: STEALTH=%u CS_ACTUAL=%u PWM_OFS_AUTO=%u",
                 DRV_STATUS::stealth.read_from(rx_data.payload()),
                 DRV_STATUS::cs_actual.read_from(rx_data.payload()),
                 pwm_ofs_auto);

        // 200 rpm * 200steps/rot * 256microsteps/step / 60sec/min
        constexpr uint32_t microsteps_per_sec = 120 * 200 * 256 / 60;
        set_velocity(microsteps_per_sec);

        // Datasheet suggests up to 400 fullsteps; use a larger margin and keep velocity phase long enough.
        constexpr uint64_t tune_fullsteps = 3000;
        constexpr uint64_t tune_time_ms = (tune_fullsteps * 256ULL * 1000ULL + microsteps_per_sec - 1ULL) / microsteps_per_sec;
        const TickType_t stop_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(static_cast<uint32_t>(std::max<uint64_t>(tune_time_ms, 3000ULL)));
        TickType_t next_log = 0;
        while (xTaskGetTickCount() < stop_ticks)
        {
            read_reg<PWM_AUTO>(rx_data);
            uint32_t pwm_grad = PWM_AUTO::pwm_grad_auto.read_from(rx_data.payload());

            read_reg<PWM_SCALE>(rx_data);
            uint32_t pwm_scale_auto = PWM_SCALE::pwm_scale_auto.read_from(rx_data.payload());
            uint32_t pwm_scale_sum = PWM_SCALE::pwm_scale_sum.read_from(rx_data.payload());

            if (xTaskGetTickCount() >= next_log)
            {
                read_reg<DRV_STATUS>(rx_data);
                ESP_LOGI(__FILE__,
                         "AT#2 STEALTH=%u CS=%2u PWM_GRAD_AUTO=%3u PWM_SCALE_AUTO=%3u PWM_SCALE_SUM=%4u",
                         DRV_STATUS::stealth.read_from(rx_data.payload()),
                         DRV_STATUS::cs_actual.read_from(rx_data.payload()),
                         pwm_grad,
                         pwm_scale_auto,
                         pwm_scale_sum);
                next_log = xTaskGetTickCount() + pdMS_TO_TICKS(100);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        set_velocity(0);

        read_reg<PWM_AUTO>(rx_data);
        const uint32_t pwm_grad_final = PWM_AUTO::pwm_grad_auto.read_from(rx_data.payload());
        const uint32_t pwm_ofs_final = PWM_AUTO::pwm_ofs_auto.read_from(rx_data.payload());
        ESP_LOGI(__FILE__,
                 "AT done: PWM_GRAD_AUTO %u -> %u, PWM_OFS_AUTO %u",
                 data,
                 pwm_grad_final,
                 pwm_ofs_final);
        return ESP_OK;
    }

    esp_err_t TMC5240::config_spreadcycle()
    {

        update_field<GCONF>(GCONF::en_pwm_mode, 0);
        return set_spreadcycle_config(5, 2, 0, 0);
    }

    esp_err_t TMC5240::set_spreadcycle_config(uint8_t toff, uint8_t tbl, uint8_t hstart, uint8_t hend)
    {
        CHOPCONF data;

        // Datasheet-recommended spreadCycle tuning range:
        // TOFF [2..15], TBL [0..3], HSTRT [0..7], HEND [0..15]
        toff = static_cast<uint8_t>(std::clamp<uint8_t>(toff, 2, 15));
        tbl = static_cast<uint8_t>(std::min<uint8_t>(tbl, 3));
        hstart = static_cast<uint8_t>(std::min<uint8_t>(hstart, 7));
        hend = static_cast<uint8_t>(std::min<uint8_t>(hend, 15));
        ESP_LOGI(kTag,
                 "Applying spreadcycle config on dev %d: TOFF=%u TBL=%u HSTART=%u HEND=%u",
                 dev_id,
                 static_cast<unsigned>(toff),
                 static_cast<unsigned>(tbl),
                 static_cast<unsigned>(hstart),
                 static_cast<unsigned>(hend));

        data.set_field(CHOPCONF::toff, toff)
            .set_field(CHOPCONF::tbl, tbl)
            .set_field(CHOPCONF::hstrt_tfd210, hstart)
            .set_field(CHOPCONF::hend_offset, hend);
        ESP_RETURN_ON_ERROR(write(data), kTag, "Failed writing CHOPCONF");
        return ESP_OK;
    }

    esp_err_t TMC5240::config_motion_ctrl_vel_mode()
    {
        write(AMAX().set_field(AMAX::amax, 10000));
        return ESP_OK;
    }
    enum status_flags
    {
        RESET_FLAG,
        DRIVER_ERROR,
        SG2,
        STANDSTILL,
        VELOCITY_REACHED,
        POSITION_REACHED,
        STATUS_STOP_L,
        STATUS_STOP_R
    };
    esp_err_t
    TMC5240::write(const tmc5240::SpiWriteBuf datagram)
    {
        ReadDatagram rx;
        spi_bus.transfer(dev_id, datagram, rx.readbuf(), 5);
        handle_status(rx.status());
        return ESP_OK;
    }
    void TMC5240::handle_status(uint8_t status)
    {
        constexpr uint8_t kDriverErrorMask = (1u << DRIVER_ERROR);
        constexpr uint8_t kResetMask = (1u << RESET_FLAG);

        const bool status_changed = (status != last_status);
        last_status = status;

        if (status_side_effects_in_progress)
        {
            return;
        }

        status_side_effects_in_progress = true;

        if (status_changed && (status & kDriverErrorMask))
        {
            ESP_LOGE(__FILE__, "SPI Status DRIVER_ERROR");

            ReadDatagram rx;
            WriteDatagram<DRV_STATUS> tx{uint32_t{0}, false};
            const esp_err_t first_ret = spi_bus.transfer(dev_id, tx, rx.readbuf(), 5);
            const esp_err_t second_ret = (first_ret == ESP_OK) ? spi_bus.transfer(dev_id, tx, rx.readbuf(), 5) : first_ret;
            if (second_ret != ESP_OK)
            {
                ESP_LOGE(__FILE__, "Failed reading DRV_STATUS");
            }
            else
            {
                const uint32_t drv_status = rx.payload();
                auto log_drv_status_flag = [&](const char *flag_name, BitField<DRV_STATUS> field)
                {
                    if (field.read_from(drv_status))
                    {
                        ESP_LOGW(__FILE__, "DRV_STATUS %s set", flag_name);
                    }
                };

                log_drv_status_flag("s2vsa", DRV_STATUS::s2vsa);
                log_drv_status_flag("s2vsb", DRV_STATUS::s2vsb);
                log_drv_status_flag("ot", DRV_STATUS::ot);
                log_drv_status_flag("otpw", DRV_STATUS::otpw);
                log_drv_status_flag("s2ga", DRV_STATUS::s2ga);
                log_drv_status_flag("s2gb", DRV_STATUS::s2gb);
                log_drv_status_flag("ola", DRV_STATUS::ola);
                log_drv_status_flag("olb", DRV_STATUS::olb);
                log_drv_status_flag("stst", DRV_STATUS::stst);
                last_status = rx.status();
            }
        }

        if (status & kResetMask)
        {
            ESP_LOGW(__FILE__, "SPI Status RESET_FLAG");

            ReadDatagram rx;
            WriteDatagram<GSTAT> tx{uint32_t{0}};
            tx.set_field(GSTAT::reset, 1);
            const esp_err_t ret = spi_bus.transfer(dev_id, tx, rx.readbuf(), 5);
            if (ret != ESP_OK)
            {
                ESP_LOGE(__FILE__, "Failed clearing GSTAT reset flag");
            }
            else
            {
                last_status = rx.status();
            }
        }

        status_side_effects_in_progress = false;
    }
    template <typename reg>
    esp_err_t TMC5240::read_reg(ReadDatagram &datagram, bool single)
    {
        tmc5240::WriteDatagram<reg> tx{0, false};

        spi_bus.transfer(dev_id, tx, datagram.readbuf(), 5);
        // when trying to read a single register, do another transfer so that the second transfer
        // reads the data at the address provided in the first transfer
        if (single)
            spi_bus.transfer(dev_id, tx, datagram.readbuf(), 5);
        handle_status(datagram.status());
        return ESP_OK;
    }
    template <typename reg>
    esp_err_t TMC5240::update_field(BitField<reg> field, uint32_t value)
    {
        ReadDatagram rx;
        read_reg<reg>(rx);

        WriteDatagram<reg> tx{rx.payload()};
        tx.set_field(field, value);

        write(tx);
        return ESP_OK;
    }
};
