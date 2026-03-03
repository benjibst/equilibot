#pragma once

#include <cstdint>

#include "spi_bus.hpp"
#include "driver/gpio.h"

namespace tmc5240::registers
{
    struct Field
    {
        uint8_t msb;
        uint8_t lsb;
    };

    [[nodiscard]] constexpr uint32_t field_width(const Field field)
    {
        return static_cast<uint32_t>(field.msb - field.lsb + 1U);
    }

    [[nodiscard]] constexpr uint32_t field_value_mask(const Field field)
    {
        const uint32_t width = field_width(field);
        if (width >= 32U)
        {
            return 0xFFFFFFFFu;
        }
        return (1u << width) - 1u;
    }

    [[nodiscard]] constexpr uint32_t field_mask(const Field field)
    {
        return field_value_mask(field) << field.lsb;
    }

    [[nodiscard]] constexpr uint32_t read_field(const uint32_t reg_value, const Field field)
    {
        return (reg_value >> field.lsb) & field_value_mask(field);
    }

    [[nodiscard]] constexpr int32_t read_field_signed(const uint32_t reg_value, const Field field)
    {
        const uint32_t width = field_width(field);
        const uint32_t value = read_field(reg_value, field);
        if (width == 0U || width >= 32U)
        {
            return static_cast<int32_t>(value);
        }
        const uint32_t sign_bit = 1u << (width - 1U);
        if ((value & sign_bit) == 0U)
        {
            return static_cast<int32_t>(value);
        }
        const uint32_t extended = value | ~field_value_mask(field);
        return static_cast<int32_t>(extended);
    }

    [[nodiscard]] constexpr uint32_t write_field(const uint32_t reg_value, const Field field, const uint32_t field_value)
    {
        const uint32_t mask = field_mask(field);
        const uint32_t shifted = (field_value & field_value_mask(field)) << field.lsb;
        return (reg_value & ~mask) | shifted;
    }

    [[nodiscard]] constexpr uint32_t write_field_signed(const uint32_t reg_value, const Field field, const int32_t field_value)
    {
        return write_field(reg_value, field, static_cast<uint32_t>(field_value));
    }

#define TMC5240_REGISTER_FIELD(field_name, msb_value, lsb_value)                              \
    inline constexpr Field field_name{msb_value, lsb_value};                                  \
    [[nodiscard]] inline constexpr uint32_t get_##field_name(const uint32_t reg_value)        \
    {                                                                                         \
        return read_field(reg_value, field_name);                                             \
    }                                                                                         \
    [[nodiscard]] inline constexpr int32_t get_signed_##field_name(const uint32_t reg_value)  \
    {                                                                                         \
        return read_field_signed(reg_value, field_name);                                      \
    }                                                                                         \
    [[nodiscard]] inline constexpr uint32_t set_##field_name(const uint32_t reg_value,        \
                                                             const uint32_t value)            \
    {                                                                                         \
        return write_field(reg_value, field_name, value);                                     \
    }                                                                                         \
    [[nodiscard]] inline constexpr uint32_t set_signed_##field_name(const uint32_t reg_value, \
                                                                    const int32_t value)      \
    {                                                                                         \
        return write_field_signed(reg_value, field_name, value);                              \
    }

    namespace GCONF
    {
        inline constexpr uint8_t address = 0x00;

        TMC5240_REGISTER_FIELD(length_step_pulse, 20, 17);

        TMC5240_REGISTER_FIELD(direct_mode, 16, 16);

        TMC5240_REGISTER_FIELD(stop_enable, 15, 15);

        TMC5240_REGISTER_FIELD(small_hysteresis, 14, 14);

        TMC5240_REGISTER_FIELD(diag1_poscomp_pushpull, 13, 13);

        TMC5240_REGISTER_FIELD(diag0_int_pushpull, 12, 12);

        TMC5240_REGISTER_FIELD(diag1_nposcomp_dir, 8, 8);

        TMC5240_REGISTER_FIELD(diag0_nint_step, 7, 7);

        TMC5240_REGISTER_FIELD(shaft, 4, 4);

        TMC5240_REGISTER_FIELD(en_pwm_mode, 2, 2);

        TMC5240_REGISTER_FIELD(fast_standstill, 1, 1);

    } // namespace GCONF

    namespace GSTAT
    {
        inline constexpr uint8_t address = 0x01;

        TMC5240_REGISTER_FIELD(vm_uvlo, 4, 4);

        TMC5240_REGISTER_FIELD(register_reset, 3, 3);

        TMC5240_REGISTER_FIELD(uv_cp, 2, 2);

        TMC5240_REGISTER_FIELD(drv_err, 1, 1);

        TMC5240_REGISTER_FIELD(reset, 0, 0);

    } // namespace GSTAT

    namespace IFCNT
    {
        inline constexpr uint8_t address = 0x02;

        TMC5240_REGISTER_FIELD(ifcnt, 7, 0);

    } // namespace IFCNT

    namespace NODECONF
    {
        inline constexpr uint8_t address = 0x03;

        TMC5240_REGISTER_FIELD(senddelay, 11, 8);

        TMC5240_REGISTER_FIELD(nodeaddr, 7, 0);

    } // namespace NODECONF

    namespace IOIN
    {
        inline constexpr uint8_t address = 0x04;

        TMC5240_REGISTER_FIELD(version, 31, 24);

        TMC5240_REGISTER_FIELD(silicon_rv, 18, 16);

        TMC5240_REGISTER_FIELD(adc_err, 15, 15);

        TMC5240_REGISTER_FIELD(ext_clk, 14, 14);

        TMC5240_REGISTER_FIELD(ext_res_det, 13, 13);

        TMC5240_REGISTER_FIELD(output, 12, 12);

        TMC5240_REGISTER_FIELD(comp_b1_b2, 11, 11);

        TMC5240_REGISTER_FIELD(comp_a1_a2, 10, 10);

        TMC5240_REGISTER_FIELD(comp_b, 9, 9);

        TMC5240_REGISTER_FIELD(comp_a, 8, 8);

        TMC5240_REGISTER_FIELD(reserved, 7, 7);

        TMC5240_REGISTER_FIELD(uart_en, 6, 6);

        TMC5240_REGISTER_FIELD(encn, 5, 5);

        TMC5240_REGISTER_FIELD(drv_enn, 4, 4);

        TMC5240_REGISTER_FIELD(enca, 3, 3);

        TMC5240_REGISTER_FIELD(encb, 2, 2);

        TMC5240_REGISTER_FIELD(refr, 1, 1);

        TMC5240_REGISTER_FIELD(refl, 0, 0);

    } // namespace IOIN

    namespace X_COMPARE
    {
        inline constexpr uint8_t address = 0x05;

        TMC5240_REGISTER_FIELD(x_compare, 31, 0);

    } // namespace X_COMPARE

    namespace X_COMPARE_REPEAT
    {
        inline constexpr uint8_t address = 0x06;

        TMC5240_REGISTER_FIELD(x_compare_repeat, 23, 0);

    } // namespace X_COMPARE_REPEAT

    namespace DRV_CONF
    {
        inline constexpr uint8_t address = 0x0A;

        TMC5240_REGISTER_FIELD(slope_control, 5, 4);

        TMC5240_REGISTER_FIELD(current_range, 1, 0);

    } // namespace DRV_CONF

    namespace GLOBAL_SCALER
    {
        inline constexpr uint8_t address = 0x0B;

        TMC5240_REGISTER_FIELD(globalscaler, 7, 0);

    } // namespace GLOBAL_SCALER

    namespace IHOLD_IRUN
    {
        inline constexpr uint8_t address = 0x10;

        TMC5240_REGISTER_FIELD(irundelay, 27, 24);

        TMC5240_REGISTER_FIELD(iholddelay, 19, 16);

        TMC5240_REGISTER_FIELD(irun, 12, 8);

        TMC5240_REGISTER_FIELD(ihold, 4, 0);

    } // namespace IHOLD_IRUN

    namespace TPOWERDOWN
    {
        inline constexpr uint8_t address = 0x11;

        TMC5240_REGISTER_FIELD(tpowerdown, 7, 0);

    } // namespace TPOWERDOWN

    namespace TSTEP
    {
        inline constexpr uint8_t address = 0x12;

        TMC5240_REGISTER_FIELD(tstep, 19, 0);

    } // namespace TSTEP

    namespace TPWMTHRS
    {
        inline constexpr uint8_t address = 0x13;

        TMC5240_REGISTER_FIELD(tpwmthrs, 19, 0);

    } // namespace TPWMTHRS

    namespace TCOOLTHRS
    {
        inline constexpr uint8_t address = 0x14;

        TMC5240_REGISTER_FIELD(tcoolthrs, 19, 0);

    } // namespace TCOOLTHRS

    namespace THIGH
    {
        inline constexpr uint8_t address = 0x15;

        TMC5240_REGISTER_FIELD(thigh, 19, 0);

    } // namespace THIGH

    namespace RAMPMODE
    {
        inline constexpr uint8_t address = 0x20;

        TMC5240_REGISTER_FIELD(rampmode, 1, 0);

    } // namespace RAMPMODE

    namespace XACTUAL
    {
        inline constexpr uint8_t address = 0x21;

        TMC5240_REGISTER_FIELD(xactual, 31, 0);

    } // namespace XACTUAL

    namespace VACTUAL
    {
        inline constexpr uint8_t address = 0x22;

        TMC5240_REGISTER_FIELD(vactual, 23, 0);

    } // namespace VACTUAL

    namespace VSTART
    {
        inline constexpr uint8_t address = 0x23;

        TMC5240_REGISTER_FIELD(vstart, 17, 0);

    } // namespace VSTART

    namespace A1
    {
        inline constexpr uint8_t address = 0x24;

        TMC5240_REGISTER_FIELD(a1, 17, 0);

    } // namespace A1

    namespace V1
    {
        inline constexpr uint8_t address = 0x25;

        TMC5240_REGISTER_FIELD(v1, 19, 0);

    } // namespace V1

    namespace AMAX
    {
        inline constexpr uint8_t address = 0x26;

        TMC5240_REGISTER_FIELD(amax, 17, 0);

    } // namespace AMAX

    namespace VMAX
    {
        inline constexpr uint8_t address = 0x27;

        TMC5240_REGISTER_FIELD(vmax, 22, 0);

    } // namespace VMAX

    namespace DMAX
    {
        inline constexpr uint8_t address = 0x28;

        TMC5240_REGISTER_FIELD(dmax, 17, 0);

    } // namespace DMAX

    namespace TVMAX
    {
        inline constexpr uint8_t address = 0x29;

        TMC5240_REGISTER_FIELD(tvmax, 15, 0);

    } // namespace TVMAX

    namespace D1
    {
        inline constexpr uint8_t address = 0x2A;

        TMC5240_REGISTER_FIELD(d1, 17, 0);

    } // namespace D1

    namespace VSTOP
    {
        inline constexpr uint8_t address = 0x2B;

        TMC5240_REGISTER_FIELD(vstop, 17, 0);

    } // namespace VSTOP

    namespace TZEROWAIT
    {
        inline constexpr uint8_t address = 0x2C;

        TMC5240_REGISTER_FIELD(tzerowait, 15, 0);

    } // namespace TZEROWAIT

    namespace XTARGET
    {
        inline constexpr uint8_t address = 0x2D;

        TMC5240_REGISTER_FIELD(xtarget, 31, 0);

    } // namespace XTARGET

    namespace V2
    {
        inline constexpr uint8_t address = 0x2E;

        TMC5240_REGISTER_FIELD(v2, 19, 0);

    } // namespace V2

    namespace A2
    {
        inline constexpr uint8_t address = 0x2F;

        TMC5240_REGISTER_FIELD(a2, 17, 0);

    } // namespace A2

    namespace D2
    {
        inline constexpr uint8_t address = 0x30;

        TMC5240_REGISTER_FIELD(d2, 17, 0);

    } // namespace D2

    namespace VDCMIN
    {
        inline constexpr uint8_t address = 0x33;

        TMC5240_REGISTER_FIELD(vdcmin, 22, 8);

        TMC5240_REGISTER_FIELD(reserved, 7, 0);

    } // namespace VDCMIN

    namespace SW_MODE
    {
        inline constexpr uint8_t address = 0x34;

        TMC5240_REGISTER_FIELD(virtual_stop_enc, 14, 14);

        TMC5240_REGISTER_FIELD(en_virtual_stop_r, 13, 13);

        TMC5240_REGISTER_FIELD(en_virtual_stop_l, 12, 12);

        TMC5240_REGISTER_FIELD(en_softstop, 11, 11);

        TMC5240_REGISTER_FIELD(sg_stop, 10, 10);

        TMC5240_REGISTER_FIELD(en_latch_encoder, 9, 9);

        TMC5240_REGISTER_FIELD(latch_r_inactive, 8, 8);

        TMC5240_REGISTER_FIELD(latch_r_active, 7, 7);

        TMC5240_REGISTER_FIELD(latch_l_inactive, 6, 6);

        TMC5240_REGISTER_FIELD(latch_l_active, 5, 5);

        TMC5240_REGISTER_FIELD(swap_lr, 4, 4);

        TMC5240_REGISTER_FIELD(pol_stop_r, 3, 3);

        TMC5240_REGISTER_FIELD(pol_stop_l, 2, 2);

        TMC5240_REGISTER_FIELD(stop_r_enable, 1, 1);

        TMC5240_REGISTER_FIELD(stop_l_enable, 0, 0);

    } // namespace SW_MODE

    namespace RAMP_STAT
    {
        inline constexpr uint8_t address = 0x35;

        TMC5240_REGISTER_FIELD(status_virtual_stop_r, 15, 15);

        TMC5240_REGISTER_FIELD(status_virtual_stop_l, 14, 14);

        TMC5240_REGISTER_FIELD(status_sg, 13, 13);

        TMC5240_REGISTER_FIELD(second_move, 12, 12);

        TMC5240_REGISTER_FIELD(t_zerowait_active, 11, 11);

        TMC5240_REGISTER_FIELD(vzero, 10, 10);

        TMC5240_REGISTER_FIELD(position_reached, 9, 9);

        TMC5240_REGISTER_FIELD(velocity_reached, 8, 8);

        TMC5240_REGISTER_FIELD(event_pos_reached, 7, 7);

        TMC5240_REGISTER_FIELD(event_stop_sg, 6, 6);

        TMC5240_REGISTER_FIELD(event_stop_r, 5, 5);

        TMC5240_REGISTER_FIELD(event_stop_l, 4, 4);

        TMC5240_REGISTER_FIELD(status_latch_r, 3, 3);

        TMC5240_REGISTER_FIELD(status_latch_l, 2, 2);

        TMC5240_REGISTER_FIELD(status_stop_r, 1, 1);

        TMC5240_REGISTER_FIELD(status_stop_l, 0, 0);

    } // namespace RAMP_STAT

    namespace XLATCH
    {
        inline constexpr uint8_t address = 0x36;

        TMC5240_REGISTER_FIELD(xlatch, 31, 0);

    } // namespace XLATCH

    namespace ENCMODE
    {
        inline constexpr uint8_t address = 0x38;

        TMC5240_REGISTER_FIELD(enc_sel_decimal, 10, 10);

        TMC5240_REGISTER_FIELD(latch_x_act, 9, 9);

        TMC5240_REGISTER_FIELD(clr_enc_x, 8, 8);

        TMC5240_REGISTER_FIELD(pos_neg_edge, 7, 6);

        TMC5240_REGISTER_FIELD(clr_once, 5, 5);

        TMC5240_REGISTER_FIELD(clr_cont, 4, 4);

        TMC5240_REGISTER_FIELD(ignore_ab, 3, 3);

        TMC5240_REGISTER_FIELD(pol_n, 2, 2);

        TMC5240_REGISTER_FIELD(pol_b, 1, 1);

        TMC5240_REGISTER_FIELD(pol_a, 0, 0);

    } // namespace ENCMODE

    namespace X_ENC
    {
        inline constexpr uint8_t address = 0x39;

        TMC5240_REGISTER_FIELD(x_enc, 31, 0);

    } // namespace X_ENC

    namespace ENC_CONST
    {
        inline constexpr uint8_t address = 0x3A;

        TMC5240_REGISTER_FIELD(enc_const, 31, 0);

    } // namespace ENC_CONST

    namespace ENC_STATUS
    {
        inline constexpr uint8_t address = 0x3B;

        TMC5240_REGISTER_FIELD(deviation_warn, 1, 1);

        TMC5240_REGISTER_FIELD(n_event, 0, 0);

    } // namespace ENC_STATUS

    namespace ENC_LATCH
    {
        inline constexpr uint8_t address = 0x3C;

        TMC5240_REGISTER_FIELD(enc_latch, 31, 0);

    } // namespace ENC_LATCH

    namespace ENC_DEVIATION
    {
        inline constexpr uint8_t address = 0x3D;

        TMC5240_REGISTER_FIELD(enc_deviation, 19, 0);

    } // namespace ENC_DEVIATION

    namespace VIRTUAL_STOP_L
    {
        inline constexpr uint8_t address = 0x3E;

        TMC5240_REGISTER_FIELD(virtual_stop_l, 31, 0);

    } // namespace VIRTUAL_STOP_L

    namespace VIRTUAL_STOP_R
    {
        inline constexpr uint8_t address = 0x3F;

        TMC5240_REGISTER_FIELD(virtual_stop_r, 31, 0);

    } // namespace VIRTUAL_STOP_R

    namespace ADC_VSUPPLY_AIN
    {
        inline constexpr uint8_t address = 0x50;

        TMC5240_REGISTER_FIELD(adc_ain, 28, 16);

        TMC5240_REGISTER_FIELD(adc_vsupply, 12, 0);

    } // namespace ADC_VSUPPLY_AIN

    namespace ADC_TEMP
    {
        inline constexpr uint8_t address = 0x51;

        TMC5240_REGISTER_FIELD(reserved, 28, 16);

        TMC5240_REGISTER_FIELD(adc_temp, 12, 0);

    } // namespace ADC_TEMP

    namespace OTW_OV_VTH
    {
        inline constexpr uint8_t address = 0x52;

        TMC5240_REGISTER_FIELD(overtempprewarning_vth, 28, 16);

        TMC5240_REGISTER_FIELD(overvoltage_vth, 12, 0);

    } // namespace OTW_OV_VTH

    namespace MSLUT_0
    {
        inline constexpr uint8_t address = 0x60;

        TMC5240_REGISTER_FIELD(mslut_0, 31, 0);

    } // namespace MSLUT_0

    namespace MSLUT_1
    {
        inline constexpr uint8_t address = 0x61;

        TMC5240_REGISTER_FIELD(mslut_1, 31, 0);

    } // namespace MSLUT_1

    namespace MSLUT_2
    {
        inline constexpr uint8_t address = 0x62;

        TMC5240_REGISTER_FIELD(mslut_2, 31, 0);

    } // namespace MSLUT_2

    namespace MSLUT_3
    {
        inline constexpr uint8_t address = 0x63;

        TMC5240_REGISTER_FIELD(mslut_3, 31, 0);

    } // namespace MSLUT_3

    namespace MSLUT_4
    {
        inline constexpr uint8_t address = 0x64;

        TMC5240_REGISTER_FIELD(mslut_4, 31, 0);

    } // namespace MSLUT_4

    namespace MSLUT_5
    {
        inline constexpr uint8_t address = 0x65;

        TMC5240_REGISTER_FIELD(mslut_5, 31, 0);

    } // namespace MSLUT_5

    namespace MSLUT_6
    {
        inline constexpr uint8_t address = 0x66;

        TMC5240_REGISTER_FIELD(mslut_6, 31, 0);

    } // namespace MSLUT_6

    namespace MSLUT_7
    {
        inline constexpr uint8_t address = 0x67;

        TMC5240_REGISTER_FIELD(mslut_7, 31, 0);

    } // namespace MSLUT_7

    namespace MSLUTSEL
    {
        inline constexpr uint8_t address = 0x68;

        TMC5240_REGISTER_FIELD(x3, 31, 24);

        TMC5240_REGISTER_FIELD(x2, 23, 16);

        TMC5240_REGISTER_FIELD(x1, 15, 8);

        TMC5240_REGISTER_FIELD(w3, 7, 6);

        TMC5240_REGISTER_FIELD(w2, 5, 4);

        TMC5240_REGISTER_FIELD(w1, 3, 2);

        TMC5240_REGISTER_FIELD(w0, 1, 0);

    } // namespace MSLUTSEL

    namespace MSLUTSTART
    {
        inline constexpr uint8_t address = 0x69;

        TMC5240_REGISTER_FIELD(offset_sin90, 31, 24);

        TMC5240_REGISTER_FIELD(start_sin90, 23, 16);

        TMC5240_REGISTER_FIELD(start_sin, 7, 0);

    } // namespace MSLUTSTART

    namespace MSCNT
    {
        inline constexpr uint8_t address = 0x6A;

        TMC5240_REGISTER_FIELD(mscnt, 9, 0);

    } // namespace MSCNT

    namespace MSCURACT
    {
        inline constexpr uint8_t address = 0x6B;

        TMC5240_REGISTER_FIELD(cur_a, 24, 16);

        TMC5240_REGISTER_FIELD(cur_b, 8, 0);

    } // namespace MSCURACT

    namespace CHOPCONF
    {
        inline constexpr uint8_t address = 0x6C;

        TMC5240_REGISTER_FIELD(diss2vs, 31, 31);

        TMC5240_REGISTER_FIELD(diss2g, 30, 30);

        TMC5240_REGISTER_FIELD(reserved, 29, 29);

        TMC5240_REGISTER_FIELD(intpol, 28, 28);

        TMC5240_REGISTER_FIELD(mres, 27, 24);

        TMC5240_REGISTER_FIELD(tpfd, 23, 20);

        TMC5240_REGISTER_FIELD(vhighchm, 19, 19);

        TMC5240_REGISTER_FIELD(vhighfs, 18, 18);

        TMC5240_REGISTER_FIELD(tbl, 16, 15);

        TMC5240_REGISTER_FIELD(chm, 14, 14);

        TMC5240_REGISTER_FIELD(disfdcc, 12, 12);

        TMC5240_REGISTER_FIELD(fd3, 11, 11);

        TMC5240_REGISTER_FIELD(hend_offset, 10, 7);

        TMC5240_REGISTER_FIELD(hstrt_tfd210, 6, 4);

        TMC5240_REGISTER_FIELD(toff, 3, 0);

    } // namespace CHOPCONF

    namespace COOLCONF
    {
        inline constexpr uint8_t address = 0x6D;

        TMC5240_REGISTER_FIELD(sfilt, 24, 24);

        TMC5240_REGISTER_FIELD(sgt, 22, 16);

        TMC5240_REGISTER_FIELD(seimin, 15, 15);

        TMC5240_REGISTER_FIELD(sedn, 14, 13);

        TMC5240_REGISTER_FIELD(semax, 11, 8);

        TMC5240_REGISTER_FIELD(seup, 6, 5);

        TMC5240_REGISTER_FIELD(semin, 3, 0);

    } // namespace COOLCONF

    namespace DCCTRL
    {
        inline constexpr uint8_t address = 0x6E;

        TMC5240_REGISTER_FIELD(dc_sg, 23, 16);

        TMC5240_REGISTER_FIELD(dc_time, 9, 0);

    } // namespace DCCTRL

    namespace DRV_STATUS
    {
        inline constexpr uint8_t address = 0x6F;

        TMC5240_REGISTER_FIELD(stst, 31, 31);

        TMC5240_REGISTER_FIELD(olb, 30, 30);

        TMC5240_REGISTER_FIELD(ola, 29, 29);

        TMC5240_REGISTER_FIELD(s2gb, 28, 28);

        TMC5240_REGISTER_FIELD(s2ga, 27, 27);

        TMC5240_REGISTER_FIELD(otpw, 26, 26);

        TMC5240_REGISTER_FIELD(ot, 25, 25);

        TMC5240_REGISTER_FIELD(stallguard, 24, 24);

        TMC5240_REGISTER_FIELD(cs_actual, 20, 16);

        TMC5240_REGISTER_FIELD(fsactive, 15, 15);

        TMC5240_REGISTER_FIELD(stealth, 14, 14);

        TMC5240_REGISTER_FIELD(s2vsb, 13, 13);

        TMC5240_REGISTER_FIELD(s2vsa, 12, 12);

        TMC5240_REGISTER_FIELD(sg_result, 9, 0);

    } // namespace DRV_STATUS

    namespace PWMCONF
    {
        inline constexpr uint8_t address = 0x70;

        TMC5240_REGISTER_FIELD(pwm_lim, 31, 28);

        TMC5240_REGISTER_FIELD(pwm_reg, 27, 24);

        TMC5240_REGISTER_FIELD(pwm_dis_reg_stst, 23, 23);

        TMC5240_REGISTER_FIELD(pwm_meas_sd_enable, 22, 22);

        TMC5240_REGISTER_FIELD(freewheel, 21, 20);

        TMC5240_REGISTER_FIELD(pwm_autograd, 19, 19);

        TMC5240_REGISTER_FIELD(pwm_autoscale, 18, 18);

        TMC5240_REGISTER_FIELD(pwm_freq, 17, 16);

        TMC5240_REGISTER_FIELD(pwm_grad, 15, 8);

        TMC5240_REGISTER_FIELD(pwm_ofs, 7, 0);

    } // namespace PWMCONF

    namespace PWM_SCALE
    {
        inline constexpr uint8_t address = 0x71;

        TMC5240_REGISTER_FIELD(pwm_scale_auto, 24, 16);

        TMC5240_REGISTER_FIELD(pwm_scale_sum, 9, 0);

    } // namespace PWM_SCALE

    namespace PWM_AUTO
    {
        inline constexpr uint8_t address = 0x72;

        TMC5240_REGISTER_FIELD(pwm_grad_auto, 23, 16);

        TMC5240_REGISTER_FIELD(pwm_ofs_auto, 7, 0);

    } // namespace PWM_AUTO

    namespace SG4_THRS
    {
        inline constexpr uint8_t address = 0x74;

        TMC5240_REGISTER_FIELD(sg_angle_offset, 9, 9);

        TMC5240_REGISTER_FIELD(sg4_filt_en, 8, 8);

        TMC5240_REGISTER_FIELD(sg4_thrs, 7, 0);

    } // namespace SG4_THRS

    namespace SG4_RESULT
    {
        inline constexpr uint8_t address = 0x75;

        TMC5240_REGISTER_FIELD(sg4_result, 9, 0);

    } // namespace SG4_RESULT

    namespace SG4_IND
    {
        inline constexpr uint8_t address = 0x76;

        TMC5240_REGISTER_FIELD(sg4_ind_3, 31, 24);

        TMC5240_REGISTER_FIELD(sg4_ind_2, 23, 16);

        TMC5240_REGISTER_FIELD(sg4_ind_1, 15, 8);

        TMC5240_REGISTER_FIELD(sg4_ind_0, 7, 0);

    } // namespace SG4_IND

#undef TMC5240_REGISTER_FIELD

} // namespace tmc5240::registers

class TMC5240
{
public:
    TMC5240(SpiBus &spi_bus, gpio_num_t en_pin, gpio_num_t cs_pin, int device_id, float rref_kohm, float motor_current_rms);
    int device_id() { return this->dev_id; };
    esp_err_t set_velocity(int32_t vel);
    esp_err_t set_spreadcycle_config(uint8_t toff, uint8_t tbl, uint8_t hstart, uint8_t hend);

private:
    esp_err_t config_current();
    esp_err_t config_stealthchop2();
    esp_err_t config_spreadcycle();

    esp_err_t config_motion_ctrl_vel_mode();
    esp_err_t write(uint8_t addr, uint32_t data, uint8_t &spi_status);
    esp_err_t read(uint8_t addr, uint32_t &val, uint8_t &spi_status);
    SpiBus &spi_bus;
    const int dev_id;
    const float rref_kohm;
    const float motor_current_rms;
};
