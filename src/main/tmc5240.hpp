#pragma once

#include <cstdint>
#include "util.hpp"

#include "spi_bus.hpp"
#include "driver/gpio.h"

namespace tmc5240
{
    template <typename Register>
    class BitField
    {
    public:
        uint8_t high_bit, low_bit;
        constexpr BitField(uint8_t bit) : high_bit(bit), low_bit(bit) {}
        constexpr BitField(uint8_t high_bit, uint8_t low_bit) : high_bit(high_bit), low_bit(low_bit) {}
        constexpr uint32_t mask() const
        {
            const uint32_t high_mask = high_bit >= 31 ? 0xFFFFFFFFu : ((1u << (high_bit + 1)) - 1u);
            return high_mask & ~((1u << low_bit) - 1u);
        }
        constexpr uint32_t read_from(uint32_t data) const
        {
            return (data & mask()) >> low_bit;
        }
    };

    using SpiWriteBuf = uint8_t *;
    using SpiReadBuf = uint8_t *;
    constexpr uint8_t addr_set_rw_bit(bool write, uint8_t addr)
    {
        if (write)
            return addr | 0x80;
        return addr;
    }
    template <typename Register>
    struct WriteDatagram
    {
    public:
        uint8_t buf[8] = {0};
        uint32_t &payload()
        {
            return *reinterpret_cast<uint32_t *>(buf + 4);
        }
        uint8_t &addr()
        {
            return buf[3];
        }
        WriteDatagram<Register> &set_field(BitField<Register> field, uint32_t value)
        {
            uint32_t mask = field.mask();
            payload() &= ~mask;
            payload() |= value << field.low_bit & mask;
            return *this;
        }
        operator SpiWriteBuf()
        {
            if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
                payload() = __builtin_bswap32(payload());
            return buf + 3;
        }
        WriteDatagram(uint32_t init_payload, bool for_write = true) : WriteDatagram(for_write)
        {
            payload() = init_payload;
        }
        WriteDatagram(bool for_write = true)
        {
            addr() = addr_set_rw_bit(for_write, Register::addr);
        }
    };

    class GCONF : public WriteDatagram<GCONF>
    {
    public:
        static constexpr uint8_t addr{0x00};
        static constexpr BitField<GCONF> length_step_pulse{20, 17};
        static constexpr BitField<GCONF> direct_mode{16};
        static constexpr BitField<GCONF> stop_enable{15};
        static constexpr BitField<GCONF> small_hysteresis{14};
        static constexpr BitField<GCONF> diag1_poscomp_pushpull{13};
        static constexpr BitField<GCONF> diag1_int_pushpull{12};
        static constexpr BitField<GCONF> diag1_nposcomp_dir{8};
        static constexpr BitField<GCONF> diag0_nint_step{7};
        static constexpr BitField<GCONF> shaft{4};
        static constexpr BitField<GCONF> en_pwm_mode{2};
        static constexpr BitField<GCONF> fast_standstill{1};
    };

    class GSTAT : public WriteDatagram<GSTAT>
    {
    public:
        static constexpr uint8_t addr{0x01};
        static constexpr BitField<GSTAT> vm_uvlo{4};
        static constexpr BitField<GSTAT> register_reset{3};
        static constexpr BitField<GSTAT> uv_cp{2};
        static constexpr BitField<GSTAT> drv_err{1};
        static constexpr BitField<GSTAT> reset{0};
    };

    class IFCNT : public WriteDatagram<IFCNT>
    {
    public:
        static constexpr uint8_t addr{0x02};
        static constexpr BitField<IFCNT> ifcnt{7, 0};
    };

    class NODECONF : public WriteDatagram<NODECONF>
    {
    public:
        static constexpr uint8_t addr{0x03};
        static constexpr BitField<NODECONF> senddelay{11, 8};
        static constexpr BitField<NODECONF> nodeaddr{7, 0};
    };

    class IOIN : public WriteDatagram<IOIN>
    {
    public:
        static constexpr uint8_t addr{0x04};
        static constexpr BitField<IOIN> version{31, 24};
        static constexpr BitField<IOIN> silicon_rv{18, 16};
        static constexpr BitField<IOIN> adc_err{15};
        static constexpr BitField<IOIN> ext_clk{14};
        static constexpr BitField<IOIN> ext_res_det{13};
        static constexpr BitField<IOIN> output{12};
        static constexpr BitField<IOIN> comp_b1_b2{11};
        static constexpr BitField<IOIN> comp_a1_a2{10};
        static constexpr BitField<IOIN> comp_b{9};
        static constexpr BitField<IOIN> comp_a{8};
        static constexpr BitField<IOIN> reserved{7};
        static constexpr BitField<IOIN> uart_en{6};
        static constexpr BitField<IOIN> encn{5};
        static constexpr BitField<IOIN> drv_enn{4};
        static constexpr BitField<IOIN> enca{3};
        static constexpr BitField<IOIN> encb{2};
        static constexpr BitField<IOIN> refr{1};
        static constexpr BitField<IOIN> refl{0};
    };

    class X_COMPARE : public WriteDatagram<X_COMPARE>
    {
    public:
        static constexpr uint8_t addr{0x05};
        static constexpr BitField<X_COMPARE> x_compare{31, 0};
    };

    class X_COMPARE_REPEAT : public WriteDatagram<X_COMPARE_REPEAT>
    {
    public:
        static constexpr uint8_t addr{0x06};
        static constexpr BitField<X_COMPARE_REPEAT> x_compare_repeat{23, 0};
    };

    class DRV_CONF : public WriteDatagram<DRV_CONF>
    {
    public:
        static constexpr uint8_t addr{0x0A};
        static constexpr BitField<DRV_CONF> slope_control{5, 4};
        static constexpr BitField<DRV_CONF> current_range{1, 0};
    };

    class GLOBAL_SCALER : public WriteDatagram<GLOBAL_SCALER>
    {
    public:
        static constexpr uint8_t addr{0x0B};
        static constexpr BitField<GLOBAL_SCALER> globalscaler{7, 0};
    };

    class IHOLD_IRUN : public WriteDatagram<IHOLD_IRUN>
    {
    public:
        static constexpr uint8_t addr{0x10};
        static constexpr BitField<IHOLD_IRUN> irundelay{27, 24};
        static constexpr BitField<IHOLD_IRUN> iholddelay{19, 16};
        static constexpr BitField<IHOLD_IRUN> irun{12, 8};
        static constexpr BitField<IHOLD_IRUN> ihold{4, 0};
    };

    class TPOWERDOWN : public WriteDatagram<TPOWERDOWN>
    {
    public:
        static constexpr uint8_t addr{0x11};
        static constexpr BitField<TPOWERDOWN> tpowerdown{7, 0};
    };

    class TSTEP : public WriteDatagram<TSTEP>
    {
    public:
        static constexpr uint8_t addr{0x12};
        static constexpr BitField<TSTEP> tstep{19, 0};
    };

    class TPWMTHRS : public WriteDatagram<TPWMTHRS>
    {
    public:
        static constexpr uint8_t addr{0x13};
        static constexpr BitField<TPWMTHRS> tpwmthrs{19, 0};
    };

    class TCOOLTHRS : public WriteDatagram<TCOOLTHRS>
    {
    public:
        static constexpr uint8_t addr{0x14};
        static constexpr BitField<TCOOLTHRS> tcoolthrs{19, 0};
    };

    class THIGH : public WriteDatagram<THIGH>
    {
    public:
        static constexpr uint8_t addr{0x15};
        static constexpr BitField<THIGH> thigh{19, 0};
    };

    class RAMPMODE : public WriteDatagram<RAMPMODE>
    {
    public:
        static constexpr uint8_t addr{0x20};
        static constexpr BitField<RAMPMODE> rampmode{1, 0};
    };

    class XACTUAL : public WriteDatagram<XACTUAL>
    {
    public:
        static constexpr uint8_t addr{0x21};
        static constexpr BitField<XACTUAL> xactual{31, 0};
    };

    class VACTUAL : public WriteDatagram<VACTUAL>
    {
    public:
        static constexpr uint8_t addr{0x22};
        static constexpr BitField<VACTUAL> vactual{23, 0};
    };

    class VSTART : public WriteDatagram<VSTART>
    {
    public:
        static constexpr uint8_t addr{0x23};
        static constexpr BitField<VSTART> vstart{17, 0};
    };

    class A1 : public WriteDatagram<A1>
    {
    public:
        static constexpr uint8_t addr{0x24};
        static constexpr BitField<A1> a1{17, 0};
    };

    class V1 : public WriteDatagram<V1>
    {
    public:
        static constexpr uint8_t addr{0x25};
        static constexpr BitField<V1> v1{19, 0};
    };

    class AMAX : public WriteDatagram<AMAX>
    {
    public:
        static constexpr uint8_t addr{0x26};
        static constexpr BitField<AMAX> amax{17, 0};
    };

    class VMAX : public WriteDatagram<VMAX>
    {
    public:
        static constexpr uint8_t addr{0x27};
        static constexpr BitField<VMAX> vmax{22, 0};
    };

    class DMAX : public WriteDatagram<DMAX>
    {
    public:
        static constexpr uint8_t addr{0x28};
        static constexpr BitField<DMAX> dmax{17, 0};
    };

    class TVMAX : public WriteDatagram<TVMAX>
    {
    public:
        static constexpr uint8_t addr{0x29};
        static constexpr BitField<TVMAX> tvmax{15, 0};
    };

    class D1 : public WriteDatagram<D1>
    {
    public:
        static constexpr uint8_t addr{0x2A};
        static constexpr BitField<D1> d1{17, 0};
    };

    class VSTOP : public WriteDatagram<VSTOP>
    {
    public:
        static constexpr uint8_t addr{0x2B};
        static constexpr BitField<VSTOP> vstop{17, 0};
    };

    class TZEROWAIT : public WriteDatagram<TZEROWAIT>
    {
    public:
        static constexpr uint8_t addr{0x2C};
        static constexpr BitField<TZEROWAIT> tzerowait{15, 0};
    };

    class XTARGET : public WriteDatagram<XTARGET>
    {
    public:
        static constexpr uint8_t addr{0x2D};
        static constexpr BitField<XTARGET> xtarget{31, 0};
    };

    class V2 : public WriteDatagram<V2>
    {
    public:
        static constexpr uint8_t addr{0x2E};
        static constexpr BitField<V2> v2{19, 0};
    };

    class A2 : public WriteDatagram<A2>
    {
    public:
        static constexpr uint8_t addr{0x2F};
        static constexpr BitField<A2> a2{17, 0};
    };

    class D2 : public WriteDatagram<D2>
    {
    public:
        static constexpr uint8_t addr{0x30};
        static constexpr BitField<D2> d2{17, 0};
    };

    class VDCMIN : public WriteDatagram<VDCMIN>
    {
    public:
        static constexpr uint8_t addr{0x33};
        static constexpr BitField<VDCMIN> vdcmin{22, 8};
        static constexpr BitField<VDCMIN> reserved{7, 0};
    };

    class SW_MODE : public WriteDatagram<SW_MODE>
    {
    public:
        static constexpr uint8_t addr{0x34};
        static constexpr BitField<SW_MODE> virtual_stop_enc{14};
        static constexpr BitField<SW_MODE> en_virtual_stop_r{13};
        static constexpr BitField<SW_MODE> en_virtual_stop_l{12};
        static constexpr BitField<SW_MODE> en_softstop{11};
        static constexpr BitField<SW_MODE> sg_stop{10};
        static constexpr BitField<SW_MODE> en_latch_encoder{9};
        static constexpr BitField<SW_MODE> latch_r_inactive{8};
        static constexpr BitField<SW_MODE> latch_r_active{7};
        static constexpr BitField<SW_MODE> latch_l_inactive{6};
        static constexpr BitField<SW_MODE> latch_l_active{5};
        static constexpr BitField<SW_MODE> swap_lr{4};
        static constexpr BitField<SW_MODE> pol_stop_r{3};
        static constexpr BitField<SW_MODE> pol_stop_l{2};
        static constexpr BitField<SW_MODE> stop_r_enable{1};
        static constexpr BitField<SW_MODE> stop_l_enable{0};
    };

    class RAMP_STAT : public WriteDatagram<RAMP_STAT>
    {
    public:
        static constexpr uint8_t addr{0x35};
        static constexpr BitField<RAMP_STAT> status_virtual_stop_r{15};
        static constexpr BitField<RAMP_STAT> status_virtual_stop_l{14};
        static constexpr BitField<RAMP_STAT> status_sg{13};
        static constexpr BitField<RAMP_STAT> second_move{12};
        static constexpr BitField<RAMP_STAT> t_zerowait_active{11};
        static constexpr BitField<RAMP_STAT> vzero{10};
        static constexpr BitField<RAMP_STAT> position_reached{9};
        static constexpr BitField<RAMP_STAT> velocity_reached{8};
        static constexpr BitField<RAMP_STAT> event_pos_reached{7};
        static constexpr BitField<RAMP_STAT> event_stop_sg{6};
        static constexpr BitField<RAMP_STAT> event_stop_r{5};
        static constexpr BitField<RAMP_STAT> event_stop_l{4};
        static constexpr BitField<RAMP_STAT> status_latch_r{3};
        static constexpr BitField<RAMP_STAT> status_latch_l{2};
        static constexpr BitField<RAMP_STAT> status_stop_r{1};
        static constexpr BitField<RAMP_STAT> status_stop_l{0};
    };

    class XLATCH : public WriteDatagram<XLATCH>
    {
    public:
        static constexpr uint8_t addr{0x36};
        static constexpr BitField<XLATCH> xlatch{31, 0};
    };

    class ENCMODE : public WriteDatagram<ENCMODE>
    {
    public:
        static constexpr uint8_t addr{0x38};
        static constexpr BitField<ENCMODE> enc_sel_decimal{10};
        static constexpr BitField<ENCMODE> latch_x_act{9};
        static constexpr BitField<ENCMODE> clr_enc_x{8};
        static constexpr BitField<ENCMODE> pos_neg_edge{7, 6};
        static constexpr BitField<ENCMODE> clr_once{5};
        static constexpr BitField<ENCMODE> clr_cont{4};
        static constexpr BitField<ENCMODE> ignore_ab{3};
        static constexpr BitField<ENCMODE> pol_n{2};
        static constexpr BitField<ENCMODE> pol_b{1};
        static constexpr BitField<ENCMODE> pol_a{0};
    };

    class X_ENC : public WriteDatagram<X_ENC>
    {
    public:
        static constexpr uint8_t addr{0x39};
        static constexpr BitField<X_ENC> x_enc{31, 0};
    };

    class ENC_CONST : public WriteDatagram<ENC_CONST>
    {
    public:
        static constexpr uint8_t addr{0x3A};
        static constexpr BitField<ENC_CONST> enc_const{31, 0};
    };

    class ENC_STATUS : public WriteDatagram<ENC_STATUS>
    {
    public:
        static constexpr uint8_t addr{0x3B};
        static constexpr BitField<ENC_STATUS> deviation_warn{1};
        static constexpr BitField<ENC_STATUS> n_event{0};
    };

    class ENC_LATCH : public WriteDatagram<ENC_LATCH>
    {
    public:
        static constexpr uint8_t addr{0x3C};
        static constexpr BitField<ENC_LATCH> enc_latch{31, 0};
    };

    class ENC_DEVIATION : public WriteDatagram<ENC_DEVIATION>
    {
    public:
        static constexpr uint8_t addr{0x3D};
        static constexpr BitField<ENC_DEVIATION> enc_deviation{19, 0};
    };

    class VIRTUAL_STOP_L : public WriteDatagram<VIRTUAL_STOP_L>
    {
    public:
        static constexpr uint8_t addr{0x3E};
        static constexpr BitField<VIRTUAL_STOP_L> virtual_stop_l{31, 0};
    };

    class VIRTUAL_STOP_R : public WriteDatagram<VIRTUAL_STOP_R>
    {
    public:
        static constexpr uint8_t addr{0x3F};
        static constexpr BitField<VIRTUAL_STOP_R> virtual_stop_r{31, 0};
    };

    class ADC_VSUPPLY_AIN : public WriteDatagram<ADC_VSUPPLY_AIN>
    {
    public:
        static constexpr uint8_t addr{0x50};
        static constexpr BitField<ADC_VSUPPLY_AIN> adc_ain{28, 16};
        static constexpr BitField<ADC_VSUPPLY_AIN> adc_vsupply{12, 0};
    };

    class ADC_TEMP : public WriteDatagram<ADC_TEMP>
    {
    public:
        static constexpr uint8_t addr{0x51};
        static constexpr BitField<ADC_TEMP> reserved{28, 16};
        static constexpr BitField<ADC_TEMP> adc_temp{12, 0};
    };

    class OTW_OV_VTH : public WriteDatagram<OTW_OV_VTH>
    {
    public:
        static constexpr uint8_t addr{0x52};
        static constexpr BitField<OTW_OV_VTH> overtempprewarning_vth{28, 16};
        static constexpr BitField<OTW_OV_VTH> overvoltage_vth{12, 0};
    };

    class MSLUT_0 : public WriteDatagram<MSLUT_0>
    {
    public:
        static constexpr uint8_t addr{0x60};
        static constexpr BitField<MSLUT_0> mslut_0{31, 0};
    };

    class MSLUT_1 : public WriteDatagram<MSLUT_1>
    {
    public:
        static constexpr uint8_t addr{0x61};
        static constexpr BitField<MSLUT_1> mslut_1{31, 0};
    };

    class MSLUT_2 : public WriteDatagram<MSLUT_2>
    {
    public:
        static constexpr uint8_t addr{0x62};
        static constexpr BitField<MSLUT_2> mslut_2{31, 0};
    };

    class MSLUT_3 : public WriteDatagram<MSLUT_3>
    {
    public:
        static constexpr uint8_t addr{0x63};
        static constexpr BitField<MSLUT_3> mslut_3{31, 0};
    };

    class MSLUT_4 : public WriteDatagram<MSLUT_4>
    {
    public:
        static constexpr uint8_t addr{0x64};
        static constexpr BitField<MSLUT_4> mslut_4{31, 0};
    };

    class MSLUT_5 : public WriteDatagram<MSLUT_5>
    {
    public:
        static constexpr uint8_t addr{0x65};
        static constexpr BitField<MSLUT_5> mslut_5{31, 0};
    };

    class MSLUT_6 : public WriteDatagram<MSLUT_6>
    {
    public:
        static constexpr uint8_t addr{0x66};
        static constexpr BitField<MSLUT_6> mslut_6{31, 0};
    };

    class MSLUT_7 : public WriteDatagram<MSLUT_7>
    {
    public:
        static constexpr uint8_t addr{0x67};
        static constexpr BitField<MSLUT_7> mslut_7{31, 0};
    };

    class MSLUTSEL : public WriteDatagram<MSLUTSEL>
    {
    public:
        static constexpr uint8_t addr{0x68};
        static constexpr BitField<MSLUTSEL> x3{31, 24};
        static constexpr BitField<MSLUTSEL> x2{23, 16};
        static constexpr BitField<MSLUTSEL> x1{15, 8};
        static constexpr BitField<MSLUTSEL> w3{7, 6};
        static constexpr BitField<MSLUTSEL> w2{5, 4};
        static constexpr BitField<MSLUTSEL> w1{3, 2};
        static constexpr BitField<MSLUTSEL> w0{1, 0};
    };

    class MSLUTSTART : public WriteDatagram<MSLUTSTART>
    {
    public:
        static constexpr uint8_t addr{0x69};
        static constexpr BitField<MSLUTSTART> offset_sin90{31, 24};
        static constexpr BitField<MSLUTSTART> start_sin90{23, 16};
        static constexpr BitField<MSLUTSTART> start_sin{7, 0};
    };

    class MSCNT : public WriteDatagram<MSCNT>
    {
    public:
        static constexpr uint8_t addr{0x6A};
        static constexpr BitField<MSCNT> mscnt{9, 0};
    };

    class MSCURACT : public WriteDatagram<MSCURACT>
    {
    public:
        static constexpr uint8_t addr{0x6B};
        static constexpr BitField<MSCURACT> cur_a{24, 16};
        static constexpr BitField<MSCURACT> cur_b{8, 0};
    };

    class CHOPCONF : public WriteDatagram<CHOPCONF>
    {
    public:
        static constexpr uint8_t addr{0x6C};
        static constexpr BitField<CHOPCONF> diss2vs{31};
        static constexpr BitField<CHOPCONF> diss2g{30};
        static constexpr BitField<CHOPCONF> reserved{29};
        static constexpr BitField<CHOPCONF> intpol{28};
        static constexpr BitField<CHOPCONF> mres{27, 24};
        static constexpr BitField<CHOPCONF> tpfd{23, 20};
        static constexpr BitField<CHOPCONF> vhighchm{19};
        static constexpr BitField<CHOPCONF> vhighfs{18};
        static constexpr BitField<CHOPCONF> tbl{16, 15};
        static constexpr BitField<CHOPCONF> chm{14};
        static constexpr BitField<CHOPCONF> disfdcc{12};
        static constexpr BitField<CHOPCONF> fd3{11};
        static constexpr BitField<CHOPCONF> hend_offset{10, 7};
        static constexpr BitField<CHOPCONF> hstrt_tfd210{6, 4};
        static constexpr BitField<CHOPCONF> toff{3, 0};
    };

    class COOLCONF : public WriteDatagram<COOLCONF>
    {
    public:
        static constexpr uint8_t addr{0x6D};
        static constexpr BitField<COOLCONF> sfilt{24};
        static constexpr BitField<COOLCONF> sgt{22, 16};
        static constexpr BitField<COOLCONF> seimin{15};
        static constexpr BitField<COOLCONF> sedn{14, 13};
        static constexpr BitField<COOLCONF> semax{11, 8};
        static constexpr BitField<COOLCONF> seup{6, 5};
        static constexpr BitField<COOLCONF> semin{3, 0};
    };

    class DCCTRL : public WriteDatagram<DCCTRL>
    {
    public:
        static constexpr uint8_t addr{0x6E};
        static constexpr BitField<DCCTRL> dc_sg{23, 16};
        static constexpr BitField<DCCTRL> dc_time{9, 0};
    };

    class DRV_STATUS : public WriteDatagram<DRV_STATUS>
    {
    public:
        static constexpr uint8_t addr{0x6F};
        static constexpr BitField<DRV_STATUS> stst{31};
        static constexpr BitField<DRV_STATUS> olb{30};
        static constexpr BitField<DRV_STATUS> ola{29};
        static constexpr BitField<DRV_STATUS> s2gb{28};
        static constexpr BitField<DRV_STATUS> s2ga{27};
        static constexpr BitField<DRV_STATUS> otpw{26};
        static constexpr BitField<DRV_STATUS> ot{25};
        static constexpr BitField<DRV_STATUS> stallguard{24};
        static constexpr BitField<DRV_STATUS> cs_actual{20, 16};
        static constexpr BitField<DRV_STATUS> fsactive{15};
        static constexpr BitField<DRV_STATUS> stealth{14};
        static constexpr BitField<DRV_STATUS> s2vsb{13};
        static constexpr BitField<DRV_STATUS> s2vsa{12};
        static constexpr BitField<DRV_STATUS> sg_result{9, 0};
    };

    class PWMCONF : public WriteDatagram<PWMCONF>
    {
    public:
        static constexpr uint8_t addr{0x70};
        static constexpr BitField<PWMCONF> pwm_lim{31, 28};
        static constexpr BitField<PWMCONF> pwm_reg{27, 24};
        static constexpr BitField<PWMCONF> pwm_dis_reg_stst{23};
        static constexpr BitField<PWMCONF> pwm_meas_sd_enable{22};
        static constexpr BitField<PWMCONF> freewheel{21, 20};
        static constexpr BitField<PWMCONF> pwm_autograd{19};
        static constexpr BitField<PWMCONF> pwm_autoscale{18};
        static constexpr BitField<PWMCONF> pwm_freq{17, 16};
        static constexpr BitField<PWMCONF> pwm_grad{15, 8};
        static constexpr BitField<PWMCONF> pwm_ofs{7, 0};
    };

    class PWM_SCALE : public WriteDatagram<PWM_SCALE>
    {
    public:
        static constexpr uint8_t addr{0x71};
        static constexpr BitField<PWM_SCALE> pwm_scale_auto{24, 16};
        static constexpr BitField<PWM_SCALE> pwm_scale_sum{9, 0};
    };

    class PWM_AUTO : public WriteDatagram<PWM_AUTO>
    {
    public:
        static constexpr uint8_t addr{0x72};
        static constexpr BitField<PWM_AUTO> pwm_grad_auto{23, 16};
        static constexpr BitField<PWM_AUTO> pwm_ofs_auto{7, 0};
    };

    class SG4_THRS : public WriteDatagram<SG4_THRS>
    {
    public:
        static constexpr uint8_t addr{0x74};
        static constexpr BitField<SG4_THRS> sg_angle_offset{9};
        static constexpr BitField<SG4_THRS> sg4_filt_en{8};
        static constexpr BitField<SG4_THRS> sg4_thrs{7, 0};
    };

    class SG4_RESULT : public WriteDatagram<SG4_RESULT>
    {
    public:
        static constexpr uint8_t addr{0x75};
        static constexpr BitField<SG4_RESULT> sg4_result{9, 0};
    };

    class SG4_IND : public WriteDatagram<SG4_IND>
    {
    public:
        static constexpr uint8_t addr{0x76};
        static constexpr BitField<SG4_IND> sg4_ind_3{31, 24};
        static constexpr BitField<SG4_IND> sg4_ind_2{23, 16};
        static constexpr BitField<SG4_IND> sg4_ind_1{15, 8};
        static constexpr BitField<SG4_IND> sg4_ind_0{7, 0};
    };

    class ReadDatagram
    {
    public:
        uint8_t buf[8] = {0};
        uint32_t payload()
        {
            if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
                return __builtin_bswap32(*reinterpret_cast<uint32_t *>(buf + 4));
            return *reinterpret_cast<uint32_t *>(buf + 4);
        }
        uint8_t &status()
        {
            return buf[3];
        }
        SpiReadBuf readbuf()
        {
            return buf + 3;
        }
    };

    class TMC5240
    {
    public:
        TMC5240(SpiBus &spi_bus, gpio_num_t en_pin, gpio_num_t cs_pin, int device_id, float rref_kohm, float motor_current_rms);
        int device_id() { return this->dev_id; };
        esp_err_t set_velocity(float rpm);
        esp_err_t set_spreadcycle_config(uint8_t toff, uint8_t tbl, uint8_t hstart, uint8_t hend);
        esp_err_t get_position(int32_t &position);

    private:
        esp_err_t config_current();
        esp_err_t config_stealthchop2();
        esp_err_t config_spreadcycle();
        esp_err_t stealthchop2_auto_tune();

        esp_err_t config_motion_ctrl_vel_mode();
        esp_err_t write(const tmc5240::SpiWriteBuf datagram);
        // fills datagram with status + data from the register address sent in the previous read command
        // when single is true, performs 2 transfers and returns the data read in the second transfer
        template <typename reg>
        esp_err_t read_reg(tmc5240::ReadDatagram &datagram, bool single = true);
        template <typename reg>
        esp_err_t update_field(BitField<reg> field, uint32_t value);
        void handle_status(uint8_t status);
        SpiBus &spi_bus;
        const int dev_id;
        const float rref_kohm;
        const float motor_current_rms;
        uint8_t last_status{0};
        bool status_side_effects_in_progress{false};
    };

} // namespace tmc5240

using TMC5240 = tmc5240::TMC5240;
