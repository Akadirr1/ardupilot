#include "mode.h"
#include "Plane.h"

/*
  wig-vtol-sim: yer etkisi yükseklik modu (GROUND_EFFECT).

  Başlangıç noktası squilter'ın yer etkisi modu (wig-vtol-sim sim/ardupilot/port_215fb9a.patch). Farklar:
  - yükseklik yalnız aşağı bakan mesafe ölçerden (EKF'nin yükseklik kestirimi kullanılmaz); tırmanma hızı EKF'den;
  - dış yükseklik döngüsü nav_pitch_cd'yi verir (P + I + D, D tırmanma hızı hatasında), iç yunuslama ve yatış döngüleri
    stok (Mode::run → stabilize_pitch: PTCH_TRIM_DEG ve AP_PitchController);
  - gaz hava hızından (AIRSPEED_CRUISE), TRIM_THROTTLE etrafında P + I; squilter'da gaz yükseklik hatasındandı;
  - yanal kısım CRUISE'unki (rota kilidi);
  - girişte istek ölçülen yükseklikten GE_HGT'ye en çok GE_SINK hızıyla iner (yer etkisine giriş);
  - mesafe ölçer 0.5 s okuma vermezse CRUISE'a geçer.
  Kazançlar wig-vtol-sim'de doğrusal modelden tasarlanır (params/d1_arac.yaml); varsayılanlar 0.
*/

static constexpr uint32_t GE_RANGEFINDER_TIMEOUT_MS = 500;

bool ModeGroundEffect::read_height(float &h_m)
{
    const RangeFinder *rf = AP::rangefinder();
    if (rf == nullptr) {
        return false;
    }
    switch (rf->status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::Good:
        h_m = rf->distance_orient(ROTATION_PITCH_270);
        return true;
    case RangeFinder::Status::OutOfRangeLow:
        // en kısa menzilin altındayız: en az bu kadar alçak
        h_m = rf->min_distance_orient(ROTATION_PITCH_270);
        return true;
    case RangeFinder::Status::OutOfRangeHigh:
        h_m = rf->max_distance_orient(ROTATION_PITCH_270);
        return true;
    default:
        return false;
    }
}

bool ModeGroundEffect::_enter()
{
    float h;
    if (!is_positive(plane.g2.ge_hgt) || !is_positive(plane.g2.ge_sink) || !read_height(h)) {
        return false;
    }
    if (!ModeCruise::_enter()) {
        return false;
    }
    h_filt = h;
    h_dem = h;
    i_hgt = 0;
    i_spd = 0;
    last_good_ms = AP_HAL::millis();
    return true;
}

void ModeGroundEffect::update()
{
    const float dt = plane.scheduler.get_loop_period_s();
    const uint32_t now = AP_HAL::millis();

    // yanal: CRUISE ile aynı (ModeCruise::update'in yanal kısmı)
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        locked_heading = false;
        lock_timer_ms = 0;
    }
    if (!locked_heading) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }

    // yükseklik: mesafe ölçer, birinci mertebe süzgeç
    float h;
    if (read_height(h)) {
        last_good_ms = now;
        const float a = is_positive(plane.g2.ge_hgt_flt) ? calc_lowpass_alpha_dt(dt, plane.g2.ge_hgt_flt) : 1.0;
        h_filt += a * (h - h_filt);
    } else if (now - last_good_ms > GE_RANGEFINDER_TIMEOUT_MS) {
        gcs().send_text(MAV_SEVERITY_WARNING, "GEFF: rangefinder lost");
        IGNORE_RETURN(plane.set_mode(plane.mode_cruise, ModeReason::UNAVAILABLE));
        return;
    }

    // istek: girişteki yükseklikten GE_HGT'ye en çok GE_SINK hızıyla
    const float step = constrain_float(plane.g2.ge_hgt - h_dem, -plane.g2.ge_sink * dt, plane.g2.ge_sink * dt);
    h_dem += step;
    const float hdot_dem = is_positive(dt) ? step / dt : 0.0;

    Vector3f vel;
    const float hdot = plane.ahrs.get_velocity_NED(vel) ? -vel.z : 0.0;

    const float err = h_dem - h_filt;
    const float imax = plane.g2.ge_hgt_imax;
    i_hgt = constrain_float(i_hgt + plane.g2.ge_hgt_i * err * dt, -imax, imax);
    const float pitch_deg = plane.g2.ge_hgt_p * err + i_hgt + plane.g2.ge_hgt_d * (hdot_dem - hdot);
    plane.nav_pitch_cd = constrain_int32(pitch_deg * 100, plane.pitch_limit_min * 100, plane.aparm.pitch_limit_max * 100);

    // gaz: hava hızı, TRIM_THROTTLE etrafında
    float aspeed;
    const float v_err = plane.ahrs.airspeed_EAS(aspeed) ? plane.aparm.airspeed_cruise - aspeed : 0.0;
    const float vmax = plane.g2.ge_spd_imax;
    i_spd = constrain_float(i_spd + plane.g2.ge_spd_i * v_err * dt, -vmax, vmax);
    const float thr = plane.aparm.throttle_cruise + plane.g2.ge_spd_p * v_err + i_spd;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, thr);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GEFF
    // @Description: wig-vtol-sim ground effect mode
    // @Field: TimeUS: Time since system startup
    // @Field: H: filtered rangefinder height
    // @Field: HD: height demand
    // @Field: HDot: EKF climb rate
    // @Field: PD: pitch demand (relative to PTCH_TRIM_DEG)
    // @Field: IH: height integrator
    // @Field: VE: airspeed error
    // @Field: Thr: throttle demand
    // @Field: IV: airspeed integrator
    AP::logger().WriteStreaming("GEFF", "TimeUS,H,HD,HDot,PD,IH,VE,Thr,IV", "Qffffffff",
                                AP_HAL::micros64(), h_filt, h_dem, hdot, pitch_deg, i_hgt, v_err, thr, i_spd);
#endif
}
