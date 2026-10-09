#include "mode.h"
#include "Plane.h"

/*
  wig-vtol-sim: yer etkisi yükseklik modu (GROUND_EFFECT).

  Başlangıç noktası squilter'ın yer etkisi modu (wig-vtol-sim sim/ardupilot/port_215fb9a.patch). Farklar:
  - yükseklik yalnız aşağı bakan mesafe ölçerden (EKF'nin yükseklik kestirimi kullanılmaz); tırmanma hızı EKF'den;
  - yükseklik elevatörü doğrudan verir (wig-vtol-sim kullanıcı kararı: stok açı döngüsü ve girdi şekillendirici atlanır):
    çıkış = GE_ELEV + P·e + I∫e + D·(ḣ_dem − ḣ) − GE_PTCH_T·(θ − PTCH_TRIM_DEG − γ_dem) − GE_PTCH_Q·q, cd/100 (±45 tam
    sapma); γ_dem = ḣ_dem/AIRSPEED_CRUISE (istek rampasında gereken yörünge açısı, ileri besleme);
    yatış ve sapma stok (stabilize_roll, stabilize_yaw);
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
    i_spd = 0;
    // kesintisiz giriş: integratör, ilk çıkış o anki elevatör çıkışına eşit olacak şekilde (IMAX içinde)
    elev_out = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * 0.01;
    i_hgt = constrain_float(elev_out - plane.g2.ge_elev - pitch_terms(0.0), -plane.g2.ge_hgt_imax, plane.g2.ge_hgt_imax);
    last_good_ms = AP_HAL::millis();
    return true;
}

// D terimi ve tutum geri beslemesi: D·(ḣ_dem − ḣ) − T·(θ − PTCH_TRIM_DEG − γ_dem) − Q·q  [cd/100]
float ModeGroundEffect::pitch_terms(float hdot_dem)
{
    Vector3f vel;
    hdot = plane.ahrs.get_velocity_NED(vel) ? -vel.z : 0.0;
    const float gamma_dem = degrees(hdot_dem / MAX(plane.aparm.airspeed_cruise.get(), 1.0f));
    return plane.g2.ge_hgt_d * (hdot_dem - hdot)
           - plane.g2.ge_ptch_t * (plane.ahrs.get_pitch_deg() - plane.g.pitch_trim - gamma_dem)
           - plane.g2.ge_ptch_q * degrees(plane.ahrs.get_gyro().y);
}

void ModeGroundEffect::run()
{
    plane.stabilize_roll();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_out * 100);
    plane.pitchController.reset();  // stok yunuslama denetleyicisi çalışmıyor (mode_training gibi)
    plane.stabilize_yaw();
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

    const float err = h_dem - h_filt;
    const float imax = plane.g2.ge_hgt_imax;
    i_hgt = constrain_float(i_hgt + plane.g2.ge_hgt_i * err * dt, -imax, imax);
    // ponytail: hız ölçekleyici yok (elevatör etkinliği V²; mod seyir hızı çevresinde, V ≥ 1.2·V_s); geniş hız aralığında gerekirse
    elev_out = constrain_float(plane.g2.ge_elev + plane.g2.ge_hgt_p * err + i_hgt + pitch_terms(hdot_dem), -45, 45);

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
    // @Field: E: elevator output (centidegrees/100)
    // @Field: IH: height integrator
    // @Field: VE: airspeed error
    // @Field: Thr: throttle demand
    // @Field: IV: airspeed integrator
    AP::logger().WriteStreaming("GEFF", "TimeUS,H,HD,HDot,E,IH,VE,Thr,IV", "Qffffffff",
                                AP_HAL::micros64(), h_filt, h_dem, hdot, elev_out, i_hgt, v_err, thr, i_spd);
#endif
}
