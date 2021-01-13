#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    locked_heading = false;
    lock_timer_ms = 0;

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    // Reset landing
    landing = false;

    return true;
}

void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (!landing && (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0)) {
        locked_heading = false;
        lock_timer_ms = 0;
    }

    if (!locked_heading) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }

    if (landing) {
        plane.nav_pitch_cd = (int32_t)(plane.channel_pitch->norm_input() * ControllerData::pitch.maxCmdDeg * 100);
        plane.calc_throttle();
    } else
        plane.update_fbwb_speed_height();
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void ModeCruise::navigate()
{
    if (!locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&
        plane.rudder_input() == 0 &&
        plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        plane.gps.ground_speed() >= 3 &&
        lock_timer_ms == 0 &&
        !landing) {
        // user wants to lock the heading - start the timer
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        locked_heading = true;
        lock_timer_ms = 0;
        locked_heading_cd = plane.gps.ground_course_cd();
        plane.prev_WP_loc = plane.current_loc;
    }
    if (locked_heading) {
        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

void ModeCruise::set_cruise_land()
{
    // Waypoint parameters
    // TODO: set in params?
    // Malesherbes
    double runway_wpt_a[2] = {48.299370, 2.362039};
    double runway_wpt_b[2] = {48.297366, 2.362289};
    // Compute bearing
    Location loc_a = plane.current_loc;
    loc_a.lat = (int32_t)(runway_wpt_a[0] * 1e7);
    loc_a.lng = (int32_t)(runway_wpt_a[1] * 1e7);
    Location loc_b = plane.current_loc;
    loc_b.lat = (int32_t)(runway_wpt_b[0] * 1e7);
    loc_b.lng = (int32_t)(runway_wpt_b[1] * 1e7);
    float ab = loc_a.get_bearing(loc_b);
    // printf("loc_a %d %d loc_b %d %d bearing %f rad\n", loc_a.lat, loc_a.lng, loc_b.lat, loc_b.lng, ab);
    float bearing = radians(plane.gps.ground_course());
    float delta_bearing = wrap_PI(bearing - ab);
    plane.prev_WP_loc = loc_a;
    if (fabsf(delta_bearing) > M_PI_2) {
        ab = wrap_PI(ab + M_PI);
        plane.prev_WP_loc = loc_b;
    }
    locked_heading_cd = (int32_t)(degrees(ab) * 100);
    landing = true;
}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading)
{
    target_heading = locked_heading_cd;
    return locked_heading;
}
