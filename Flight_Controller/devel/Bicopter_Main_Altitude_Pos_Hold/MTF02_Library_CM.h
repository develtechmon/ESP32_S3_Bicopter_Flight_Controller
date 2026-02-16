
// MTF02_Library.h - Enhanced MTF-02 Optical Flow & Distance Sensor Library
// Version 3.3 - WITH EXTERNAL ALTITUDE SUPPORT
// Author: Flight Controller Integration Version
// Usage: #include "MTF02_Library_CM.h" then call mtf02_init() and mtf02_update()

#ifndef MTF02_LIBRARY_H
#define MTF02_LIBRARY_H

#include <HardwareSerial.h>
#include <math.h>

// ===== HARDWARE CONFIGURATION =====
#define MTF02_RX_PIN 1 
#define MTF02_TX_PIN 3 
#define MTF02_BAUD 115200
#define MTF02_MAVLINK_SYSID 200

// ===== 100Hz PERFORMANCE OPTIMIZATION =====
#define MTF02_MAX_BYTES_PER_UPDATE 300  // Optimized for 100Hz
#define MTF02_EXPECTED_RATE_HZ 100      // Sensor update rate (100Hz)
#define MTF02_TIMEOUT_MS 50             // Fast timeout for 100Hz

// ===== SENSOR PARAMETERS =====
#define MTF02_FLOW_MIN_QUALITY 50       // Minimum quality threshold to check if speed still drops to 0

#define MTF02_MIN_DISTANCE_CM 8         // Minimum valid distance
#define MTF02_MAX_DISTANCE_CM 300       // Maximum valid distance

// ===== SENSOR SPECIFICATIONS =====
#define MTF02_FOV_RAD 0.733             // 42 degrees in radians
#define MTF02_SENSOR_RESOLUTION 35.0    // PMW3901 sensor resolution (35x35 pixels)
#define MTF02_FLOW_SCALE_FACTOR 100.0   // Flow scale factor (from MicoAssistant)

// ===== ORIENTATION OPTIONS =====
enum MTF02_Orientation {
    MTF02_ORIENT_FORWARD = 0,
    MTF02_ORIENT_BACKWARD,
    MTF02_ORIENT_LEFT,
    MTF02_ORIENT_RIGHT
};

// ===== INTERNAL DATA STRUCTURES =====
struct MTF02_RawData {
    int16_t flow_x_raw;
    int16_t flow_y_raw;
    uint16_t distance_cm;
    uint8_t flow_quality;
    uint8_t sensor_id;
    float flow_x_pixels;
    float flow_y_pixels;
    uint32_t last_flow_update;
    uint32_t last_distance_update;
    bool flow_valid;
    bool distance_valid;
};

struct MTF02_Navigation {
    float velocity_x_cms;           // Velocity X in cm/s
    float velocity_y_cms;           // Velocity Y in cm/s
    float velocity_z_cms;           // Velocity Z in cm/s
    float position_x_cm;            // Position X in cm
    float position_y_cm;            // Position Y in cm
    float position_z_cm;            // Position Z in cm
    uint32_t last_nav_update;
    bool nav_valid;
};

struct MTF02_Calibration {
    float flow_scale_x;
    float flow_scale_y;
    int16_t flow_offset_x;
    int16_t flow_offset_y;
    float rotation_correction;
    bool use_trigonometric_scaling;
};

struct MTF02_Performance {
    uint32_t flow_packets_received;
    uint32_t distance_packets_received;
    uint32_t crc_errors;
    uint32_t parse_errors;
    uint32_t timeouts;
    float actual_rate_hz;
    uint32_t last_rate_calc;
    uint32_t rate_counter;
    float avg_flow_quality;
    uint32_t quality_samples;
    float max_velocity_x_cms;       // Max velocity in cm/s
    float max_velocity_y_cms;       // Max velocity in cm/s
};

// ===== EXTERNAL ALTITUDE INTEGRATION =====
struct MTF02_ExternalAltitude {
    bool use_external_altitude;
    float external_altitude_cm;
    uint32_t last_external_update;
    uint32_t external_timeout_ms;
    bool external_valid;
};

// ===== GLOBAL INSTANCES =====
static MTF02_RawData mtf02_raw = {0};
static MTF02_Navigation mtf02_nav = {0};
static MTF02_Calibration mtf02_cal = {1.0, 1.0, 0, 0, 0.0, true};
static MTF02_Performance mtf02_perf = {0};
static MTF02_ExternalAltitude mtf02_ext_alt = {false, 0.0f, 0, 100, false}; // NEW
static MTF02_Orientation mtf02_orient = MTF02_ORIENT_FORWARD;
static HardwareSerial mtf02_serial(1);

// ===== PRIVATE FUNCTION DECLARATIONS =====
void mtf02_parse_mavlink();
void mtf02_process_optical_flow(uint8_t* payload);
void mtf02_process_distance_sensor(uint8_t* payload);
void mtf02_update_navigation();
void mtf02_apply_calibration();
void mtf02_apply_orientation(float* x, float* y);
void mtf02_crc_add(uint8_t d, uint16_t* crc);
void mtf02_update_performance_stats();
float mtf02_calculate_cm_per_pixel(float height_cm);

// =============================================================================
// PUBLIC API FUNCTIONS - Call these from your flight controller
// =============================================================================

// ===== INITIALIZATION =====
void mtf02_init() {
    mtf02_serial.begin(MTF02_BAUD, SERIAL_8N1, MTF02_RX_PIN, MTF02_TX_PIN);
    
    memset(&mtf02_raw, 0, sizeof(mtf02_raw));
    memset(&mtf02_nav, 0, sizeof(mtf02_nav));
    memset(&mtf02_perf, 0, sizeof(mtf02_perf));
    memset(&mtf02_ext_alt, 0, sizeof(mtf02_ext_alt)); // NEW
    
    mtf02_cal.flow_scale_x = 1.0;
    mtf02_cal.flow_scale_y = 1.0;
    mtf02_cal.flow_offset_x = 0;
    mtf02_cal.flow_offset_y = 0;
    mtf02_cal.rotation_correction = 0.0;
    mtf02_cal.use_trigonometric_scaling = true;
    
    // Initialize external altitude system
    mtf02_ext_alt.use_external_altitude = false;
    mtf02_ext_alt.external_altitude_cm = 0.0f;
    mtf02_ext_alt.external_timeout_ms = 100;  // 100ms timeout for external altitude
    mtf02_ext_alt.external_valid = false;
    
    uint32_t now = millis();
    mtf02_nav.last_nav_update = now;
    mtf02_perf.last_rate_calc = now;
}

// ===== MAIN UPDATE - Call this in your main loop =====
void mtf02_update() {
    mtf02_parse_mavlink();
    mtf02_update_navigation();
    mtf02_update_performance_stats();
    
    // Check external altitude timeout
    if (mtf02_ext_alt.use_external_altitude) {
        uint32_t now = millis();
        if ((now - mtf02_ext_alt.last_external_update) > mtf02_ext_alt.external_timeout_ms) {
            mtf02_ext_alt.external_valid = false;
        }
    }
}

// =============================================================================
// EXTERNAL ALTITUDE FUNCTIONS - NEW
// =============================================================================

void mtf02_set_external_altitude(float altitude_cm) {
    mtf02_ext_alt.external_altitude_cm = altitude_cm;
    mtf02_ext_alt.last_external_update = millis();
    mtf02_ext_alt.external_valid = true;
}

void mtf02_enable_external_altitude(bool enable) {
    mtf02_ext_alt.use_external_altitude = enable;
    if (!enable) {
        mtf02_ext_alt.external_valid = false;
    }
}

bool mtf02_is_using_external_altitude() {
    return mtf02_ext_alt.use_external_altitude && mtf02_ext_alt.external_valid;
}

void mtf02_set_external_altitude_timeout(uint32_t timeout_ms) {
    mtf02_ext_alt.external_timeout_ms = timeout_ms;
}

float mtf02_get_effective_altitude() {
    if (mtf02_is_using_external_altitude()) {
        return mtf02_ext_alt.external_altitude_cm;
    } else {
        return (float)mtf02_raw.distance_cm;
    }
}

// =============================================================================
// DISTANCE/ALTITUDE FUNCTIONS - For Altitude Hold (CM UNITS)
// =============================================================================

uint16_t mtf02_get_raw_distance_cm() {
    return mtf02_raw.distance_valid ? mtf02_raw.distance_cm : 0;
}

uint16_t mtf02_get_distance_cm() {
    // Return external altitude if available, otherwise raw sensor
    if (mtf02_is_using_external_altitude()) {
        return (uint16_t)mtf02_ext_alt.external_altitude_cm;
    } else {
        return mtf02_raw.distance_valid ? mtf02_raw.distance_cm : 0;
    }
}

float mtf02_get_distance_m() {
    float distance_cm = mtf02_get_distance_cm();
    return (distance_cm > 0) ? (distance_cm / 100.0f) : -1.0f;
}

uint16_t mtf02_get_altitude_cm() {
    return mtf02_get_distance_cm();
}

float mtf02_get_altitude_m() {
    return mtf02_get_distance_m();
}

bool mtf02_is_distance_valid() {
    uint32_t now = millis();
    
    if (mtf02_is_using_external_altitude()) {
        // External altitude is valid if it's recent and within range
        return mtf02_ext_alt.external_valid &&
               mtf02_ext_alt.external_altitude_cm >= MTF02_MIN_DISTANCE_CM &&
               mtf02_ext_alt.external_altitude_cm <= MTF02_MAX_DISTANCE_CM;
    } else {
        // Use raw sensor validation
        return mtf02_raw.distance_valid && 
               (now - mtf02_raw.last_distance_update) < MTF02_TIMEOUT_MS &&
               mtf02_raw.distance_cm >= MTF02_MIN_DISTANCE_CM &&
               mtf02_raw.distance_cm <= MTF02_MAX_DISTANCE_CM;
    }
}

bool mtf02_is_altitude_valid() {
    return mtf02_is_distance_valid();
}

// =============================================================================
// VELOCITY FUNCTIONS - For Position Hold (CM/S UNITS)
// =============================================================================

float mtf02_get_velocity_x_cms() {
    return mtf02_nav.nav_valid ? mtf02_nav.velocity_x_cms : 0.0f;
}

float mtf02_get_velocity_y_cms() {
    return mtf02_nav.nav_valid ? mtf02_nav.velocity_y_cms : 0.0f;
}

float mtf02_get_velocity_z_cms() {
    return mtf02_nav.nav_valid ? mtf02_nav.velocity_z_cms : 0.0f;
}

// Legacy functions for compatibility (returns cm/s)
float mtf02_get_velocity_x() {
    return mtf02_get_velocity_x_cms();
}

float mtf02_get_velocity_y() {
    return mtf02_get_velocity_y_cms();
}

float mtf02_get_velocity_z() {
    return mtf02_get_velocity_z_cms();
}

void mtf02_get_velocity_cms(float* vel_x, float* vel_y, float* vel_z) {
    if (vel_x) *vel_x = mtf02_get_velocity_x_cms();
    if (vel_y) *vel_y = mtf02_get_velocity_y_cms();
    if (vel_z) *vel_z = mtf02_get_velocity_z_cms();
}

// Meters per second functions (for compatibility)
float mtf02_get_velocity_x_ms() {
    return mtf02_nav.nav_valid ? (mtf02_nav.velocity_x_cms / 100.0f) : 0.0f;
}

float mtf02_get_velocity_y_ms() {
    return mtf02_nav.nav_valid ? (mtf02_nav.velocity_y_cms / 100.0f) : 0.0f;
}

float mtf02_get_velocity_z_ms() {
    return mtf02_nav.nav_valid ? (mtf02_nav.velocity_z_cms / 100.0f) : 0.0f;
}

void mtf02_get_velocity_ms(float* vel_x, float* vel_y, float* vel_z) {
    if (vel_x) *vel_x = mtf02_get_velocity_x_ms();
    if (vel_y) *vel_y = mtf02_get_velocity_y_ms();
    if (vel_z) *vel_z = mtf02_get_velocity_z_ms();
}

bool mtf02_is_velocity_valid() {
    return mtf02_nav.nav_valid && mtf02_raw.flow_valid && mtf02_is_distance_valid();
}

// =============================================================================
// POSITION FUNCTIONS - For Position Hold (CM UNITS)
// =============================================================================

float mtf02_get_position_x_cm() {
    return mtf02_nav.position_x_cm;
}

float mtf02_get_position_y_cm() {
    return mtf02_nav.position_y_cm;
}

float mtf02_get_position_z_cm() {
    return mtf02_nav.position_z_cm;
}

// Legacy functions for compatibility (returns cm)
float mtf02_get_position_x() {
    return mtf02_get_position_x_cm();
}

float mtf02_get_position_y() {
    return mtf02_get_position_y_cm();
}

float mtf02_get_position_z() {
    return mtf02_get_position_z_cm();
}

void mtf02_get_position_cm(float* pos_x, float* pos_y, float* pos_z) {
    if (pos_x) *pos_x = mtf02_nav.position_x_cm;
    if (pos_y) *pos_y = mtf02_nav.position_y_cm;
    if (pos_z) *pos_z = mtf02_nav.position_z_cm;
}

// Meters functions (for compatibility)
float mtf02_get_position_x_m() {
    return mtf02_nav.position_x_cm / 100.0f;
}

float mtf02_get_position_y_m() {
    return mtf02_nav.position_y_cm / 100.0f;
}

float mtf02_get_position_z_m() {
    return mtf02_nav.position_z_cm / 100.0f;
}

void mtf02_get_position_m(float* pos_x, float* pos_y, float* pos_z) {
    if (pos_x) *pos_x = mtf02_get_position_x_m();
    if (pos_y) *pos_y = mtf02_get_position_y_m();
    if (pos_z) *pos_z = mtf02_get_position_z_m();
}

void mtf02_reset_position() {
    mtf02_nav.position_x_cm = 0.0f;
    mtf02_nav.position_y_cm = 0.0f;
    mtf02_nav.last_nav_update = millis();
}

void mtf02_reset_position_xy() {
    mtf02_nav.position_x_cm = 0.0f;
    mtf02_nav.position_y_cm = 0.0f;
}

// =============================================================================
// OPTICAL FLOW FUNCTIONS - For Advanced Control
// =============================================================================

float mtf02_get_flow_x_pixels() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_x_pixels : 0.0f;
}

float mtf02_get_flow_y_pixels() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_y_pixels : 0.0f;
}

int16_t mtf02_get_flow_x_raw() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_x_raw : 0;
}

int16_t mtf02_get_flow_y_raw() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_y_raw : 0;
}

uint8_t mtf02_get_flow_quality() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_quality : 0;
}

bool mtf02_is_flow_valid() {
    uint32_t now = millis();
    return mtf02_raw.flow_valid && 
           (now - mtf02_raw.last_flow_update) < MTF02_TIMEOUT_MS &&
           mtf02_raw.flow_quality >= MTF02_FLOW_MIN_QUALITY;
}

// =============================================================================
// SYSTEM STATUS FUNCTIONS
// =============================================================================

bool mtf02_is_ready_for_position_hold() {
    return mtf02_is_flow_valid() && mtf02_is_distance_valid() && mtf02_is_velocity_valid();
}

bool mtf02_is_ready_for_altitude_hold() {
    return mtf02_is_distance_valid();
}

float mtf02_get_update_rate() {
    return mtf02_perf.actual_rate_hz;
}

bool mtf02_is_system_healthy() {
    float error_rate = (float)(mtf02_perf.crc_errors + mtf02_perf.parse_errors) / 
                       (float)(mtf02_perf.flow_packets_received + mtf02_perf.distance_packets_received + 1);
    return error_rate < 0.05 && 
           mtf02_perf.actual_rate_hz > 80.0 && 
           mtf02_get_flow_quality() > MTF02_FLOW_MIN_QUALITY;
}

// =============================================================================
// CONFIGURATION FUNCTIONS
// =============================================================================

void mtf02_set_calibration(float scale_x, float scale_y, int16_t offset_x, int16_t offset_y) {
    mtf02_cal.flow_scale_x = scale_x;
    mtf02_cal.flow_scale_y = scale_y;
    mtf02_cal.flow_offset_x = offset_x;
    mtf02_cal.flow_offset_y = offset_y;
}

void mtf02_set_orientation(MTF02_Orientation orientation) {
    mtf02_orient = orientation;
}

// =============================================================================
// DEBUG FUNCTIONS - ENHANCED
// =============================================================================

void mtf02_print_status() {
    Serial.printf("MTF02[100Hz]: Alt=%dcm(Raw=%dcm) FlowQ=%d VelX=%.1f VelY=%.1f PosX=%.1f PosY=%.1f Ready=%s Rate=%.1fHz ExtAlt=%s\n",
                 mtf02_get_distance_cm(),
                 mtf02_get_raw_distance_cm(),
                 mtf02_get_flow_quality(),
                 mtf02_get_velocity_x_cms(),
                 mtf02_get_velocity_y_cms(),
                 mtf02_get_position_x_cm(),
                 mtf02_get_position_y_cm(),
                 mtf02_is_ready_for_position_hold() ? "YES" : "NO",
                 mtf02_get_update_rate(),
                 mtf02_is_using_external_altitude() ? "ON" : "OFF");
}

void mtf02_print_debug() {
    Serial.printf("MTF02 DEBUG: Dist=%dcm FlowX=%d FlowY=%d Quality=%d VelX=%.1f VelY=%.1f cm/s ExtAlt=%.1f Valid=%s\n",
                 mtf02_get_distance_cm(),
                 mtf02_get_flow_x_raw(),
                 mtf02_get_flow_y_raw(),
                 mtf02_get_flow_quality(),
                 mtf02_get_velocity_x_cms(),
                 mtf02_get_velocity_y_cms(),
                 mtf02_ext_alt.external_altitude_cm,
                 mtf02_ext_alt.external_valid ? "YES" : "NO");
}

// =============================================================================
// PRIVATE IMPLEMENTATION FUNCTIONS - ENHANCED FOR EXTERNAL ALTITUDE
// =============================================================================

float mtf02_calculate_cm_per_pixel(float height_cm) {
    if (mtf02_cal.use_trigonometric_scaling) {
        // CORRECTED: Use tangent for proper geometric scaling in CM
        float half_fov = MTF02_FOV_RAD / 2.0f;
        float height_m = height_cm / 100.0f;  // Convert to meters for calculation
        float ground_width_m = 2.0f * height_m * tan(half_fov);
        float ground_width_cm = ground_width_m * 100.0f;  // Convert back to cm
        return ground_width_cm / MTF02_SENSOR_RESOLUTION;
    } else {
        float radians_per_pixel = MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION;
        return height_cm * radians_per_pixel;
    }
}

// ENHANCED: Navigation update with external altitude support
void mtf02_update_navigation() {
    uint32_t now = millis();
    
    if (!mtf02_is_flow_valid() || !mtf02_is_distance_valid()) {
        mtf02_nav.nav_valid = false;
        return;
    }
    
    // ENHANCED: Use external altitude if available, otherwise use raw sensor
    float height_cm = mtf02_get_effective_altitude();
    
    if (height_cm > 0) {
        float cm_per_pixel = mtf02_calculate_cm_per_pixel(height_cm);
        
        // Calculate velocity in cm/s (CORRECTED FOR CM UNITS)
        mtf02_nav.velocity_x_cms = mtf02_raw.flow_x_pixels * cm_per_pixel * MTF02_EXPECTED_RATE_HZ;
        mtf02_nav.velocity_y_cms = mtf02_raw.flow_y_pixels * cm_per_pixel * MTF02_EXPECTED_RATE_HZ;
        
        mtf02_apply_orientation(&mtf02_nav.velocity_x_cms, &mtf02_nav.velocity_y_cms);
        
        if (abs(mtf02_cal.rotation_correction) > 0.01) {
            float cos_rot = cos(mtf02_cal.rotation_correction);
            float sin_rot = sin(mtf02_cal.rotation_correction);
            float temp_x = mtf02_nav.velocity_x_cms * cos_rot - mtf02_nav.velocity_y_cms * sin_rot;
            float temp_y = mtf02_nav.velocity_x_cms * sin_rot + mtf02_nav.velocity_y_cms * cos_rot;
            mtf02_nav.velocity_x_cms = temp_x;
            mtf02_nav.velocity_y_cms = temp_y;
        }
    }
    
    // ENHANCED: Calculate Z velocity from altitude changes (use effective altitude)
    static float prev_altitude = 0;
    if (prev_altitude > 0) {
        float dt = (now - mtf02_nav.last_nav_update) / 1000.0f;
        if (dt > 0 && dt < 0.05f) {
            float altitude_change_cm = height_cm - prev_altitude;
            mtf02_nav.velocity_z_cms = altitude_change_cm / dt;
        }
    }
    prev_altitude = height_cm;
    
    // Integrate position (in cm)
    if (mtf02_nav.last_nav_update > 0) {
        float dt = (now - mtf02_nav.last_nav_update) / 1000.0f;
        if (dt > 0 && dt < 0.05f) {
            mtf02_nav.position_x_cm += mtf02_nav.velocity_x_cms * dt;
            mtf02_nav.position_y_cm += mtf02_nav.velocity_y_cms * dt;
        }
    }
    
    // Z position from effective altitude (filtered or raw)
    mtf02_nav.position_z_cm = height_cm;
    mtf02_nav.last_nav_update = now;
    mtf02_nav.nav_valid = true;
    
    // Update performance statistics (in cm/s)
    mtf02_perf.max_velocity_x_cms = max(mtf02_perf.max_velocity_x_cms, abs(mtf02_nav.velocity_x_cms));
    mtf02_perf.max_velocity_y_cms = max(mtf02_perf.max_velocity_y_cms, abs(mtf02_nav.velocity_y_cms));
}

void mtf02_apply_calibration() {
    mtf02_raw.flow_x_pixels = (mtf02_raw.flow_x_raw - mtf02_cal.flow_offset_x) * 
                              mtf02_cal.flow_scale_x / MTF02_FLOW_SCALE_FACTOR;
    mtf02_raw.flow_y_pixels = (mtf02_raw.flow_y_raw - mtf02_cal.flow_offset_y) * 
                              mtf02_cal.flow_scale_y / MTF02_FLOW_SCALE_FACTOR;
}

void mtf02_apply_orientation(float* x, float* y) {
    float temp_x = *x;
    float temp_y = *y;
    
    switch(mtf02_orient) {
        case MTF02_ORIENT_BACKWARD:
            *x = -temp_x; 
            *y = -temp_y; 
            break;
        case MTF02_ORIENT_LEFT:
            *x = temp_y; 
            *y = -temp_x; 
            break;
        case MTF02_ORIENT_RIGHT:
            *x = -temp_y; 
            *y = temp_x; 
            break;
        default:
            break;
    }
}

void mtf02_parse_mavlink() {
    static uint8_t state = 0, len, msgid, idx;
    static uint8_t buf[64];
    static uint16_t crc;
    static uint8_t ck_a;
    
    int bytes_processed = 0;
    
    while (mtf02_serial.available() && bytes_processed < MTF02_MAX_BYTES_PER_UPDATE) {
        uint8_t c = mtf02_serial.read();
        bytes_processed++;
        
        switch (state) {
            case 0:
                if (c == 0xFE) { 
                    crc = 0xFFFF; 
                    state = 1; 
                }
                break;
            case 1:
                len = c; 
                mtf02_crc_add(c, &crc); 
                state = 2; 
                break;
            case 2:
                mtf02_crc_add(c, &crc); 
                state = 3; 
                break;
            case 3:
                mtf02_crc_add(c, &crc); 
                if (c != MTF02_MAVLINK_SYSID) { 
                    state = 0; 
                    mtf02_perf.parse_errors++; 
                } else {
                    state = 4; 
                }
                break;
            case 4:
                mtf02_crc_add(c, &crc); 
                state = 5; 
                break;
            case 5:
                msgid = c; 
                mtf02_crc_add(c, &crc); 
                idx = 0; 
                state = 6; 
                break;
            case 6:
                if (idx < len && idx < sizeof(buf)) {
                    buf[idx++] = c; 
                    mtf02_crc_add(c, &crc);
                    if (idx == len) {
                        if (msgid == 100) mtf02_crc_add(175, &crc);
                        if (msgid == 132) mtf02_crc_add(85, &crc);
                        state = 7;
                    }
                } else { 
                    state = 0; 
                    mtf02_perf.parse_errors++; 
                }
                break;
            case 7:
                ck_a = c; 
                state = 8; 
                break;
            case 8:
                {
                    uint16_t received_crc = ck_a | (c << 8);
                    if (received_crc == crc) {
                        if (msgid == 100) {
                            mtf02_process_optical_flow(buf);
                        } else if (msgid == 132) {
                            mtf02_process_distance_sensor(buf);
                        }
                    } else {
                        mtf02_perf.crc_errors++;
                    }
                    state = 0;
                }
                break;
        }
    }
}

void mtf02_process_optical_flow(uint8_t* payload) {
    mtf02_raw.flow_x_raw = payload[20] | (payload[21] << 8);
    mtf02_raw.flow_y_raw = payload[22] | (payload[23] << 8);
    mtf02_raw.sensor_id = payload[24];
    mtf02_raw.flow_quality = payload[25];
    
    mtf02_apply_calibration();
    
    mtf02_raw.flow_valid = true;
    mtf02_raw.last_flow_update = millis();
    
    mtf02_perf.flow_packets_received++;
    mtf02_perf.avg_flow_quality = 
        (mtf02_perf.avg_flow_quality * mtf02_perf.quality_samples + mtf02_raw.flow_quality) / 
        (mtf02_perf.quality_samples + 1);
    mtf02_perf.quality_samples++;
}

void mtf02_process_distance_sensor(uint8_t* payload) {
    mtf02_raw.distance_cm = payload[8] | (payload[9] << 8);
    mtf02_raw.distance_valid = true;
    mtf02_raw.last_distance_update = millis();
    mtf02_perf.distance_packets_received++;
}

void mtf02_crc_add(uint8_t d, uint16_t* crc) {
    uint8_t tmp = d ^ (*crc & 0xFF);
    tmp ^= tmp << 4;
    *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void mtf02_update_performance_stats() {
    uint32_t now = millis();
    mtf02_perf.rate_counter++;
    
    if (now - mtf02_perf.last_rate_calc >= 1000) {
        mtf02_perf.actual_rate_hz = mtf02_perf.rate_counter * 1000.0f / (now - mtf02_perf.last_rate_calc);
        mtf02_perf.rate_counter = 0;
        mtf02_perf.last_rate_calc = now;
    }
    
    if ((now - mtf02_raw.last_flow_update) > MTF02_TIMEOUT_MS && mtf02_raw.flow_valid) {
        mtf02_perf.timeouts++;
        mtf02_raw.flow_valid = false;
    }
    if ((now - mtf02_raw.last_distance_update) > MTF02_TIMEOUT_MS && mtf02_raw.distance_valid) {
        mtf02_perf.timeouts++;
        mtf02_raw.distance_valid = false;
    }
}

#endif // MTF02_LIBRARY_H
