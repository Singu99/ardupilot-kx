#include "AP_Beacon_KonexUWB.h"

#if AP_BEACON_KONEXUWB_ENABLED

#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>

#define KONEXUWB_PREABLE 0x55

extern const AP_HAL::HAL& hal;

bool AP_Beacon_KonexUWB::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// TODD: Handle case when there is more available data than the size of the frame. Then we should drop the oldest data and read the latest frame.
void AP_Beacon_KonexUWB::update()
{
    if (uart == nullptr) {
        return;
    }
    // read any available characters
    uint16_t num_bytes_read = MIN(uart->available(), sizeof(KonexUWBFrame) - num_bytes_in_block_received);
    int16_t recv = 0;
    while (num_bytes_read-- > 0) {
        recv = uart->read();
        if (recv < 0) {
            break;  // Nothing to read
        }
        const uint8_t received_char = recv;
        recv_frame.raw[num_bytes_in_block_received] = received_char;
        switch (num_bytes_in_block_received)
        {
        case 0: // Preamble
            // Make sure the first byte is the preamble
            if (received_char == KONEXUWB_PREABLE) {
                num_bytes_in_block_received++;
            }
            break;
        case 1: // Check header validity
            if (received_char < static_cast<uint8_t>(RECV_UNKOWN)) {
                parse_state = (ParseState)received_char;
                num_bytes_in_block_received++;
                switch (received_char)
                {
                case RECV_ADD_ANCHOR:
                    expected_frame_size = sizeof(KonexUWBFrame::AddAnchor);
                    break;
                case RECV_REMOVE_ANCHOR:
                    expected_frame_size = sizeof(KonexUWBFrame::RemoveAnchor);
                    break;
                case RECV_SAMPLE:
                    expected_frame_size = sizeof(KonexUWBFrame::Sample);
                    break;
                case RECV_RANGE_SAMPLE:
                    expected_frame_size = sizeof(KonexUWBFrame::RangeSample);
                    break;
                }
            } else {
                parse_state = RECV_UNKOWN;
                expected_frame_size = 0;
                num_bytes_in_block_received = 0;
            }
            break;
        default:
            if (num_bytes_in_block_received+1 >= expected_frame_size) {
                // TODO: Check crc
                switch (parse_state)
                {
                case RECV_ADD_ANCHOR:
                    process_add_beacon_frame();
                    break;
                case RECV_REMOVE_ANCHOR:
                    process_remove_beacon_frame();
                    break;
                case RECV_SAMPLE:
                    process_sample_frame();
                    break;
                case RECV_RANGE_SAMPLE:
                    process_range_frame();
                default:
                    break;
                }
                parse_state = RECV_UNKOWN;
                expected_frame_size = 0;
                num_bytes_in_block_received = 0;
                last_update_ms = AP_HAL::millis();      // record success 
            } else {
                num_bytes_in_block_received++;
            }
            break;
        }
    }



}

void AP_Beacon_KonexUWB::process_add_beacon_frame()
{
    // For now 
    uint8_t anchor_id = recv_frame.add_frame.anchor_id % AP_BEACON_MAX_BEACONS;

    beacon_position_NED__m[anchor_id] = Vector3f(recv_frame.add_frame.location.x__m,
                                recv_frame.add_frame.location.y__m,
                                recv_frame.add_frame.location.z__m);  
    
    // Set the beacon position to the frontend
    set_beacon_position(anchor_id, beacon_position_NED__m[anchor_id]);
    
    backend_beacon_count = MIN(backend_beacon_count + 1, static_cast<uint32_t>(AP_BEACON_MAX_BEACONS));
    if (backend_beacon_count < AP_BEACON_MAX_BEACONS) {
        beacon_positions_initialized = false;
    } else {
        beacon_positions_initialized = true;
    }

    // Respond with ack
    uart->write((uint8_t)RECV_ADD_ANCHOR);
}

void AP_Beacon_KonexUWB::process_remove_beacon_frame()
{
    // Remove the beacon from the frontend
    // set_beacon_position(recv_frame.remove_frame.anchor_id, Vector3f(0.0f, 0.0f, 0.0f));
    // unimplemented for now
    // backend_beacon_count--;
    if (uart == nullptr) {
        return;
    }
    // Respond with ack
    uart->write((uint8_t)RECV_REMOVE_ANCHOR);
}

void AP_Beacon_KonexUWB::process_sample_frame()
{
    // cache the vehicle position in NED coordinates [m]
    vehicle_position_NED__m = Vector3f(recv_frame.sample.tag_loc.x__m,
                                       recv_frame.sample.tag_loc.y__m,
                                       recv_frame.sample.tag_loc.z__m);

    // Set the vehicle position
    set_vehicle_position(vehicle_position_NED__m , static_cast<float>(recv_frame.sample.error__mm)/1000.0f);
}

void AP_Beacon_KonexUWB::process_range_frame()
{
    if (beacon_positions_initialized) {
        set_beacon_distance(recv_frame.range_sample.anchor_id, recv_frame.range_sample.range__m);
    }
}

#endif  // AP_BEACON_KONEXUWB_ENABLED
