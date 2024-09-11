/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 Adapted into Ardupilot by Marc Espuña
 */

#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_KONEXUWB_ENABLED

/**
 * @class AP_Beacon_KonexUWB
 * @brief Konex UWB beacon backend
 * @details We will make it work with 400k baudrate
 */
class AP_Beacon_KonexUWB : public AP_Beacon_Backend
{
public:
    using AP_Beacon_Backend::AP_Beacon_Backend;

        // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

private:
    static constexpr uint8_t XYZ_AXIS_COUNT = 3;

    struct RelativeLocation
    {
        float x__m, y__m, z__m;
    }PACKED;

    union KonexUWBFrame 
    {
        struct AddAnchor    // Response with ACK with header value needed
        {
            uint8_t preamble;
            uint8_t header;            
            uint8_t anchor_id;          
            RelativeLocation location;
            uint16_t crc;
        }PACKED;
        struct RemoveAnchor  // Response with ACK with header value needed
        {
            uint8_t preamble;
            uint8_t header;   
            uint8_t anchor_id;          
            uint16_t crc;
        }PACKED;
        struct Sample
        {
            uint8_t preamble;
            uint8_t header;
            RelativeLocation tag_loc;
            uint16_t error__mm;         // Estimated error in mm
            uint16_t crc;
        }PACKED; 
        struct RangeSample
        {
            uint8_t preamble;
            uint8_t header;
            uint8_t anchor_id;
            float range__m;
            uint16_t crc;
        }PACKED;
        AddAnchor add_frame;            // Frame that reports a new anchor position
        RemoveAnchor remove_frame;      // Frame that reports the loss of an anchor connection
        Sample sample;                  // Tag position sample
        RangeSample range_sample;       // Range sample
        uint8_t raw[sizeof(Sample)]; // We use sizeof(AddAnchor) because it is the largest struct
    };

    enum ParseState : uint8_t {
        RECV_ADD_ANCHOR = 0,
        RECV_REMOVE_ANCHOR,
        RECV_SAMPLE,
        RECV_RANGE_SAMPLE,
        RECV_UNKOWN 
    } parse_state = RECV_UNKOWN; // current state of receive data

    KonexUWBFrame recv_frame;
    uint16_t expected_frame_size;
    uint32_t num_bytes_in_block_received = 0;

    // Variables for Ardupilot
    uint32_t last_update_ms;

    // cache the vehicle position in NED coordinates [m]
    Vector3f vehicle_position_NED__m;

    // cache the frontend beacon positions in NED coordinates [m]
    Vector3f beacon_position_NED__m[AP_BEACON_MAX_BEACONS];
    uint32_t backend_beacon_count;
    bool beacon_positions_initialized = false;

private: 
    void process_add_beacon_frame();
    void process_remove_beacon_frame();
    void process_sample_frame();
    void process_range_frame();
};

#endif // AP_BEACON_KONEXUWB_ENABLED