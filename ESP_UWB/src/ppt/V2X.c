
void send_vehicle_status(void)
{
    vehicle_status_packet_t packet;

    packet.vehicle_id = SELF_VEHICLE_ID;
    packet.uwb_id = SELF_UWB_ID;

    packet.platoon_id = SELF_PLATOON_ID;
    packet.platoon_enable = 1;
    packet.platoon_role = SELF_PLATOON_ROLE;
    packet.platoon_index = SELF_PLATOON_INDEX;

    packet.speed_mps = current_speed;
    packet.heading_deg = current_heading;

    packet.current_checkpoint = current_cp;
    packet.destination_checkpoint = destination_cp;
    packet.route_len = route_len;
    memcpy(packet.route, route, route_len);

    packet.timestamp_ms = millis();
    packet.seq = tx_seq++;

    esp_now_send(broadcast_mac,
                 (uint8_t *)&packet,
                 sizeof(packet));
}


void match_uwb_result(const uwb_result_t *uwb)
{
    for (int i = 0; i < MAX_VEHICLES; i++) {
        if (!vehicle_table[i].valid)
            continue;

        if (vehicle_table[i].uwb_id == uwb->target_uwb_id) {
            vehicle_table[i].distance_m = uwb->distance_m;
            vehicle_table[i].angle_deg = uwb->angle_deg;

            float rad = uwb->angle_deg * M_PI / 180.0f;

            vehicle_table[i].rel_x_m =
                uwb->distance_m * cosf(rad);

            vehicle_table[i].rel_y_m =
                uwb->distance_m * sinf(rad);

            vehicle_table[i].last_uwb_ms = millis();
            break;
        }
    }
}