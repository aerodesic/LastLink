/*
 * duplicate_packets.c
 *
 * Detect duplicate packets with a sequence number cache by origin address.
 */
#include "esp_log.h"

#include "duplicate_packets.h"
#include "listops.h"
#include "linklayer.h"

#define TAG "dupseq"

bool is_duplicate_packet(duplicate_packet_list_t* duplist, const packet_t* packet)
{
    bool duplicate = false;

    int origin   = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
    int sequence = get_int_field(packet, HEADER_SEQUENCE_NUMBER, SEQUENCE_NUMBER_LEN);
    int hash     = origin % NUM_ORIGIN_MAP;
    
    list_head_t *list_head = &duplist->origin_map[hash];

    bool found = false;

    duplicate_sequence_t *dupinfo = (duplicate_sequence_t *) FIRST_LIST_ITEM(list_head);

    while (!found && dupinfo != NULL) {
        if (dupinfo->origin == origin) {
            /* Return true if duplicate */
            duplicate = (dupinfo->sequence - sequence) >= 0;
            if (!duplicate) {
                /* Remember the new one now */
                dupinfo->sequence = sequence;
            }
            /* End the search when we find an address match */
            found = true;
        }
        dupinfo = NEXT_LIST_ITEM(dupinfo, list_head);
    }

    if (!found) {
        /* Add a node */
        duplicate_sequence_t *dupinfo = (duplicate_sequence_t*) malloc(sizeof(duplicate_sequence_t));
        if (dupinfo != NULL) {
            dupinfo->origin = origin;
            dupinfo->sequence = sequence;
            ADD_TO_LIST(list_head, dupinfo);
        }
    }

//if (duplicate) {
//    ESP_LOGI(TAG, "%s: %d duplicate on node %d", __func__, sequence, origin);
//}

    return duplicate;
}

