/*
 * duplicate_sequence.c
 *
 * Detect duplicate sequence number cache by address.
 */
#include "sdkconfig.h"

#include "esp_log.h"

#include "duplicate_sequence.h"
#include "listops.h"
#include "linklayer.h"

#define TAG "dupseq"


/*
 * Remove a duplicate so the next serial number is ok.
 */
void reset_duplicate(duplicate_sequence_list_t *duplist, int address)
{
    list_head_t *list_head = &duplist->address_map[address % NUM_ORIGIN_MAP];

    duplicate_sequence_t *dupinfo = (duplicate_sequence_t *) FIRST_LIST_ITEM(list_head);

    while (dupinfo != NULL && dupinfo->address != address) {
        dupinfo = NEXT_LIST_ITEM(dupinfo, list_head);
    }

    /* sequence 0 never matches so we don't actually need remove it from the list */
    if (dupinfo != NULL) {
        dupinfo->sequence = 0;
        dupinfo->count = 0;
    }
}

/*
 * Sequence numbers must be treated within as signed numbers.
 */
bool is_duplicate(duplicate_sequence_list_t* duplist, int address, int sequence)
{
    list_head_t *list_head = &duplist->address_map[address % NUM_ORIGIN_MAP];

    bool found = false;

    bool duplicate = false;

    duplicate_sequence_t *dupinfo = (duplicate_sequence_t *) FIRST_LIST_ITEM(list_head);


    while (!found && dupinfo != NULL) {
        if (dupinfo->address == address) {

            /* We will stop the search here. */
            found = true;

            duplicate = (dupinfo->address != 0) &&
                        (((sequence <= dupinfo->sequence) && ((dupinfo->sequence - sequence) < (1 << (SEQUENCE_NUMBER_LEN*8-1)))) ||
                         ((sequence >  dupinfo->sequence) && ((sequence - dupinfo->sequence) > (1 << (SEQUENCE_NUMBER_LEN*8-1)))));


            if (!duplicate || dupinfo->count >= CONFIG_LASTLINK_MAX_SEQUENCE_DUPLICATE_COUNT) {
                /* Remember the new one now */
                dupinfo->sequence = sequence;
                dupinfo->count = 0;
                duplicate = false;
            } else {
                /* We count the occurrences of duplicate packets for this address.  If we are below the limit,
                 * it was duplicate (caller will probably ignore the packet) and we advance the duplicate count.
                 * If we end up hitting the duplicate count limit, pretend it is no longer duplicate.
                 */
                dupinfo->count++;
            }
        }

        dupinfo = NEXT_LIST_ITEM(dupinfo, list_head);
    }

    if (!found) {
        /* First packet seen from this node.  Remember it's sequence number */
        duplicate_sequence_t *dupinfo = (duplicate_sequence_t*) malloc(sizeof(duplicate_sequence_t));
        if (dupinfo != NULL) {
            dupinfo->address = address;
            dupinfo->sequence = sequence;
            dupinfo->count = 0;
            ADD_TO_LIST(list_head, dupinfo);
        }
    }

    return duplicate;
}

