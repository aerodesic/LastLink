/*
 * listops.h
 *
 * Operations on doubly-linked lists.
 */
#ifndef __listops_h_included
#define __listops_h_included

typedef struct list_element list_element_t;

typedef struct list_element {
    list_element_t *next;
    list_element_t *prev;
} list_element_t;

typedef struct list_head {
    list_element_t *list;
    int            count;
} list_head_t;

#define INIT_LIST(listhead)  do { (listhead)->list = NULL; (listhead)->count = 0; } while(0)

#define ADD_TO_LIST(listhead, element) \
    do { \
        if ((listhead)->count != 0) { \
            *((list_element_t**) &((element)->next))  = (listhead)->list; \
            *((list_element_t**) &((element)->prev))  = (listhead)->list->prev; \
            ((listhead)->list)->prev->next            = (list_element_t*) (element); \
            ((listhead)->list)->prev                  = (list_element_t*) (element); \
        } else { \
            (listhead)->list                          = (list_element_t*) (element); \
            *((list_element_t**) &((element)->next))  = (list_element_t*) (element); \
            *((list_element_t**) &((element)->prev))  = (list_element_t*) (element); \
        } \
        (listhead)->count++; \
    } while(0);

#define REMOVE_FROM_LIST(listhead, element) \
    do { \
        *((list_element_t**) &((element)->prev->next)) = (list_element_t*) (element)->next; \
        *((list_element_t**) &((element)->next->prev)) = (list_element_t*) (element)->prev; \
        if (-- (listhead)->count == 0) { \
            (listhead)->list = NULL; \
        } else if ((listhead)->list == (list_element_t*) (element)) { \
            (listhead)->list = (listhead)->list->next; \
        } \
    } while(0);


#define HEAD_OF_LIST(listhead) ((listhead)->list)

#define NUM_IN_LIST(listhead) ((listhead)->count)

#define IS_LIST_EMPTY(listhead) (NUM_IN_LIST(listhead) == 0)

#endif /* __listops_h_included */

