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

#define INIT_LIST(__listhead)  do { (__listhead)->list = NULL; (__listhead)->count = 0; } while(0)

#if 1
#define ADD_TO_LIST(__listhead, __element) \
    do { \
        if ((__listhead)->count != 0) { \
            ((list_element_t*)(__element))->next        = (__listhead)->list; \
            ((list_element_t*)(__element))->prev        = (__listhead)->list->prev; \
            ((__listhead)->list)->prev->next            = (list_element_t*) (__element); \
            ((__listhead)->list)->prev                  = (list_element_t*) (__element); \
        } else { \
            (__listhead)->list                          = (list_element_t*) (__element); \
            ((list_element_t*)(__element))->next        = (list_element_t*) (__element); \
            ((list_element_t*)(__element))->prev        = (list_element_t*) (__element); \
        } \
        (__listhead)->count++; \
    } while(0);
#else
#define ADD_TO_LIST(__listhead, __element) \
    do { \
        if ((__listhead)->count != 0) { \
            *((list_element_t**) &((__element)->next))  = (__listhead)->list; \
            *((list_element_t**) &((__element)->prev))  = (__listhead)->list->prev; \
            ((__listhead)->list)->prev->next            = (list_element_t*) (__element); \
            ((__listhead)->list)->prev                  = (list_element_t*) (__element); \
        } else { \
            (__listhead)->list                          = (list_element_t*) (__element); \
            *((list_element_t**) &((__element)->next))  = (list_element_t*) (__element); \
            *((list_element_t**) &((__element)->prev))  = (list_element_t*) (__element); \
        } \
        (__listhead)->count++; \
    } while(0);
#endif

#if 1
#define REMOVE_FROM_LIST(__listhead, __element) \
    do { \
        ((list_element_t*)(__element))->prev->next       = (list_element_t*) (__element)->next; \
        ((list_element_t*)(__element))->next->prev       = (list_element_t*) (__element)->prev; \
        if (-- (__listhead)->count == 0) { \
            (__listhead)->list = NULL; \
        } else if ((__listhead)->list == (list_element_t*) (__element)) { \
            (__listhead)->list = (__listhead)->list->next; \
        } \
    } while(0);
#else
#define REMOVE_FROM_LIST(__listhead, __element) \
    do { \
        *((list_element_t**) &((__element)->prev->next)) = (list_element_t*) (__element)->next; \
        *((list_element_t**) &((__element)->next->prev)) = (list_element_t*) (__element)->prev; \
        if (-- (__listhead)->count == 0) { \
            (__listhead)->list = NULL; \
        } else if ((__listhead)->list == (list_element_t*) (__element)) { \
            (__listhead)->list = (__listhead)->list->next; \
        } \
    } while(0);
#endif

#define FIRST_LIST_ITEM(__listhead) ((__listhead)->list)

#define LAST_LIST_ITEM(__listhead) ((__listhead)->list->next)

#define NUM_IN_LIST(__listhead) ((__listhead)->count)

#define IS_LIST_EMPTY(__listhead) (NUM_IN_LIST(__listhead) == 0)

#define NEXT_LIST_ITEM(__var, __listhead)  ((__var && (list_element_t*) __var->next != FIRST_LIST_ITEM(__listhead)) ? __var->next : NULL)

#define PREV_LIST_ITEM(__var, __listhead)  ((__var && (list_element_t*) __var->prev != FIRST_LIST_ITEM(__listhead)) ? __var->prev : NULL)

#endif /* __listops_h_included */

