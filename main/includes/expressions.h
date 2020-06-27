/*
 * expression.h
 *
 */
#ifndef __expressions_h_included
#define __expressions_h_included

#define MAX_CONSTANT_STRING    128

#include "varlist.h"

typedef var_item_t* (*get_var_value_t)(void *context, const char *name);
    
typedef enum {
    EVERR_NONE = 0,
    EVERR_ATOM,
    EVERR_DIV0,
    EVERR_PARENS,
    EVERR_UNDEFINED,
    EVERR_TYPE,
    EVERR_STRING,
    EVERR_WRONG_ARG_COUNT,
} eval_error_t;

/*****************************************************************************
 Value cell
 *****************************************************************************/
typedef struct eval_value {
    enum { VT_VOID = 0, VT_INT, VT_STR }  type;
    union {
        const char* string;
        int         integer;
    };
} eval_value_t;

void set_int_value(eval_value_t *value, int integer);
void set_str_value(eval_value_t *value, const char* string);

/*****************************************************************************
 Built-in Functions
 *****************************************************************************/
/*
 * The parameter list
 */
typedef struct function_parameter function_parameter_t;

typedef struct function_parameter {
    function_parameter_t   *next;
    function_parameter_t   *prev;
    eval_value_t           value;
} function_parameter_t;
    
typedef list_head_t  function_parameter_list_t;

/*
 * The built-in function definitions
 */
typedef eval_error_t(*built_in_function_call_t)(eval_value_t* value, void* context, function_parameter_list_t *parameters);

typedef struct built_in_function built_in_function_t;  // FWD

typedef struct built_in_function {
    built_in_function_t       *next;
    built_in_function_t       *prev;
    built_in_function_call_t  function;
    char                      name[1];
} built_in_function_t;

typedef list_head_t  built_in_functions_t;

bool expr_add_built_in_function(const char* name, built_in_function_call_t function);
void expr_release_built_in_function(built_in_function_t *function);

/*****************************************************************************
 Expression evaluation
 *****************************************************************************/
typedef eval_error_t(*eval_op_t)(eval_value_t*, eval_value_t*);

eval_error_t eval_expression(eval_value_t *value, get_var_value_t get_var_value, void* context, const char **args);

const char *eval_get_expr_value(eval_value_t *value, char *buf, size_t buflen);

#endif /* __expressions_h_included */

