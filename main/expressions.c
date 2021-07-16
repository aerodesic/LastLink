/*
 * expression.c
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/param.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "listops.h"
#include "varlist.h"

#include "expressions.h"

/*
 * Operatons on values
 */

void set_none_value(eval_value_t *value)
{
    if (value->type == VT_STR) {
        free((void*) value->string);
    }
    value->type = VT_NONE;
}

void set_undefined_value(eval_value_t *value)
{
    set_none_value(value);
    value->type = VT_UNDEFINED;
}

void set_int_value(eval_value_t *value, int integer)
{
    set_none_value(value);

    value->integer = integer;
    value->type = VT_INT;
}

void set_str_value(eval_value_t *value, const char *string)
{
    /* Don't release value if setting to a string of current value */
    if (value->type != VT_STR || strcmp(value->string, string) != 0) {
        if (value->type == VT_STR) {
            free((void*) value->string);
        }

        value->type = VT_STR;
        value->string = strdup(string);
    }
}

/*
 * Built-in function definitions
 */
static built_in_functions_t  built_in_functions;

static built_in_function_t *find_built_in_function(const char *name)
{
    built_in_function_t *bifunction = (built_in_function_t*) FIRST_LIST_ITEM(&built_in_functions);

    built_in_function_t *found = NULL;

    while (!found && bifunction != NULL) {
        if (strcmp(bifunction->name, name) == 0) {
            found = bifunction;
        } else {
            bifunction = NEXT_LIST_ITEM(bifunction, &built_in_functions);
        }
    }

    return found;
}

bool expr_add_built_in_function(const char *name, built_in_function_call_t function)
{
    built_in_function_t *bifunction = find_built_in_function(name);

    if (bifunction != NULL) {
        bifunction = NULL;  /* Failure */
    } else {
        bifunction = (built_in_function_t*) malloc(sizeof(built_in_function_t) + strlen(name));
        if (bifunction != NULL) {
            strcpy(bifunction->name, name);
            bifunction->function = function;
            ADD_TO_LIST(&built_in_functions, bifunction);
        }
    }

    return function != NULL;
}

void expr_remove_built_in_function(const char *name)
{
    built_in_function_t *function = find_built_in_function(name);
    if (function != NULL) {
        REMOVE_FROM_LIST(&built_in_functions, function);
        free((void*) function);
    }
}

/*
 * Expression evaluation
 */
typedef eval_error_t (*eval_op_t)(eval_value_t* val1, eval_value_t *val2);

/*
 * len() built-in function
 */
eval_error_t built_in_function_len(eval_value_t *value, void *context, function_parameter_list_t *parameters)
{
    eval_error_t error = EVERR_NONE;

    if (NUM_IN_LIST(parameters) != 1) {
        error = EVERR_WRONG_ARG_COUNT;
    } else {
        function_parameter_t *param = (function_parameter_t *) FIRST_LIST_ITEM(parameters);
        if (param->value.type != VT_STR) {
            error = EVERR_NOT_STRING;
        } else {
            set_int_value(value, strlen(param->value.string));
        }
    }
    return error;
}

/*
 * defined() built-in function
 *
 * defined(variable_name)
 */
eval_error_t built_in_function_defined(eval_value_t *value, void *context, function_parameter_list_t *parameters)
{
    eval_error_t error = EVERR_NONE;

    if (NUM_IN_LIST(parameters) != 1) {
        error = EVERR_WRONG_ARG_COUNT;
    } else {
        function_parameter_t *param = (function_parameter_t *) FIRST_LIST_ITEM(parameters);
        /* Lookup the symbol and set value to true resolved to value */
        set_int_value(value, param->value.type != VT_UNDEFINED);
printf("%s: defined function parameter type is %d\n", __func__, param->value.type);
        error = EVERR_NONE;
    }

    return error;
}


/*
 * Monadic operators
 */
eval_error_t eval_op_not(eval_value_t *value)
{
    eval_error_t error = EVERR_NONE;

    if (value->type == VT_STR) {
        set_int_value(value, strlen(value->string) != 0);
    } else {
        value->integer = !value->integer;
    }

    return error;
}

eval_error_t eval_op_neg(eval_value_t *value)
{
    eval_error_t error = EVERR_NONE;

    if (value->type == VT_INT) {
        value->integer = -value->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }
    return error;
}

eval_error_t eval_op_comp(eval_value_t *value)
{
    eval_error_t error = EVERR_NONE;

    if (value->type == VT_INT) {
        value->integer = ~value->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

/*
 * Dyadic operators.
 */
static eval_error_t eval_op_add(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer += val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        /* Concatenate */
        char buffer[strlen(val1->string) + strlen(val2->string) + 1];
        strcpy(buffer, val1->string);
        strcat(buffer, val2->string);
        set_str_value(val1, buffer);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_subtract(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer -= val2->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

static eval_error_t eval_op_multiply(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer *= val2->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

static eval_error_t eval_op_divide(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        if (val2->integer != 0) {
            val1->integer /= val2->integer;
        } else {
            error = EVERR_DIV0;
        }
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

static eval_error_t eval_op_bitor(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer |= val2->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

static eval_error_t eval_op_bitand(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer &= val2->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

static eval_error_t eval_op_bitxor(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer ^= val2->integer;
    } else {
        error = EVERR_NOT_INTEGER;
    }

    return error;
}

static eval_error_t eval_op_logor(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer || val2->integer;
    } else if (val1->type == VT_INT && val2->type == VT_STR) {
        val1->integer = val1->integer || strlen(val2->string);
    } else if (val1->type == VT_STR && val2->type == VT_INT) {
        set_int_value(val1, strlen(val1->string) || val2->integer);
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strlen(val1->string) || strlen(val2->string));
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_logand(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer && val2->integer;
    } else if (val1->type == VT_INT && val2->type == VT_STR) {
        val1->integer = val1->integer && strlen(val2->string);
    } else if (val1->type == VT_STR && val2->type == VT_INT) {
        set_int_value(val1, strlen(val1->string) && val2->integer);
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strlen(val1->string) && strlen(val2->string));
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_equal(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer == val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strcmp(val1->string, val2->string) == 0);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_notequal(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer == val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strcmp(val1->string, val2->string) != 0);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_lessequal(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer <= val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strcmp(val1->string, val2->string) <= 0);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_greaterequal(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer >= val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strcmp(val1->string, val2->string) >= 0);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_less(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer < val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strcmp(val1->string, val2->string) < 0);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static eval_error_t eval_op_greater(eval_value_t *val1, eval_value_t *val2)
{
    eval_error_t error = EVERR_NONE;

    if (val1->type == VT_INT && val2->type == VT_INT) {
        val1->integer = val1->integer > val2->integer;
    } else if (val1->type == VT_STR && val2->type == VT_STR) {
        set_int_value(val1, strcmp(val1->string, val2->string) > 0);
    } else {
        error = EVERR_TYPE;
    }

    return error;
}

static bool eval_is_next(const char **args, const char *str)
{
    while (isspace(**args)) {
        ++(*args);
    }

    size_t len = strlen(str);

    bool is_next = strncmp(str, *args, len) == 0;

    /* Skip over item if we matched */
    if (is_next) {
        *(args) += len;
    }

    return is_next;
}

static eval_op_t eval_operator(const char **args, ...)
{
    va_list ap;
    va_start(ap, args);

    /* Skip leading spaces */
    while (isspace(**args)) {
       ++(*args);
    }

    const char *op;

    eval_op_t found = NULL;

    do {
        op = va_arg(ap, const char*);
        if (op != NULL) {
            eval_op_t action = va_arg(ap, eval_op_t);
            if (eval_is_next(args, op)) {
                found = action;
            }
        }
    } while (found == NULL && op != NULL);

    va_end(ap);

//printf("%s: found '%s'\n", __func__, op);

    return found;
}

static eval_error_t eval_number(eval_value_t *value, const char **args)
{
    eval_error_t error = EVERR_NONE;

    char *endp;
    set_int_value(value, strtol(*args, &endp, 0));
    *args = endp;

//printf("eval_number %d at '%s'\n", value->integer, *args);

    return error;
}

/*
 * Pick up two hex chars and return value.
 */
static int hex_char(const char **args)
{
   int ch1 = *++(*args);
   int ch2 = *++(*args);

//printf("%s: ch1 %02x ch2 %02x\n", __func__, ch1, ch2);

   ch1 = tolower(ch1);
   ch2 = tolower(ch2);

   ch1 = (ch1 >= '0' && ch1 <= '9') ? ch1 - '0' : ch1 - 'a' + 10;
   ch2 = (ch2 >= '0' && ch2 <= '9') ? ch2 - '0' : ch2 - 'a' + 10;

   return (ch1 << 4) + ch2;
}

static int oct_char(const char **args)
{
   int ch1 = *++(*args) & 0x07;
   int ch2 = *++(*args) & 0x07;
   int ch3 = *++(*args) & 0x07;

   return (ch1 << 6) + (ch2 << 3) + ch3;
}

static eval_error_t eval_string(eval_value_t *value, const char **args)
{
    eval_error_t error = EVERR_NONE;

    /* Parse a string with \ escapes */
    int pos = 0;
    char buffer[MAX_CONSTANT_STRING+1];

    while (**args != '\0' && **args != '\"') {
        int ch;

        if (**args == '\\') {
            ++(*args);
            if (**args == 'r') {
                ch = '\r';
            } else if (**args == 'n') {
                ch = '\n';
            } else if (**args == 't') {
                ch = '\t';
            } else if (**args == 'b') {
                ch = '\b';
            } else if (**args == 'f') {
                ch = '\f';
            } else if (**args == 'v') {
                ch = '\v';
            } else if (**args == 'a') {
                ch = '\a';
            } else if (**args == 'x') {
                /* Hex constant */
                ch = hex_char(args);
            } else if (**args >= '0' && **args <= '7') {
                /* Octal constant */
                ch = oct_char(args);
            } else {
                /* Character */
                ch = **args;
            }
        } else {
            ch = **args;
        }

        if (pos < MAX_CONSTANT_STRING) {
            buffer[pos++] = ch;
        }

        ++(*args);
    }

    buffer[pos] = '\0';

    if (eval_is_next(args, "\"")) {
        set_str_value(value, buffer);
    } else {
        error = EVERR_STRING;
    }

    return error;
}

static eval_error_t eval_symbol(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    eval_error_t error = EVERR_NONE;

    char buffer[MAX_CONSTANT_STRING+1];
    int pos = 0;

    while (isalnum(**args)) {
        if (pos < MAX_CONSTANT_STRING) {
            buffer[pos++] = **args;
        }
        ++(*args);
    }

    buffer[pos] = '\0';

    /* First check built-in functions */
    built_in_function_t *function = find_built_in_function(buffer);
    if (function != NULL) {
        if (eval_is_next(args, "(")) {
            /* Process parameters */
            function_parameter_list_t parameters;
            INIT_LIST(&parameters);

            do {
                /* Use return value as temporary */
                value->type = VT_NONE;
                error = eval_expression(value, get_var_value, context, args);
                /* Tack on the new parameter */
                function_parameter_t *parameter = (function_parameter_t *) malloc(sizeof(function_parameter_t));
                parameter->value = *value;
                value->type = VT_NONE;
                ADD_TO_LIST(&parameters, parameter);
            } while ((error == EVERR_NONE || error == EVERR_UNDEFINED) && eval_is_next(args, ","));

            if ((error == EVERR_NONE) || (error == EVERR_UNDEFINED)) {
                if (eval_is_next(args, ")")) {
                     error = function->function(value, context, &parameters);
                }
            }

            while (NUM_IN_LIST(&parameters) != 0) {
                function_parameter_t *param = (function_parameter_t *) FIRST_LIST_ITEM(&parameters);
                REMOVE_FROM_LIST(&parameters, param);
                /* Release any space in use */
                set_none_value(&param->value);
            }
        }
    } else {
        /* Look up the symbol */
        var_item_t *var = get_var_value(context, buffer);
        if (var != NULL) {
            set_str_value(value, var->value);
        } else {
            set_undefined_value(value);
        }
    }

    return error;
}


static eval_error_t eval_atom(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    eval_error_t error = EVERR_NONE;

    if (eval_is_next(args, "(")) {
        error = eval_expression(value, get_var_value, context, args);
        if (!eval_is_next(args, ")")) {
            error = EVERR_PARENS;
        }

    } else if (eval_is_next(args, "!")) {
        error = eval_atom(value, get_var_value, context, args);
        if (error == EVERR_NONE) {
            error = eval_op_not(value);
        }
    } else if (eval_is_next(args, "-")) {
        error = eval_atom(value, get_var_value, context, args);
        if (error == EVERR_NONE) {
            error = eval_op_neg(value);
        }
    } else if (eval_is_next(args, "~")) {
        error = eval_atom(value, get_var_value, context, args);
        if (error == EVERR_NONE) {
            error = eval_op_comp(value);
        }
    } else if (isalpha(**args)) {
        error = eval_symbol(value, get_var_value, context, args);

    } else if (isdigit(**args)) {
        error = eval_number(value, args);

    } else if (eval_is_next(args, "\"")) {
        error = eval_string(value, args);

    } else {
        error = EVERR_ATOM;
    }

    return error;
}

static int eval_relational(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    eval_error_t error = eval_atom(value, get_var_value, context, args);
    if (error == EVERR_NONE) {
        eval_op_t op;

        do {
           op = eval_operator(args, "==", eval_op_equal, "!=", eval_op_notequal, ">=", eval_op_greaterequal, "<=", eval_op_lessequal, ">", eval_op_greater, "<", eval_op_less, NULL);

            if (op != NULL) {
                eval_value_t value2;
                error = eval_atom(&value2, get_var_value, context, args);
                if (error == EVERR_NONE) {
                    error = op(value, &value2);
                }
            }
        } while(error == EVERR_NONE && op != NULL);
    }

    return error;
}

static int eval_factor(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    eval_error_t error = eval_relational(value, get_var_value, context, args);
    if (error == EVERR_NONE) {
        eval_op_t op;

        do {
            op = eval_operator(args, "/", eval_op_divide, "*", eval_op_multiply, NULL);

            if (op != NULL) {
                eval_value_t value2;
                error = eval_relational(&value2, get_var_value, context, args);
                if (error == EVERR_NONE) {
                    error = op(value, &value2);
                }
            }
        } while(error == EVERR_NONE && op != NULL);
    }

    return error;
}

static int eval_sums(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    eval_error_t error = eval_factor(value, get_var_value, context, args);
    if (error == EVERR_NONE) {
        eval_op_t op;

        do {
            op = eval_operator(args, "+", eval_op_add, "-", eval_op_subtract, NULL);

            if (op != NULL) {
                eval_value_t value2;
                error = eval_factor(&value2, get_var_value, context, args);
                if (error == EVERR_NONE) {
                    error = op(value, &value2);
                }
            }
        } while(error == EVERR_NONE && op != NULL);
    }

    return error;
}

static eval_error_t eval_logical(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    eval_error_t error = eval_sums(value, get_var_value, context, args);

    if (error == EVERR_NONE) {
        eval_op_t op;

        do {
            op = eval_operator(args, "||", eval_op_logor, "|", eval_op_bitor, "&&", eval_op_logand, "&", eval_op_bitand, "^", eval_op_bitxor, NULL);

            if (op != NULL) {
                eval_value_t value2;
                error = eval_sums(&value2, get_var_value, context, args);
                if (error == EVERR_NONE) {
                    error = op(value, &value2);
                }
            }
        } while(error == EVERR_NONE && op != NULL);
    }

    return error;
}

eval_error_t eval_expression(eval_value_t *value, get_var_value_t get_var_value, void *context, const char **args)
{
    if (NUM_IN_LIST(&built_in_functions) == 0) {
        expr_add_built_in_function("len",     built_in_function_len);
        expr_add_built_in_function("defined", built_in_function_defined);
    }

    return eval_logical(value, get_var_value, context, args);
}

/*
 * Return string with value
 */
const char *eval_get_expr_value(eval_value_t *value, char *buf, size_t buflen)
{
    if (value->type == VT_STR) {
        snprintf(buf, buflen, "String \"%s\"", value->string);
    } else if (value->type == VT_INT) {
        snprintf(buf, buflen, "Integer %d", value->integer);
    } else {
        snprintf(buf, buflen, "Undefined type %d", value->type);
    }

    return buf;
}


const char *eval_error(eval_error_t error)
{
    switch (error) {
        case EVERR_NONE:              return "None";
        case EVERR_ATOM:              return "Unknown atom";
        case EVERR_DIV0:              return "Divide by 0";
        case EVERR_PARENS:            return "Mismatched ()";
        case EVERR_UNDEFINED:         return "Undefined";
        case EVERR_TYPE:              return "Mismatched types";
        case EVERR_NOT_STRING:        return "Not a string";
        case EVERR_NOT_INTEGER:       return "Not an integer";
        case EVERR_STRING:            return "Missing string terminator";
        case EVERR_WRONG_ARG_COUNT:   return "Wrong arg count";
        default:                      return "UNKNOWN";
    }
}

#ifdef TESTING
var_list_t var_list;

var_item_t *get_var_value(void* context, const char *name)
{
    return find_var(&var_list, name);
}

int main(int argc, char **argv)
{
    char buffer[80];

    INIT_LIST(&var_list);
    set_var(&var_list, "test", "test123");
    set_var(&var_list, "zot", "zoster");
    set_var(&var_list, "digits", "100");

    while (1) {
        printf("? ");
        fgets(buffer, sizeof(buffer), stdin);
        char *p = strchr(buffer, '\n');
        if (p != NULL) {
            *p = '\0';
        }

        eval_value_t value = { .type = VT_NONE };
        const char *bufp = buffer;

        eval_error_t error = eval_expression(&value, get_var_value, NULL, &bufp);

        if (error != EVERR_NONE) {
            printf("Error \"%s\" Remaining \"%s\"\n", eval_error(error), bufp);
        } else if (value.type == VT_INT) {
            printf("Integer %d Remaining \"%s\"\n", value.integer, bufp);
        } else if (value.type == VT_STR) {
            printf("String \"%s\" Remaining \"%s\"\n", value.string, bufp);
        } else {
            printf("Undefined type  %d Remaining \"%s\"\n", value.type, bufp);
        }
    }
}
#endif /* TESTING */
