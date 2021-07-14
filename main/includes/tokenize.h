/*
 * tokenize.h
 *
 */
#ifndef __tokenize_h_included
#define __tokenize_h_included

int tokenize(char *buffer, const char**args, int maxargs);
char* strstrip(char *buffer);
char* skip_blanks(char* bufp);

#endif /* __tokenize_h_included */

