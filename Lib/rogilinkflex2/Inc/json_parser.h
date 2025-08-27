#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define JSON_MAX_TOKENS 128
#define JSON_MAX_STRING_LENGTH 64

// JSON token types
typedef enum {
    JSON_UNDEFINED = 0,
    JSON_OBJECT = 1,
    JSON_ARRAY = 2,
    JSON_STRING = 3,
    JSON_PRIMITIVE = 4
} json_type_t;

// JSON token structure
typedef struct {
    json_type_t type;
    int start;
    int end;
    int size;  // Number of children (for objects/arrays)
    int parent;
} json_token_t;

// JSON parser state
typedef struct {
    unsigned int pos;
    unsigned int toknext;
    int toksuper;
} json_parser_t;

// Parser functions
void json_init(json_parser_t *parser);
int json_parse(json_parser_t *parser, const char *js, size_t len,
               json_token_t *tokens, unsigned int num_tokens);

// Token utility functions
json_token_t* json_find_token(const char *js, json_token_t *tokens, 
                              int num_tokens, const char *key);
int json_get_string(const char *js, json_token_t *token, char *dst, size_t size);
int json_get_int(const char *js, json_token_t *token);
float json_get_float(const char *js, json_token_t *token);
bool json_get_bool(const char *js, json_token_t *token);

// JSON builder functions
void json_build_object_start(char *buffer, size_t buffer_size, int *pos);
void json_build_object_end(char *buffer, size_t buffer_size, int *pos);
void json_build_array_start(char *buffer, size_t buffer_size, int *pos, const char *key);
void json_build_array_end(char *buffer, size_t buffer_size, int *pos);
void json_build_string(char *buffer, size_t buffer_size, int *pos, 
                      const char *key, const char *value);
void json_build_int(char *buffer, size_t buffer_size, int *pos, 
                   const char *key, int value);
void json_build_float(char *buffer, size_t buffer_size, int *pos, 
                     const char *key, float value);
void json_build_bool(char *buffer, size_t buffer_size, int *pos, 
                    const char *key, bool value);

#ifdef __cplusplus
}
#endif