#include "json_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Helper macros
#define JSON_TOKEN_CLEAR(token) do { \
    (token)->type = JSON_UNDEFINED; \
    (token)->start = -1; \
    (token)->end = -1; \
    (token)->size = 0; \
    (token)->parent = -1; \
} while(0)

// Helper functions
static json_token_t *json_alloc_token(json_parser_t *parser,
                                      json_token_t *tokens,
                                      size_t num_tokens) {
    if (parser->toknext >= num_tokens) {
        return NULL;
    }
    json_token_t *tok = &tokens[parser->toknext++];
    JSON_TOKEN_CLEAR(tok);
    return tok;
}

static void json_fill_token(json_token_t *token, json_type_t type,
                           int start, int end) {
    token->type = type;
    token->start = start;
    token->end = end;
    token->size = 0;
}

static int json_parse_primitive(json_parser_t *parser, const char *js,
                               size_t len, json_token_t *tokens,
                               size_t num_tokens) {
    json_token_t *token;
    int start;

    start = parser->pos;

    for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
        switch (js[parser->pos]) {
            case '\t': case '\r': case '\n': case ' ':
            case ',': case ']': case '}':
                goto found;
        }
        if (js[parser->pos] < 32 || js[parser->pos] >= 127) {
            parser->pos = start;
            return -1;
        }
    }

found:
    if (tokens == NULL) {
        parser->pos--;
        return 0;
    }
    token = json_alloc_token(parser, tokens, num_tokens);
    if (token == NULL) {
        parser->pos = start;
        return -1;
    }
    json_fill_token(token, JSON_PRIMITIVE, start, parser->pos);
    parser->pos--;
    return 0;
}

static int json_parse_string(json_parser_t *parser, const char *js,
                            size_t len, json_token_t *tokens,
                            size_t num_tokens) {
    json_token_t *token;
    int start = parser->pos;

    parser->pos++;

    // Skip starting quote
    for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
        char c = js[parser->pos];

        // Quote: end of string
        if (c == '\"') {
            if (tokens == NULL) {
                return 0;
            }
            token = json_alloc_token(parser, tokens, num_tokens);
            if (token == NULL) {
                parser->pos = start;
                return -1;
            }
            json_fill_token(token, JSON_STRING, start + 1, parser->pos);
            return 0;
        }

        // Backslash: Quoted symbol expected
        if (c == '\\' && parser->pos + 1 < len) {
            parser->pos++;
            switch (js[parser->pos]) {
                case '\"': case '/': case '\\': case 'b':
                case 'f': case 'r': case 'n': case 't':
                    break;
                case 'u':
                    parser->pos++;
                    for (int i = 0; i < 4 && parser->pos < len && js[parser->pos] != '\0'; i++) {
                        if (!((js[parser->pos] >= 48 && js[parser->pos] <= 57) ||
                              (js[parser->pos] >= 65 && js[parser->pos] <= 70) ||
                              (js[parser->pos] >= 97 && js[parser->pos] <= 102))) {
                            parser->pos = start;
                            return -1;
                        }
                        parser->pos++;
                    }
                    parser->pos--;
                    break;
                default:
                    parser->pos = start;
                    return -1;
            }
        }
    }
    parser->pos = start;
    return -1;
}

void json_init(json_parser_t *parser) {
    parser->pos = 0;
    parser->toknext = 0;
    parser->toksuper = -1;
}

int json_parse(json_parser_t *parser, const char *js, size_t len,
               json_token_t *tokens, unsigned int num_tokens) {
    int r;
    int i;
    json_token_t *token;
    int count = parser->toknext;

    for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
        char c;
        json_type_t type;

        c = js[parser->pos];
        switch (c) {
            case '{': case '[':
                count++;
                if (tokens == NULL) {
                    break;
                }
                token = json_alloc_token(parser, tokens, num_tokens);
                if (token == NULL) {
                    return -1;
                }
                if (parser->toksuper != -1) {
                    json_token_t *t = &tokens[parser->toksuper];
                    t->size++;
                    token->parent = parser->toksuper;
                }
                token->type = (c == '{' ? JSON_OBJECT : JSON_ARRAY);
                token->start = parser->pos;
                parser->toksuper = parser->toknext - 1;
                break;
            case '}': case ']':
                if (tokens == NULL) {
                    break;
                }
                type = (c == '}' ? JSON_OBJECT : JSON_ARRAY);
                if (parser->toknext < 1) {
                    return -1;
                }
                token = &tokens[parser->toknext - 1];
                for (;;) {
                    if (token->start != -1 && token->end == -1) {
                        if (token->type != type) {
                            return -1;
                        }
                        token->end = parser->pos + 1;
                        parser->toksuper = token->parent;
                        break;
                    }
                    if (token->parent == -1) {
                        if (token->type != type || parser->toksuper == -1) {
                            return -1;
                        }
                        break;
                    }
                    token = &tokens[token->parent];
                }
                break;
            case '\"':
                r = json_parse_string(parser, js, len, tokens, num_tokens);
                if (r < 0) {
                    return r;
                }
                count++;
                if (parser->toksuper != -1 && tokens != NULL) {
                    tokens[parser->toksuper].size++;
                }
                break;
            case '\t': case '\r': case '\n': case ' ':
                break;
            case ':':
                parser->toksuper = parser->toknext - 1;
                break;
            case ',':
                if (tokens != NULL && parser->toksuper != -1 &&
                    tokens[parser->toksuper].type != JSON_ARRAY &&
                    tokens[parser->toksuper].type != JSON_OBJECT) {
                    parser->toksuper = tokens[parser->toksuper].parent;
                }
                break;
            default:
                r = json_parse_primitive(parser, js, len, tokens, num_tokens);
                if (r < 0) {
                    return r;
                }
                count++;
                if (parser->toksuper != -1 && tokens != NULL) {
                    tokens[parser->toksuper].size++;
                }
                break;
        }
    }

    if (tokens == NULL) {
        return count;
    }

    for (i = parser->toknext - 1; i >= 0; i--) {
        if (tokens[i].start != -1 && tokens[i].end == -1) {
            if (tokens[i].type == JSON_OBJECT || tokens[i].type == JSON_ARRAY) {
                tokens[i].end = parser->pos;
            } else {
                return -1;
            }
        }
    }

    return count;
}

json_token_t* json_find_token(const char *js, json_token_t *tokens,
                              int num_tokens, const char *key) {
    int i;
    int key_len = strlen(key);

    for (i = 0; i < num_tokens; i++) {
        if (tokens[i].type == JSON_STRING) {
            int token_len = tokens[i].end - tokens[i].start;
            if (key_len == token_len &&
                strncmp(js + tokens[i].start, key, token_len) == 0) {
                // Return the value token (next token)
                if (i + 1 < num_tokens) {
                    return &tokens[i + 1];
                }
            }
        }
    }
    return NULL;
}

int json_get_string(const char *js, json_token_t *token, char *dst, size_t size) {
    if (token->type != JSON_STRING) {
        return -1;
    }
    int len = token->end - token->start;
    if (len >= (int)size) {
        len = size - 1;
    }
    strncpy(dst, js + token->start, len);
    dst[len] = '\0';
    return len;
}

int json_get_int(const char *js, json_token_t *token) {
    if (token->type != JSON_PRIMITIVE) {
        return 0;
    }
    char temp[32];
    int len = token->end - token->start;
    if (len >= sizeof(temp)) {
        len = sizeof(temp) - 1;
    }
    strncpy(temp, js + token->start, len);
    temp[len] = '\0';
    return atoi(temp);
}

float json_get_float(const char *js, json_token_t *token) {
    if (token->type != JSON_PRIMITIVE) {
        return 0.0f;
    }
    char temp[32];
    int len = token->end - token->start;
    if (len >= sizeof(temp)) {
        len = sizeof(temp) - 1;
    }
    strncpy(temp, js + token->start, len);
    temp[len] = '\0';
    return atof(temp);
}

bool json_get_bool(const char *js, json_token_t *token) {
    if (token->type != JSON_PRIMITIVE) {
        return false;
    }
    char c = js[token->start];
    return (c == 't' || c == 'T');
}

// JSON builder functions
void json_build_object_start(char *buffer, size_t buffer_size, int *pos) {
    if (*pos < (int)buffer_size - 1) {
        buffer[*pos] = '{';
        (*pos)++;
    }
}

void json_build_object_end(char *buffer, size_t buffer_size, int *pos) {
    if (*pos < (int)buffer_size - 1) {
        buffer[*pos] = '}';
        (*pos)++;
        buffer[*pos] = '\0';
    }
}

void json_build_array_start(char *buffer, size_t buffer_size, int *pos, const char *key) {
    int len = snprintf(buffer + *pos, buffer_size - *pos, "\"%s\":[", key);
    if (len > 0 && *pos + len < (int)buffer_size) {
        *pos += len;
    }
}

void json_build_array_end(char *buffer, size_t buffer_size, int *pos) {
    if (*pos < (int)buffer_size - 1) {
        buffer[*pos] = ']';
        (*pos)++;
    }
}

void json_build_string(char *buffer, size_t buffer_size, int *pos,
                      const char *key, const char *value) {
    if (*pos > 1) {  // Add comma if not first element
        if (*pos < (int)buffer_size - 1) {
            buffer[*pos] = ',';
            (*pos)++;
        }
    }
    int len = snprintf(buffer + *pos, buffer_size - *pos, "\"%s\":\"%s\"", key, value);
    if (len > 0 && *pos + len < (int)buffer_size) {
        *pos += len;
    }
}

void json_build_int(char *buffer, size_t buffer_size, int *pos,
                   const char *key, int value) {
    if (*pos > 1) {  // Add comma if not first element
        if (*pos < (int)buffer_size - 1) {
            buffer[*pos] = ',';
            (*pos)++;
        }
    }
    int len = snprintf(buffer + *pos, buffer_size - *pos, "\"%s\":%d", key, value);
    if (len > 0 && *pos + len < (int)buffer_size) {
        *pos += len;
    }
}

void json_build_float(char *buffer, size_t buffer_size, int *pos,
                     const char *key, float value) {
    if (*pos > 1) {  // Add comma if not first element
        if (*pos < (int)buffer_size - 1) {
            buffer[*pos] = ',';
            (*pos)++;
        }
    }
    int len = snprintf(buffer + *pos, buffer_size - *pos, "\"%s\":%.2f", key, value);
    if (len > 0 && *pos + len < (int)buffer_size) {
        *pos += len;
    }
}

void json_build_bool(char *buffer, size_t buffer_size, int *pos,
                    const char *key, bool value) {
    if (*pos > 1) {  // Add comma if not first element
        if (*pos < (int)buffer_size - 1) {
            buffer[*pos] = ',';
            (*pos)++;
        }
    }
    int len = snprintf(buffer + *pos, buffer_size - *pos, "\"%s\":%s", key, value ? "true" : "false");
    if (len > 0 && *pos + len < (int)buffer_size) {
        *pos += len;
    }
}

// ========== ENHANCED ARRAY PARSING FUNCTIONS ==========

json_token_t* json_find_array_element(const char* js, json_token_t* tokens, int num_tokens,
                                     const char* array_key, int element_index) {
    if (!js || !tokens || !array_key || element_index < 0) {
        return NULL;
    }
    
    // First find the array token
    json_token_t* array_token = json_find_token(js, tokens, num_tokens, array_key);
    if (!array_token || array_token->type != JSON_ARRAY) {
        return NULL;
    }
    
    // Check if element index is within bounds
    if (element_index >= array_token->size) {
        return NULL;
    }
    
    // Find the array token's position in the tokens array
    int array_token_index = array_token - tokens;
    
    // Navigate to the requested element
    // In a proper JSON parser, we need to traverse children
    int current_element = 0;
    for (int i = array_token_index + 1; i < num_tokens; i++) {
        json_token_t* token = &tokens[i];
        
        // Check if this token is a direct child of our array
        if (token->parent == array_token_index) {
            if (current_element == element_index) {
                return token;
            }
            current_element++;
        }
        
        // If we've gone past the array, stop searching
        if (token->start >= array_token->end) {
            break;
        }
    }
    
    return NULL;
}

int json_count_array_elements(json_token_t* array_token) {
    if (!array_token || array_token->type != JSON_ARRAY) {
        return 0;
    }
    return array_token->size;
}

bool json_is_array_element(const char* js, json_token_t* tokens, int token_index,
                          const char* parent_array_key, int expected_index) {
    if (!js || !tokens || !parent_array_key || token_index < 0 || expected_index < 0) {
        return false;
    }
    
    json_token_t* element = json_find_array_element(js, tokens, JSON_MAX_TOKENS,
                                                   parent_array_key, expected_index);
    
    return (element == &tokens[token_index]);
}