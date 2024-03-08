#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CONFIG_KEY_LENGTH 64
#define MAX_CONFIG_VALUE_LENGTH 1024

typedef struct
{
  char key[MAX_CONFIG_KEY_LENGTH + 1];
  char value[MAX_CONFIG_VALUE_LENGTH + 1];
} Config_t;

void parse_line(char *line, Config_t *config)
{
  char *delimiter_position = strchr(line, ':');
  if (delimiter_position != NULL)
  {
    strncpy(config->key, line, delimiter_position - line);
    config->key[delimiter_position - line] = '\0';

    strcpy(config->value, delimiter_position + 1);
    config->value[strcspn(config->value, "\n")] = '\0';
  }
}

int config_read(const char *filename, Config_t *configs, int max_configs)
{
  FILE *file = fopen(filename, "r");
  if (file == NULL)
  {
    perror("Failed to open the file");
    return -1;
  }

  char line[MAX_CONFIG_KEY_LENGTH + MAX_CONFIG_VALUE_LENGTH + 2];
  int i = 0;
  while (fgets(line, sizeof(line), file) && i < max_configs)
  {
    line[strcspn(line, "\n")] = 0;
    if (line[0] == '\0' || line[0] == '#')
    {
      continue;
    }
    parse_line(line, &configs[i]);
    ++i;
  }

  fclose(file);
  return i;
}

void config_print(const Config_t *configs, int num_configs)
{
  for (int i = 0; i < num_configs; ++i)
  {
    printf("Key: %s, Value: %s\n", configs[i].key, configs[i].value);
  }
}

int config_get_string(const Config_t *configs, int num_configs, const char *key, char *value)
{
  for (int i = 0; i < num_configs; i++)
  {
    if (strcmp(configs[i].key, key) == 0)
    {
      strncpy(value, configs[i].value, MAX_CONFIG_VALUE_LENGTH);
      return 1;
    }
  }
  return 0;
}

int config_get_int(const Config_t *configs, int num_configs, const char *key, int *value)
{
  for (int i = 0; i < num_configs; i++)
  {
    if (strcmp(configs[i].key, key) == 0)
    {
      *value = atoi(configs[i].value);
      return 1;
    }
  }
  return 0;
}

int config_get_float(const Config_t *configs, int num_configs, const char *key, float *value)
{
  for (int i = 0; i < num_configs; i++)
  {
    if (strcmp(configs[i].key, key) == 0)
    {
      *value = atof(configs[i].value);
      return 1;
    }
  }
  return 0;
}

char **split_string(const char *str, const char *delimiter, int *num_elements)
{
  char **result = NULL;
  int i = 0;
  int str_len = strlen(str);
  int delimiter_len = strlen(delimiter);

  if (str_len == 0 || delimiter_len == 0)
  {
    *num_elements = 0;
    return NULL;
  }

  result = (char **)malloc((str_len / delimiter_len + 1) * sizeof(char *));
  if (!result)
  {
    *num_elements = 0;
    return NULL;
  }

  char *token = strtok(strdup(str), delimiter);
  while (token != NULL)
  {
    result[i] = strdup(token);
    i++;
    token = strtok(NULL, delimiter);
  }

  *num_elements = i;

  return result;
}

int config_get_array(const Config_t *configs, int num_configs, const char *key, char ***array, int *num_elements)
{
  for (int i = 0; i < num_configs; i++)
  {
    if (strcmp(configs[i].key, key) == 0)
    {
      *array = split_string(configs[i].value, ",", num_elements);
      return 1;
    }
  }
  *array = NULL;
  *num_elements = 0;
  return 0;
}

void free_array(char **array, int num_elements)
{
  for (int i = 0; i < num_elements; i++)
  {
    free(array[i]);
  }
  free(array);
}

void test()
{
  const char *filename = "config.txt";
  Config_t configs[10];
  int num_configs = config_read(filename, configs, sizeof(configs) / sizeof(configs[0]));
  config_print(configs, num_configs);

  float value_f;
  if (config_get_float(configs, num_configs, "A", &value_f))
  {
    printf("A: %f\n", value_f);
  }
  else
  {
    printf("Error: Key 'A' not found in config file\n");
  }
  int value_i;
  if (config_get_int(configs, num_configs, "B", &value_i))
  {
    printf("B: %d\n", value_i);
  }
  else
  {
    printf("Error: Key 'B' not found in config file\n");
  }
  char value[MAX_CONFIG_VALUE_LENGTH];
  if (config_get_string(configs, num_configs, "C", value))
  {
    printf("C: %s\n", value);
  }
  else
  {
    printf("Error: Key 'C' not found in config file\n");
  }
  if (config_get_array(configs, num_configs, "C", value))
  {
    printf("C: %s\n", value);
  }
  else
  {
    printf("Error: Key 'C' not found in config file\n");
  }
}

int main()
{
  test();
  return 0;
}
