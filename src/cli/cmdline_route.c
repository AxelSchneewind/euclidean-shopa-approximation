/*
  File autogenerated by gengetopt version 2.23
  generated with the following command:
  /usr/bin/gengetopt -i src/cli/route.ggo -F cmdline_route --output-dir=src/cli --set-version=0.1 

  The developers of gengetopt consider the fixed text that goes in all
  gengetopt output files to be in the public domain:
  we make no copyright claims on it.
*/

/* If we use autoconf.  */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FIX_UNUSED
#define FIX_UNUSED(X) (void) (X) /* avoid warnings for unused params */
#endif

#include <getopt.h>

#include "cmdline_route.h"

const char *gengetopt_args_info_purpose = "";

const char *gengetopt_args_info_usage = "Usage: route --graph-file /path/to/graph-file --output-directory\n/path/to/results-directory --epsilon 0.5 -q src0,dest0,...";

const char *gengetopt_args_info_versiontext = "";

const char *gengetopt_args_info_description = "computes the shortest path for each given pair of source and destination node.";

const char *gengetopt_args_info_help[] = {
  "  -h, --help                   Print help and exit",
  "  -V, --version                Print version and exit",
  "\ninput/output files:",
  "  -g, --graph-file=FILE        path to graph file (of type .fmi or .graph)",
  "  -o, --output-directory=FILE  path to output directory (where the subdirectory\n                                 with the output files should be created)\n                                 (default=`.')",
  "\ntriangulation:",
  "  -e, --epsilon=DOUBLE         ε value to use for discretizing the\n                                 triangulation (if a .graph file is given)\n                                 (default=`0.0')",
  "\nquery:",
  "  -q, --query=STRING           pair(s) of source and destination node ids",
  "\noutput:",
  "  options for the command line output",
  "  -p, --projection=ENUM        which projection to apply to coordinates when\n                                 writing to files  (possible values=\"none\",\n                                 \"google_bing\", \"wgs84\" default=`none')",
  "  -t, --tree[=INT]             maximum tree size to export to graph file\n                                 (default=`1000000')",
  "  -l, --live-status            print live status about computation to stdout\n                                 (default=on)",
  "\nrouting algorithms:",
  "  some advanced options on dijkstra search",
  "  -a, --astar                  use A* heuristic to speed up one-to-one queries\n                                 (default=off)",
  "      --neighbor-finding=ENUM  the type of algorithm to find neighbors with\n                                 minimal bending angle  (possible\n                                 values=\"param\", \"trig\", \"binary\",\n                                 \"linear\" default=`param')",
  "      --pruning=ENUM           which type of pruning to use for steiner graphs\n                                 (possible values=\"none\", \"prune\",\n                                 \"prune-min-angle\" default=`prune')",
  "      --no-tree                if enabled, only computes distances without\n                                 keeping tree information (does not produce\n                                 paths)  (default=off)",
    0
};

typedef enum {ARG_NO
  , ARG_FLAG
  , ARG_STRING
  , ARG_INT
  , ARG_DOUBLE
  , ARG_ENUM
} cmdline_parser_arg_type;

static
void clear_given (struct gengetopt_args_info *args_info);
static
void clear_args (struct gengetopt_args_info *args_info);

static int
cmdline_parser_internal (int argc, char **argv, struct gengetopt_args_info *args_info,
                        struct cmdline_parser_params *params, const char *additional_error);

static int
cmdline_parser_required2 (struct gengetopt_args_info *args_info, const char *prog_name, const char *additional_error);

const char *cmdline_parser_projection_values[] = {"none", "google_bing", "wgs84", 0}; /*< Possible values for projection. */
const char *cmdline_parser_neighbor_finding_values[] = {"param", "trig", "binary", "linear", 0}; /*< Possible values for neighbor-finding. */
const char *cmdline_parser_pruning_values[] = {"none", "prune", "prune-min-angle", 0}; /*< Possible values for pruning. */

static char *
gengetopt_strdup (const char *s);

static
void clear_given (struct gengetopt_args_info *args_info)
{
  args_info->help_given = 0 ;
  args_info->version_given = 0 ;
  args_info->graph_file_given = 0 ;
  args_info->output_directory_given = 0 ;
  args_info->epsilon_given = 0 ;
  args_info->query_given = 0 ;
  args_info->projection_given = 0 ;
  args_info->tree_given = 0 ;
  args_info->live_status_given = 0 ;
  args_info->astar_given = 0 ;
  args_info->neighbor_finding_given = 0 ;
  args_info->pruning_given = 0 ;
  args_info->no_tree_given = 0 ;
}

static
void clear_args (struct gengetopt_args_info *args_info)
{
  FIX_UNUSED (args_info);
  args_info->graph_file_arg = NULL;
  args_info->graph_file_orig = NULL;
  args_info->output_directory_arg = gengetopt_strdup (".");
  args_info->output_directory_orig = NULL;
  args_info->epsilon_arg = 0.0;
  args_info->epsilon_orig = NULL;
  args_info->query_arg = NULL;
  args_info->query_orig = NULL;
  args_info->projection_arg = projection_arg_none;
  args_info->projection_orig = NULL;
  args_info->tree_arg = 1000000;
  args_info->tree_orig = NULL;
  args_info->live_status_flag = 1;
  args_info->astar_flag = 0;
  args_info->neighbor_finding_arg = neighbor_finding_arg_param;
  args_info->neighbor_finding_orig = NULL;
  args_info->pruning_arg = pruning_arg_prune;
  args_info->pruning_orig = NULL;
  args_info->no_tree_flag = 0;
  
}

static
void init_args_info(struct gengetopt_args_info *args_info)
{


  args_info->help_help = gengetopt_args_info_help[0] ;
  args_info->version_help = gengetopt_args_info_help[1] ;
  args_info->graph_file_help = gengetopt_args_info_help[3] ;
  args_info->output_directory_help = gengetopt_args_info_help[4] ;
  args_info->epsilon_help = gengetopt_args_info_help[6] ;
  args_info->query_help = gengetopt_args_info_help[8] ;
  args_info->query_min = 0;
  args_info->query_max = 0;
  args_info->projection_help = gengetopt_args_info_help[11] ;
  args_info->tree_help = gengetopt_args_info_help[12] ;
  args_info->live_status_help = gengetopt_args_info_help[13] ;
  args_info->astar_help = gengetopt_args_info_help[16] ;
  args_info->neighbor_finding_help = gengetopt_args_info_help[17] ;
  args_info->pruning_help = gengetopt_args_info_help[18] ;
  args_info->no_tree_help = gengetopt_args_info_help[19] ;
  
}

void
cmdline_parser_print_version (void)
{
  printf ("%s %s\n",
     (strlen(CMDLINE_PARSER_PACKAGE_NAME) ? CMDLINE_PARSER_PACKAGE_NAME : CMDLINE_PARSER_PACKAGE),
     CMDLINE_PARSER_VERSION);

  if (strlen(gengetopt_args_info_versiontext) > 0)
    printf("\n%s\n", gengetopt_args_info_versiontext);
}

static void print_help_common(void)
{
	size_t len_purpose = strlen(gengetopt_args_info_purpose);
	size_t len_usage = strlen(gengetopt_args_info_usage);

	if (len_usage > 0) {
		printf("%s\n", gengetopt_args_info_usage);
	}
	if (len_purpose > 0) {
		printf("%s\n", gengetopt_args_info_purpose);
	}

	if (len_usage || len_purpose) {
		printf("\n");
	}

	if (strlen(gengetopt_args_info_description) > 0) {
		printf("%s\n\n", gengetopt_args_info_description);
	}
}

void
cmdline_parser_print_help (void)
{
  int i = 0;
  print_help_common();
  while (gengetopt_args_info_help[i])
    printf("%s\n", gengetopt_args_info_help[i++]);
}

void
cmdline_parser_init (struct gengetopt_args_info *args_info)
{
  clear_given (args_info);
  clear_args (args_info);
  init_args_info (args_info);
}

void
cmdline_parser_params_init(struct cmdline_parser_params *params)
{
  if (params)
    { 
      params->override = 0;
      params->initialize = 1;
      params->check_required = 1;
      params->check_ambiguity = 0;
      params->print_errors = 1;
    }
}

struct cmdline_parser_params *
cmdline_parser_params_create(void)
{
  struct cmdline_parser_params *params = 
    (struct cmdline_parser_params *)malloc(sizeof(struct cmdline_parser_params));
  cmdline_parser_params_init(params);  
  return params;
}

static void
free_string_field (char **s)
{
  if (*s)
    {
      free (*s);
      *s = 0;
    }
}

/** @brief generic value variable */
union generic_value {
    int int_arg;
    double double_arg;
    char *string_arg;
    const char *default_string_arg;
};

/** @brief holds temporary values for multiple options */
struct generic_list
{
  union generic_value arg;
  char *orig;
  struct generic_list *next;
};

/**
 * @brief add a node at the head of the list 
 */
static void add_node(struct generic_list **list) {
  struct generic_list *new_node = (struct generic_list *) malloc (sizeof (struct generic_list));
  new_node->next = *list;
  *list = new_node;
  new_node->arg.string_arg = 0;
  new_node->orig = 0;
}


static void
free_multiple_string_field(unsigned int len, char ***arg, char ***orig)
{
  unsigned int i;
  if (*arg) {
    for (i = 0; i < len; ++i)
      {
        free_string_field(&((*arg)[i]));
        free_string_field(&((*orig)[i]));
      }
    free_string_field(&((*arg)[0])); /* free default string */

    free (*arg);
    *arg = 0;
    free (*orig);
    *orig = 0;
  }
}

static void
cmdline_parser_release (struct gengetopt_args_info *args_info)
{

  free_string_field (&(args_info->graph_file_arg));
  free_string_field (&(args_info->graph_file_orig));
  free_string_field (&(args_info->output_directory_arg));
  free_string_field (&(args_info->output_directory_orig));
  free_string_field (&(args_info->epsilon_orig));
  free_multiple_string_field (args_info->query_given, &(args_info->query_arg), &(args_info->query_orig));
  free_string_field (&(args_info->projection_orig));
  free_string_field (&(args_info->tree_orig));
  free_string_field (&(args_info->neighbor_finding_orig));
  free_string_field (&(args_info->pruning_orig));
  
  

  clear_given (args_info);
}

/**
 * @param val the value to check
 * @param values the possible values
 * @return the index of the matched value:
 * -1 if no value matched,
 * -2 if more than one value has matched
 */
static int
check_possible_values(const char *val, const char *values[])
{
  int i, found, last;
  size_t len;

  if (!val)   /* otherwise strlen() crashes below */
    return -1; /* -1 means no argument for the option */

  found = last = 0;

  for (i = 0, len = strlen(val); values[i]; ++i)
    {
      if (strncmp(val, values[i], len) == 0)
        {
          ++found;
          last = i;
          if (strlen(values[i]) == len)
            return i; /* exact macth no need to check more */
        }
    }

  if (found == 1) /* one match: OK */
    return last;

  return (found ? -2 : -1); /* return many values or none matched */
}


static void
write_into_file(FILE *outfile, const char *opt, const char *arg, const char *values[])
{
  int found = -1;
  if (arg) {
    if (values) {
      found = check_possible_values(arg, values);      
    }
    if (found >= 0)
      fprintf(outfile, "%s=\"%s\" # %s\n", opt, arg, values[found]);
    else
      fprintf(outfile, "%s=\"%s\"\n", opt, arg);
  } else {
    fprintf(outfile, "%s\n", opt);
  }
}

static void
write_multiple_into_file(FILE *outfile, int len, const char *opt, char **arg, const char *values[])
{
  int i;
  
  for (i = 0; i < len; ++i)
    write_into_file(outfile, opt, (arg ? arg[i] : 0), values);
}

int
cmdline_parser_dump(FILE *outfile, struct gengetopt_args_info *args_info)
{
  int i = 0;

  if (!outfile)
    {
      fprintf (stderr, "%s: cannot dump options to stream\n", CMDLINE_PARSER_PACKAGE);
      return EXIT_FAILURE;
    }

  if (args_info->help_given)
    write_into_file(outfile, "help", 0, 0 );
  if (args_info->version_given)
    write_into_file(outfile, "version", 0, 0 );
  if (args_info->graph_file_given)
    write_into_file(outfile, "graph-file", args_info->graph_file_orig, 0);
  if (args_info->output_directory_given)
    write_into_file(outfile, "output-directory", args_info->output_directory_orig, 0);
  if (args_info->epsilon_given)
    write_into_file(outfile, "epsilon", args_info->epsilon_orig, 0);
  write_multiple_into_file(outfile, args_info->query_given, "query", args_info->query_orig, 0);
  if (args_info->projection_given)
    write_into_file(outfile, "projection", args_info->projection_orig, cmdline_parser_projection_values);
  if (args_info->tree_given)
    write_into_file(outfile, "tree", args_info->tree_orig, 0);
  if (args_info->live_status_given)
    write_into_file(outfile, "live-status", 0, 0 );
  if (args_info->astar_given)
    write_into_file(outfile, "astar", 0, 0 );
  if (args_info->neighbor_finding_given)
    write_into_file(outfile, "neighbor-finding", args_info->neighbor_finding_orig, cmdline_parser_neighbor_finding_values);
  if (args_info->pruning_given)
    write_into_file(outfile, "pruning", args_info->pruning_orig, cmdline_parser_pruning_values);
  if (args_info->no_tree_given)
    write_into_file(outfile, "no-tree", 0, 0 );
  

  i = EXIT_SUCCESS;
  return i;
}

int
cmdline_parser_file_save(const char *filename, struct gengetopt_args_info *args_info)
{
  FILE *outfile;
  int i = 0;

  outfile = fopen(filename, "w");

  if (!outfile)
    {
      fprintf (stderr, "%s: cannot open file for writing: %s\n", CMDLINE_PARSER_PACKAGE, filename);
      return EXIT_FAILURE;
    }

  i = cmdline_parser_dump(outfile, args_info);
  fclose (outfile);

  return i;
}

void
cmdline_parser_free (struct gengetopt_args_info *args_info)
{
  cmdline_parser_release (args_info);
}

/** @brief replacement of strdup, which is not standard */
char *
gengetopt_strdup (const char *s)
{
  char *result = 0;
  if (!s)
    return result;

  result = (char*)malloc(strlen(s) + 1);
  if (result == (char*)0)
    return (char*)0;
  strcpy(result, s);
  return result;
}

static char *
get_multiple_arg_token(const char *arg)
{
  const char *tok;
  char *ret;
  size_t len, num_of_escape, i, j;

  if (!arg)
    return 0;

  tok = strchr (arg, ',');
  num_of_escape = 0;

  /* make sure it is not escaped */
  while (tok)
    {
      if (*(tok-1) == '\\')
        {
          /* find the next one */
          tok = strchr (tok+1, ',');
          ++num_of_escape;
        }
      else
        break;
    }

  if (tok)
    len = (size_t)(tok - arg + 1);
  else
    len = strlen (arg) + 1;

  len -= num_of_escape;

  ret = (char *) malloc (len);

  i = 0;
  j = 0;
  while (arg[i] && (j < len-1))
    {
      if (arg[i] == '\\' && 
	  arg[ i + 1 ] && 
	  arg[ i + 1 ] == ',')
        ++i;

      ret[j++] = arg[i++];
    }

  ret[len-1] = '\0';

  return ret;
}

static const char *
get_multiple_arg_token_next(const char *arg)
{
  const char *tok;

  if (!arg)
    return 0;

  tok = strchr (arg, ',');

  /* make sure it is not escaped */
  while (tok)
    {
      if (*(tok-1) == '\\')
        {
          /* find the next one */
          tok = strchr (tok+1, ',');
        }
      else
        break;
    }

  if (! tok || strlen(tok) == 1)
    return 0;

  return tok+1;
}

static int
check_multiple_option_occurrences(const char *prog_name, unsigned int option_given, unsigned int min, unsigned int max, const char *option_desc);

int
check_multiple_option_occurrences(const char *prog_name, unsigned int option_given, unsigned int min, unsigned int max, const char *option_desc)
{
  int error_occurred = 0;

  if (option_given && (min > 0 || max > 0))
    {
      if (min > 0 && max > 0)
        {
          if (min == max)
            {
              /* specific occurrences */
              if (option_given != (unsigned int) min)
                {
                  fprintf (stderr, "%s: %s option occurrences must be %d\n",
                    prog_name, option_desc, min);
                  error_occurred = 1;
                }
            }
          else if (option_given < (unsigned int) min
                || option_given > (unsigned int) max)
            {
              /* range occurrences */
              fprintf (stderr, "%s: %s option occurrences must be between %d and %d\n",
                prog_name, option_desc, min, max);
              error_occurred = 1;
            }
        }
      else if (min > 0)
        {
          /* at least check */
          if (option_given < min)
            {
              fprintf (stderr, "%s: %s option occurrences must be at least %d\n",
                prog_name, option_desc, min);
              error_occurred = 1;
            }
        }
      else if (max > 0)
        {
          /* at most check */
          if (option_given > max)
            {
              fprintf (stderr, "%s: %s option occurrences must be at most %d\n",
                prog_name, option_desc, max);
              error_occurred = 1;
            }
        }
    }
    
  return error_occurred;
}
int
cmdline_parser (int argc, char **argv, struct gengetopt_args_info *args_info)
{
  return cmdline_parser2 (argc, argv, args_info, 0, 1, 1);
}

int
cmdline_parser_ext (int argc, char **argv, struct gengetopt_args_info *args_info,
                   struct cmdline_parser_params *params)
{
  int result;
  result = cmdline_parser_internal (argc, argv, args_info, params, 0);

  if (result == EXIT_FAILURE)
    {
      cmdline_parser_free (args_info);
      exit (EXIT_FAILURE);
    }
  
  return result;
}

int
cmdline_parser2 (int argc, char **argv, struct gengetopt_args_info *args_info, int override, int initialize, int check_required)
{
  int result;
  struct cmdline_parser_params params;
  
  params.override = override;
  params.initialize = initialize;
  params.check_required = check_required;
  params.check_ambiguity = 0;
  params.print_errors = 1;

  result = cmdline_parser_internal (argc, argv, args_info, &params, 0);

  if (result == EXIT_FAILURE)
    {
      cmdline_parser_free (args_info);
      exit (EXIT_FAILURE);
    }
  
  return result;
}

int
cmdline_parser_required (struct gengetopt_args_info *args_info, const char *prog_name)
{
  int result = EXIT_SUCCESS;

  if (cmdline_parser_required2(args_info, prog_name, 0) > 0)
    result = EXIT_FAILURE;

  if (result == EXIT_FAILURE)
    {
      cmdline_parser_free (args_info);
      exit (EXIT_FAILURE);
    }
  
  return result;
}

int
cmdline_parser_required2 (struct gengetopt_args_info *args_info, const char *prog_name, const char *additional_error)
{
  int error_occurred = 0;
  FIX_UNUSED (additional_error);

  /* checks for required options */
  if (! args_info->graph_file_given)
    {
      fprintf (stderr, "%s: '--graph-file' ('-g') option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error_occurred = 1;
    }
  
  if (! args_info->output_directory_given)
    {
      fprintf (stderr, "%s: '--output-directory' ('-o') option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error_occurred = 1;
    }
  
  if (! args_info->query_given)
    {
      fprintf (stderr, "%s: '--query' ('-q') option required%s\n", prog_name, (additional_error ? additional_error : ""));
      error_occurred = 1;
    }
  
  if (check_multiple_option_occurrences(prog_name, args_info->query_given, args_info->query_min, args_info->query_max, "'--query' ('-q')"))
     error_occurred = 1;
  
  
  /* checks for dependences among options */

  return error_occurred;
}


static char *package_name = 0;

/**
 * @brief updates an option
 * @param field the generic pointer to the field to update
 * @param orig_field the pointer to the orig field
 * @param field_given the pointer to the number of occurrence of this option
 * @param prev_given the pointer to the number of occurrence already seen
 * @param value the argument for this option (if null no arg was specified)
 * @param possible_values the possible values for this option (if specified)
 * @param default_value the default value (in case the option only accepts fixed values)
 * @param arg_type the type of this option
 * @param check_ambiguity @see cmdline_parser_params.check_ambiguity
 * @param override @see cmdline_parser_params.override
 * @param no_free whether to free a possible previous value
 * @param multiple_option whether this is a multiple option
 * @param long_opt the corresponding long option
 * @param short_opt the corresponding short option (or '-' if none)
 * @param additional_error possible further error specification
 */
static
int update_arg(void *field, char **orig_field,
               unsigned int *field_given, unsigned int *prev_given, 
               char *value, const char *possible_values[],
               const char *default_value,
               cmdline_parser_arg_type arg_type,
               int check_ambiguity, int override,
               int no_free, int multiple_option,
               const char *long_opt, char short_opt,
               const char *additional_error)
{
  char *stop_char = 0;
  const char *val = value;
  int found;
  char **string_field;
  FIX_UNUSED (field);

  stop_char = 0;
  found = 0;

  if (!multiple_option && prev_given && (*prev_given || (check_ambiguity && *field_given)))
    {
      if (short_opt != '-')
        fprintf (stderr, "%s: `--%s' (`-%c') option given more than once%s\n", 
               package_name, long_opt, short_opt,
               (additional_error ? additional_error : ""));
      else
        fprintf (stderr, "%s: `--%s' option given more than once%s\n", 
               package_name, long_opt,
               (additional_error ? additional_error : ""));
      return 1; /* failure */
    }

  if (possible_values && (found = check_possible_values((value ? value : default_value), possible_values)) < 0)
    {
      if (short_opt != '-')
        fprintf (stderr, "%s: %s argument, \"%s\", for option `--%s' (`-%c')%s\n", 
          package_name, (found == -2) ? "ambiguous" : "invalid", value, long_opt, short_opt,
          (additional_error ? additional_error : ""));
      else
        fprintf (stderr, "%s: %s argument, \"%s\", for option `--%s'%s\n", 
          package_name, (found == -2) ? "ambiguous" : "invalid", value, long_opt,
          (additional_error ? additional_error : ""));
      return 1; /* failure */
    }
    
  if (field_given && *field_given && ! override)
    return 0;
  if (prev_given)
    (*prev_given)++;
  if (field_given)
    (*field_given)++;
  if (possible_values)
    val = possible_values[found];

  switch(arg_type) {
  case ARG_FLAG:
    *((int *)field) = !*((int *)field);
    break;
  case ARG_INT:
    if (val) *((int *)field) = strtol (val, &stop_char, 0);
    break;
  case ARG_DOUBLE:
    if (val) *((double *)field) = strtod (val, &stop_char);
    break;
  case ARG_ENUM:
    if (val) *((int *)field) = found;
    break;
  case ARG_STRING:
    if (val) {
      string_field = (char **)field;
      if (!no_free && *string_field)
        free (*string_field); /* free previous string */
      *string_field = gengetopt_strdup (val);
    }
    break;
  default:
    break;
  };

  /* check numeric conversion */
  switch(arg_type) {
  case ARG_INT:
  case ARG_DOUBLE:
    if (val && !(stop_char && *stop_char == '\0')) {
      fprintf(stderr, "%s: invalid numeric value: %s\n", package_name, val);
      return 1; /* failure */
    }
    break;
  default:
    ;
  };

  /* store the original value */
  switch(arg_type) {
  case ARG_NO:
  case ARG_FLAG:
    break;
  default:
    if (value && orig_field) {
      if (no_free) {
        *orig_field = value;
      } else {
        if (*orig_field)
          free (*orig_field); /* free previous string */
        *orig_field = gengetopt_strdup (value);
      }
    }
  };

  return 0; /* OK */
}

/**
 * @brief store information about a multiple option in a temporary list
 * @param list where to (temporarily) store multiple options
 */
static
int update_multiple_arg_temp(struct generic_list **list,
               unsigned int *prev_given, const char *val,
               const char *possible_values[], const char *default_value,
               cmdline_parser_arg_type arg_type,
               const char *long_opt, char short_opt,
               const char *additional_error)
{
  /* store single arguments */
  char *multi_token;
  const char *multi_next;

  if (arg_type == ARG_NO) {
    (*prev_given)++;
    return 0; /* OK */
  }

  multi_token = get_multiple_arg_token(val);
  multi_next = get_multiple_arg_token_next (val);

  while (1)
    {
      add_node (list);
      if (update_arg((void *)&((*list)->arg), &((*list)->orig), 0,
          prev_given, multi_token, possible_values, default_value, 
          arg_type, 0, 1, 1, 1, long_opt, short_opt, additional_error)) {
        if (multi_token) free(multi_token);
        return 1; /* failure */
      }

      if (multi_next)
        {
          multi_token = get_multiple_arg_token(multi_next);
          multi_next = get_multiple_arg_token_next (multi_next);
        }
      else
        break;
    }

  return 0; /* OK */
}

/**
 * @brief free the passed list (including possible string argument)
 */
static
void free_list(struct generic_list *list, short string_arg)
{
  if (list) {
    struct generic_list *tmp;
    while (list)
      {
        tmp = list;
        if (string_arg && list->arg.string_arg)
          free (list->arg.string_arg);
        if (list->orig)
          free (list->orig);
        list = list->next;
        free (tmp);
      }
  }
}

/**
 * @brief updates a multiple option starting from the passed list
 */
static
void update_multiple_arg(void *field, char ***orig_field,
               unsigned int field_given, unsigned int prev_given, union generic_value *default_value,
               cmdline_parser_arg_type arg_type,
               struct generic_list *list)
{
  int i;
  struct generic_list *tmp;

  if (prev_given && list) {
    *orig_field = (char **) realloc (*orig_field, (field_given + prev_given) * sizeof (char *));

    switch(arg_type) {
    case ARG_INT:
    case ARG_ENUM:
      *((int **)field) = (int *)realloc (*((int **)field), (field_given + prev_given) * sizeof (int)); break;
    case ARG_DOUBLE:
      *((double **)field) = (double *)realloc (*((double **)field), (field_given + prev_given) * sizeof (double)); break;
    case ARG_STRING:
      *((char ***)field) = (char **)realloc (*((char ***)field), (field_given + prev_given) * sizeof (char *)); break;
    default:
      break;
    };
    
    for (i = (prev_given - 1); i >= 0; --i)
      {
        tmp = list;
        
        switch(arg_type) {
        case ARG_INT:
          (*((int **)field))[i + field_given] = tmp->arg.int_arg; break;
        case ARG_DOUBLE:
          (*((double **)field))[i + field_given] = tmp->arg.double_arg; break;
        case ARG_ENUM:
          (*((int **)field))[i + field_given] = tmp->arg.int_arg; break;
        case ARG_STRING:
          (*((char ***)field))[i + field_given] = tmp->arg.string_arg; break;
        default:
          break;
        }        
        (*orig_field) [i + field_given] = list->orig;
        list = list->next;
        free (tmp);
      }
  } else { /* set the default value */
    if (default_value && ! field_given) {
      switch(arg_type) {
      case ARG_INT:
      case ARG_ENUM:
        if (! *((int **)field)) {
          *((int **)field) = (int *)malloc (sizeof (int));
          (*((int **)field))[0] = default_value->int_arg; 
        }
        break;
      case ARG_DOUBLE:
        if (! *((double **)field)) {
          *((double **)field) = (double *)malloc (sizeof (double));
          (*((double **)field))[0] = default_value->double_arg;
        }
        break;
      case ARG_STRING:
        if (! *((char ***)field)) {
          *((char ***)field) = (char **)malloc (sizeof (char *));
          (*((char ***)field))[0] = gengetopt_strdup(default_value->string_arg);
        }
        break;
      default: break;
      }
      if (!(*orig_field)) {
        *orig_field = (char **) malloc (sizeof (char *));
        (*orig_field)[0] = 0;
      }
    }
  }
}

int
cmdline_parser_internal (
  int argc, char **argv, struct gengetopt_args_info *args_info,
                        struct cmdline_parser_params *params, const char *additional_error)
{
  int c;	/* Character of the parsed option.  */

  struct generic_list * query_list = NULL;
  int error_occurred = 0;
  struct gengetopt_args_info local_args_info;
  
  int override;
  int initialize;
  int check_required;
  int check_ambiguity;
  
  package_name = argv[0];
  
  /* TODO: Why is this here? It is not used anywhere. */
  override = params->override;
  FIX_UNUSED(override);

  initialize = params->initialize;
  check_required = params->check_required;

  /* TODO: Why is this here? It is not used anywhere. */
  check_ambiguity = params->check_ambiguity;
  FIX_UNUSED(check_ambiguity);

  if (initialize)
    cmdline_parser_init (args_info);

  cmdline_parser_init (&local_args_info);

  optarg = 0;
  optind = 0;
  opterr = params->print_errors;
  optopt = '?';

  while (1)
    {
      int option_index = 0;

      static struct option long_options[] = {
        { "help",	0, NULL, 'h' },
        { "version",	0, NULL, 'V' },
        { "graph-file",	1, NULL, 'g' },
        { "output-directory",	1, NULL, 'o' },
        { "epsilon",	1, NULL, 'e' },
        { "query",	1, NULL, 'q' },
        { "projection",	1, NULL, 'p' },
        { "tree",	2, NULL, 't' },
        { "live-status",	0, NULL, 'l' },
        { "astar",	0, NULL, 'a' },
        { "neighbor-finding",	1, NULL, 0 },
        { "pruning",	1, NULL, 0 },
        { "no-tree",	0, NULL, 0 },
        { 0,  0, 0, 0 }
      };

      c = getopt_long (argc, argv, "hVg:o:e:q:p:t::la", long_options, &option_index);

      if (c == -1) break;	/* Exit from `while (1)' loop.  */

      switch (c)
        {
        case 'h':	/* Print help and exit.  */
          cmdline_parser_print_help ();
          cmdline_parser_free (&local_args_info);
          exit (EXIT_SUCCESS);

        case 'V':	/* Print version and exit.  */
          cmdline_parser_print_version ();
          cmdline_parser_free (&local_args_info);
          exit (EXIT_SUCCESS);

        case 'g':	/* path to graph file (of type .fmi or .graph).  */
        
        
          if (update_arg( (void *)&(args_info->graph_file_arg), 
               &(args_info->graph_file_orig), &(args_info->graph_file_given),
              &(local_args_info.graph_file_given), optarg, 0, 0, ARG_STRING,
              check_ambiguity, override, 0, 0,
              "graph-file", 'g',
              additional_error))
            goto failure;
        
          break;
        case 'o':	/* path to output directory (where the subdirectory with the output files should be created).  */
        
        
          if (update_arg( (void *)&(args_info->output_directory_arg), 
               &(args_info->output_directory_orig), &(args_info->output_directory_given),
              &(local_args_info.output_directory_given), optarg, 0, ".", ARG_STRING,
              check_ambiguity, override, 0, 0,
              "output-directory", 'o',
              additional_error))
            goto failure;
        
          break;
        case 'e':	/* ε value to use for discretizing the triangulation (if a .graph file is given).  */
        
        
          if (update_arg( (void *)&(args_info->epsilon_arg), 
               &(args_info->epsilon_orig), &(args_info->epsilon_given),
              &(local_args_info.epsilon_given), optarg, 0, "0.0", ARG_DOUBLE,
              check_ambiguity, override, 0, 0,
              "epsilon", 'e',
              additional_error))
            goto failure;
        
          break;
        case 'q':	/* pair(s) of source and destination node ids.  */
        
          if (update_multiple_arg_temp(&query_list, 
              &(local_args_info.query_given), optarg, 0, 0, ARG_STRING,
              "query", 'q',
              additional_error))
            goto failure;
        
          break;
        case 'p':	/* which projection to apply to coordinates when writing to files.  */
        
        
          if (update_arg( (void *)&(args_info->projection_arg), 
               &(args_info->projection_orig), &(args_info->projection_given),
              &(local_args_info.projection_given), optarg, cmdline_parser_projection_values, "none", ARG_ENUM,
              check_ambiguity, override, 0, 0,
              "projection", 'p',
              additional_error))
            goto failure;
        
          break;
        case 't':	/* maximum tree size to export to graph file.  */
        
        
          if (update_arg( (void *)&(args_info->tree_arg), 
               &(args_info->tree_orig), &(args_info->tree_given),
              &(local_args_info.tree_given), optarg, 0, "1000000", ARG_INT,
              check_ambiguity, override, 0, 0,
              "tree", 't',
              additional_error))
            goto failure;
        
          break;
        case 'l':	/* print live status about computation to stdout.  */
        
        
          if (update_arg((void *)&(args_info->live_status_flag), 0, &(args_info->live_status_given),
              &(local_args_info.live_status_given), optarg, 0, 0, ARG_FLAG,
              check_ambiguity, override, 1, 0, "live-status", 'l',
              additional_error))
            goto failure;
        
          break;
        case 'a':	/* use A* heuristic to speed up one-to-one queries.  */
        
        
          if (update_arg((void *)&(args_info->astar_flag), 0, &(args_info->astar_given),
              &(local_args_info.astar_given), optarg, 0, 0, ARG_FLAG,
              check_ambiguity, override, 1, 0, "astar", 'a',
              additional_error))
            goto failure;
        
          break;

        case 0:	/* Long option with no short option */
          /* the type of algorithm to find neighbors with minimal bending angle.  */
          if (strcmp (long_options[option_index].name, "neighbor-finding") == 0)
          {
          
          
            if (update_arg( (void *)&(args_info->neighbor_finding_arg), 
                 &(args_info->neighbor_finding_orig), &(args_info->neighbor_finding_given),
                &(local_args_info.neighbor_finding_given), optarg, cmdline_parser_neighbor_finding_values, "param", ARG_ENUM,
                check_ambiguity, override, 0, 0,
                "neighbor-finding", '-',
                additional_error))
              goto failure;
          
          }
          /* which type of pruning to use for steiner graphs.  */
          else if (strcmp (long_options[option_index].name, "pruning") == 0)
          {
          
          
            if (update_arg( (void *)&(args_info->pruning_arg), 
                 &(args_info->pruning_orig), &(args_info->pruning_given),
                &(local_args_info.pruning_given), optarg, cmdline_parser_pruning_values, "prune", ARG_ENUM,
                check_ambiguity, override, 0, 0,
                "pruning", '-',
                additional_error))
              goto failure;
          
          }
          /* if enabled, only computes distances without keeping tree information (does not produce paths).  */
          else if (strcmp (long_options[option_index].name, "no-tree") == 0)
          {
          
          
            if (update_arg((void *)&(args_info->no_tree_flag), 0, &(args_info->no_tree_given),
                &(local_args_info.no_tree_given), optarg, 0, 0, ARG_FLAG,
                check_ambiguity, override, 1, 0, "no-tree", '-',
                additional_error))
              goto failure;
          
          }
          
          break;
        case '?':	/* Invalid option.  */
          /* `getopt_long' already printed an error message.  */
          goto failure;

        default:	/* bug: option not considered.  */
          fprintf (stderr, "%s: option unknown: %c%s\n", CMDLINE_PARSER_PACKAGE, c, (additional_error ? additional_error : ""));
          abort ();
        } /* switch */
    } /* while */


  update_multiple_arg((void *)&(args_info->query_arg),
    &(args_info->query_orig), args_info->query_given,
    local_args_info.query_given, 0,
    ARG_STRING, query_list);

  args_info->query_given += local_args_info.query_given;
  local_args_info.query_given = 0;
  
  if (check_required)
    {
      error_occurred += cmdline_parser_required2 (args_info, argv[0], additional_error);
    }

  cmdline_parser_release (&local_args_info);

  if ( error_occurred )
    return (EXIT_FAILURE);

  return 0;

failure:
  free_list (query_list, 1 );
  
  cmdline_parser_release (&local_args_info);
  return (EXIT_FAILURE);
}
/* vim: set ft=c noet ts=8 sts=8 sw=8 tw=80 nojs spell : */
