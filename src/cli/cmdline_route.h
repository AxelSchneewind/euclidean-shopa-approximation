/** @file cmdline_route.h
 *  @brief The header file for the command line option parser
 *  generated by GNU Gengetopt version 2.23
 *  http://www.gnu.org/software/gengetopt.
 *  DO NOT modify this file, since it can be overwritten
 *  @author GNU Gengetopt */

#ifndef CMDLINE_ROUTE_H
#define CMDLINE_ROUTE_H

/* If we use autoconf.  */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h> /* for FILE */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef CMDLINE_PARSER_PACKAGE
/** @brief the program name (used for printing errors) */
#define CMDLINE_PARSER_PACKAGE "Routing"
#endif

#ifndef CMDLINE_PARSER_PACKAGE_NAME
/** @brief the complete program name (used for help and version) */
#define CMDLINE_PARSER_PACKAGE_NAME "Routing"
#endif

#ifndef CMDLINE_PARSER_VERSION
/** @brief the program version */
#define CMDLINE_PARSER_VERSION "1.0"
#endif

/** @brief Where the command line options are stored */
struct gengetopt_args_info
{
  const char *help_help; /**< @brief Print help and exit help description.  */
  const char *version_help; /**< @brief Print version and exit help description.  */
  char * graph_file_arg;	/**< @brief path to graph file (of type .fmi or .graph).  */
  char * graph_file_orig;	/**< @brief path to graph file (of type .fmi or .graph) original value given at command line.  */
  const char *graph_file_help; /**< @brief path to graph file (of type .fmi or .graph) help description.  */
  char * output_directory_arg;	/**< @brief path to output directory (where the subdirectory with the output files should be created) (default='.').  */
  char * output_directory_orig;	/**< @brief path to output directory (where the subdirectory with the output files should be created) original value given at command line.  */
  const char *output_directory_help; /**< @brief path to output directory (where the subdirectory with the output files should be created) help description.  */
  double epsilon_arg;	/**< @brief ε value to use for discretizing the triangulation (if a .graph file is given) (default='0.5').  */
  char * epsilon_orig;	/**< @brief ε value to use for discretizing the triangulation (if a .graph file is given) original value given at command line.  */
  const char *epsilon_help; /**< @brief ε value to use for discretizing the triangulation (if a .graph file is given) help description.  */
  char ** query_arg;	/**< @brief pair(s) of source and destination node ids.  */
  char ** query_orig;	/**< @brief pair(s) of source and destination node ids original value given at command line.  */
  unsigned int query_min; /**< @brief pair(s) of source and destination node ids's minimum occurreces */
  unsigned int query_max; /**< @brief pair(s) of source and destination node ids's maximum occurreces */
  const char *query_help; /**< @brief pair(s) of source and destination node ids help description.  */
  int coordinates_flag;	/**< @brief interpret the pair(s) of source and destination nodes as their coordinates (default=off).  */
  const char *coordinates_help; /**< @brief interpret the pair(s) of source and destination nodes as their coordinates help description.  */
  int stdin_flag;	/**< @brief indicates that queries should be read from stdin (default=off).  */
  const char *stdin_help; /**< @brief indicates that queries should be read from stdin help description.  */
  int csv_format_flag;	/**< @brief indicates that routing information should be printed in the csv format (default=off).  */
  const char *csv_format_help; /**< @brief indicates that routing information should be printed in the csv format help description.  */
  const char *project_help; /**< @brief which projection to apply to coordinates when writing to files (from google_bing,wgs84,none) help description.  */
  
  unsigned int help_given ;	/**< @brief Whether help was given.  */
  unsigned int version_given ;	/**< @brief Whether version was given.  */
  unsigned int graph_file_given ;	/**< @brief Whether graph-file was given.  */
  unsigned int output_directory_given ;	/**< @brief Whether output-directory was given.  */
  unsigned int epsilon_given ;	/**< @brief Whether epsilon was given.  */
  unsigned int query_given ;	/**< @brief Whether query was given.  */
  unsigned int coordinates_given ;	/**< @brief Whether coordinates was given.  */
  unsigned int stdin_given ;	/**< @brief Whether stdin was given.  */
  unsigned int csv_format_given ;	/**< @brief Whether csv-format was given.  */
  unsigned int project_given ;	/**< @brief Whether project was given.  */

} ;

/** @brief The additional parameters to pass to parser functions */
struct cmdline_parser_params
{
  int override; /**< @brief whether to override possibly already present options (default 0) */
  int initialize; /**< @brief whether to initialize the option structure gengetopt_args_info (default 1) */
  int check_required; /**< @brief whether to check that all required options were provided (default 1) */
  int check_ambiguity; /**< @brief whether to check for options already specified in the option structure gengetopt_args_info (default 0) */
  int print_errors; /**< @brief whether getopt_long should print an error message for a bad option (default 1) */
} ;

/** @brief the purpose string of the program */
extern const char *gengetopt_args_info_purpose;
/** @brief the usage string of the program */
extern const char *gengetopt_args_info_usage;
/** @brief the description string of the program */
extern const char *gengetopt_args_info_description;
/** @brief all the lines making the help output */
extern const char *gengetopt_args_info_help[];

/**
 * The command line parser
 * @param argc the number of command line options
 * @param argv the command line options
 * @param args_info the structure where option information will be stored
 * @return 0 if everything went fine, NON 0 if an error took place
 */
int cmdline_parser (int argc, char **argv,
  struct gengetopt_args_info *args_info);

/**
 * The command line parser (version with additional parameters - deprecated)
 * @param argc the number of command line options
 * @param argv the command line options
 * @param args_info the structure where option information will be stored
 * @param override whether to override possibly already present options
 * @param initialize whether to initialize the option structure my_args_info
 * @param check_required whether to check that all required options were provided
 * @return 0 if everything went fine, NON 0 if an error took place
 * @deprecated use cmdline_parser_ext() instead
 */
int cmdline_parser2 (int argc, char **argv,
  struct gengetopt_args_info *args_info,
  int override, int initialize, int check_required);

/**
 * The command line parser (version with additional parameters)
 * @param argc the number of command line options
 * @param argv the command line options
 * @param args_info the structure where option information will be stored
 * @param params additional parameters for the parser
 * @return 0 if everything went fine, NON 0 if an error took place
 */
int cmdline_parser_ext (int argc, char **argv,
  struct gengetopt_args_info *args_info,
  struct cmdline_parser_params *params);

/**
 * Save the contents of the option struct into an already open FILE stream.
 * @param outfile the stream where to dump options
 * @param args_info the option struct to dump
 * @return 0 if everything went fine, NON 0 if an error took place
 */
int cmdline_parser_dump(FILE *outfile,
  struct gengetopt_args_info *args_info);

/**
 * Save the contents of the option struct into a (text) file.
 * This file can be read by the config file parser (if generated by gengetopt)
 * @param filename the file where to save
 * @param args_info the option struct to save
 * @return 0 if everything went fine, NON 0 if an error took place
 */
int cmdline_parser_file_save(const char *filename,
  struct gengetopt_args_info *args_info);

/**
 * Print the help
 */
void cmdline_parser_print_help(void);
/**
 * Print the version
 */
void cmdline_parser_print_version(void);

/**
 * Initializes all the fields a cmdline_parser_params structure 
 * to their default values
 * @param params the structure to initialize
 */
void cmdline_parser_params_init(struct cmdline_parser_params *params);

/**
 * Allocates dynamically a cmdline_parser_params structure and initializes
 * all its fields to their default values
 * @return the created and initialized cmdline_parser_params structure
 */
struct cmdline_parser_params *cmdline_parser_params_create(void);

/**
 * Initializes the passed gengetopt_args_info structure's fields
 * (also set default values for options that have a default)
 * @param args_info the structure to initialize
 */
void cmdline_parser_init (struct gengetopt_args_info *args_info);
/**
 * Deallocates the string fields of the gengetopt_args_info structure
 * (but does not deallocate the structure itself)
 * @param args_info the structure to deallocate
 */
void cmdline_parser_free (struct gengetopt_args_info *args_info);

/**
 * Checks that all the required options were specified
 * @param args_info the structure to check
 * @param prog_name the name of the program that will be used to print
 *   possible errors
 * @return
 */
int cmdline_parser_required (struct gengetopt_args_info *args_info,
  const char *prog_name);


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* CMDLINE_ROUTE_H */
