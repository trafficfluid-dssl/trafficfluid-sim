#ifdef CONTROLLER_H
#define EXTERN_C /* nothing */
#else
#define EXTERN_C extern
#endif /* DEFINE_VARIABLES */

//example header file for additional code files

//Define structures, functions that are relevant to Controller.cpp 




void example_function();

// this is how you can define static variables
EXTERN_C int example_extern_variable_static;

