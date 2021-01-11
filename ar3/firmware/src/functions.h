#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "command.h"

#define NFUNC 7
void do_fn(const command_t& args, char* out);
int do_fn_complete();
void loop_fn();

#endif // FUNCTIONS_H
