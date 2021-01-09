#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "command.h"

#define NFUNC 7
void fn_wait_time(const command_t& args);
void fn_get_pos(const command_t& args);
void fn_calibrate_enc(const command_t& args);
void fn_drive_to_limits(const command_t& args);
void fn_move_j(const command_t& args);
void fn_move_l(const command_t& args);
void fn_move_c(const command_t& args);
bool do_fn(const command_t& args);

#endif // FUNCTIONS_H
