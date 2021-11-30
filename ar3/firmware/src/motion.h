#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define LIMIT_CHECK_TICK_PD 200

namespace motion {

void init();
void write();
void read();
void zero();
bool update();
void intent_changed();
void print_state();
void print_pid_stats();

} // namespace motion

#endif // FUNCTIONS_H
