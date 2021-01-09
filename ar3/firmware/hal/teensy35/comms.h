#ifndef COMMS_H
#define COMMS_H

void initComms();

bool tryFetchCommand(char* buf, size_t buflen);

#endif // COMMS_H
