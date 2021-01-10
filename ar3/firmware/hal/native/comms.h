#ifndef COMMS_H
#define COMMS_H

void initComms();

// Need to sendResponse() after every successful return from fetch
bool tryFetchCommand(char* buf, int buflen);
void sendResponse(char* buf, int buflen);

#endif // COMMS_H
