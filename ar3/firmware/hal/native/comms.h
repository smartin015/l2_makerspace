#ifndef COMMS_H
#define COMMS_H


namespace comms {

void init();
bool read(char* buf, int buflen);
void write(char* buf, int buflen);

} // namespace comms

#endif // COMMS_H
