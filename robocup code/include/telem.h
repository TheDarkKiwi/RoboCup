
#ifndef TELEM_H_
#define TELEM_H_

extern bool debug;


void telem_serial_init(void);

void telem_read(void);

void execute(char* cmd);

#endif /* TELEM_H_ */