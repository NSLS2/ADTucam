/* tucamAppMain.cpp */
/* Author(s): Jakub Wlodek, Thomas Hopkins */
/* Date:      May 9th, 2025 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"

int main(int argc, char *argv[]) {
  /* Start the IOC with the supplied arguments */
  if (argc >= 2) {
    iocsh(argv[1]);
    epicsThreadSleep(0.2);
  }

  /* Launch the interactive IOC shell */
  iocsh(NULL);

  /* Cleanup and exit */
  epicsExit(0);

  return (0);
}
