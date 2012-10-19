#include <stdio.h>
#include <unistd.h>	
#include <fcntl.h>
#include "/home/lswith/Documents/thesisrepo/ext2.h"
#include <string.h>
#include <rtai_shm.h>
#define BUFFERLEN 80


int *mode;
int *setpointmode;
double *sp;
double *samplerate;

void displayMode () {
    printf("*------------Current Mode-------------*\n");
    printf("MODE = ");
    switch (*mode) {
        case GAINFEEDBACK:
            printf("gain feedback control");
            break;
        case ESTIMATEONLY:
            printf("estimation only");
            break;
        case MPCCONTROL:
            printf("mpc estimation and control");
            break;
        case CONTROLONLY:
            printf("mpc control only");
            break;
        case MANUAL:
            printf("manual");
            break;
        default:
            printf("error mode not set properly");
            break;
    }
    printf("\n");

    printf("SETPOINT MODE = ");
    switch (*setpointmode) {
        case USERAND:
            printf("using random setpoint");
            break;
        case USESET:
            printf("using manual setpoint");
            break;
        default:
            printf("error setpointmode not set properly");
            break;
    }
    printf("\n");
    printf("The current Setpoint is %f\n",*sp);
}

void displayInstructions (void) {
    printf("*------------Instructions-------------*\n");
    printf("Set the mode: m\n");
    printf("Set the setpoint mode: sm\n");
    printf("Set the setpoint: sp\n");
    printf("Display the current Modes and Setpoint: dm\n");
    printf("Change the sampling rate of the controller: sa\n");
    printf("To quit: q\n");
}

void getMode(void) {
    int temp;
    char buffer[BUFFERLEN];
    printf("*--------------Change Mode------------*\n");
    printf("Possible Modes are:\n");
    printf("Estimate only = 0\n");
    printf("Mpc estimation and control = 1\n");
    printf("Mpc control only = 2\n");
    printf("Manual = 3\n");
    //printf("Gain Feedback Control = 4\n");
    printf("Which mode would you like (%d): ",*mode);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
        printf("error\n");
    } else {
        if (1 == sscanf(buffer, "%d", &temp)) {
            if (temp < 5 && temp >= 0) {
                *mode = temp;
            } else {
                printf("\nnot a valid mode\n");
            }
        } else {
            printf("\nnot a valid mode\n");
        }
    }
}

void getSetpointMode(void) {
    int temp;
    char buffer[BUFFERLEN];
    printf("*---------Change Setpoint Mode--------*\n");
    printf("Possible Modes are:\n");
    printf("Use random setpoint = 0\n");
    printf("Use manual setpoint = 1\n");
    printf("Which mode would you like (%d) : ",*setpointmode);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
        printf("error\n");
    } else {
        if (1 == sscanf(buffer, "%d", &temp)) {
            if (temp == 1 || temp == 0) {
                *setpointmode = temp;
            } else {
                printf("\nnot a valid setpointmode\n");
            }
        } else {
            printf("\nnot a valid setpointmode\n");
        }
    }
}

void getSetpoint (void) {
    double temp;
    char buffer[BUFFERLEN];
    printf("*------------Change Setpoint-----------*\n");
    if (*setpointmode == 0) {
        printf("You must change the setpoint mode to manual first\n");
    } else {
        printf("What would you like the setpoint to be? (%f): ",*sp);
        if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
            printf("error\n");
        } else {
            if (1 == sscanf(buffer, "%lf", &temp)) {
                *sp = temp;
                printf("Setpoint is now: %f\n",*sp);
            } else {
                printf("\nnot a valid setpoint\n");
            }

        }
    }
}

void getSampleRate(void ) {
    FILE *fd; 
    char buffer[BUFFERLEN];
    double temp;
    fd = fopen(SAMPLE_DEV, "w");
    printf("*----------Change Samplerate-----------*\n");
    printf("Please enter the sample rate you wish to use: (%f Hz) ",*samplerate);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
        printf("error\n");
    } else {
        if (1 == sscanf(buffer, "%lf", &temp)) {
            fwrite(&temp,sizeof(temp),1,fd); 
            printf("samplerate is now %fHz\n",temp);
        } else {
            printf("\nnot a valid samplerate\n");
        }
    }
    fclose(fd);
}

int main () {
    char buffer[BUFFERLEN];
    sp = rt_shm_alloc(nam2num("sp"),sizeof(double),USE_VMALLOC);
    mode = rt_shm_alloc(nam2num("mode"),sizeof(int),USE_VMALLOC);
    setpointmode = rt_shm_alloc(nam2num("spmode"),sizeof(int),USE_VMALLOC);
    samplerate = rt_shm_alloc(nam2num("samprt"),sizeof(double),USE_VMALLOC);
    while (! feof(stdin)) {
        displayInstructions();
        printf("\nWhat would you like to do? : ");
        fflush(stdout);
        if (NULL == fgets(buffer, BUFFERLEN, stdin)) break;

        if (! strncmp(buffer, "q", 1)) {
            printf("Quitting\n");
            break;
        } else if (! strncmp(buffer,"m",1)) {
            getMode(); 
            continue;
        } else if (! strncmp(buffer,"sm",2)) {
            getSetpointMode(); 
            continue;
        } else if (! strncmp(buffer,"sp",2)) {
            getSetpoint(); 
            continue;
        } else if (! strncmp(buffer,"sa",2)) {
            getSampleRate(); 
            continue;
        } else if (! strncmp(buffer,"dm",2)) {
            displayMode(); 
            continue;
        } else {
            printf("\nnot a valid command\n");
            continue;

        }
    }
    return 0;
}


