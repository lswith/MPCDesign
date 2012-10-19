#include <stdio.h>
#include <unistd.h>	
#include <fcntl.h>
#include "/home/lswith/Documents/thesisrepo/ext2.h"
#include <string.h>
#include <rtai_shm.h>

int main()
{

    static struct {
        int time;
        double u,y,s,sf;
        double alength;
        double blength;
        double a[VECLEN];
        double b[VECLEN];
        int mode;
    } disp;
    FILE *fd;
    FILE *fd2;
    int j, oldmode;
    fd2 = fopen("out.dat","w+");
    fprintf(fd2,"time, u, y, s, sf, a, b\n");
    fclose(fd2);

    printf("\nstarted\n");
    while (1) {
        fd = fopen(DISPLAY_DEV, "r");
        if (fd == NULL) {
            fprintf(stderr, "error opening %s\n", DISPLAY_DEV);
            return 1;
        }
        if (fread(&disp, sizeof(disp),1,fd) != 0) {
            if (disp.time == -1) {
                printf("\nstopped\n");
                fclose(fd);
                break;
            }
            fd2 = fopen("out.dat", "a+");
            if (fd2 == NULL) {
                fprintf(stderr, "error opening out.dat\n");
                return 1;
            }
            fprintf(fd2,"%10d  %5.3f  %5.3f",disp.time,disp.u,disp.y);
            fprintf(fd2,"  %5.3f  %5.3f",disp.s,disp.sf);
            for (j = 0; j < disp.alength; j++) fprintf(fd2,"  %5.3f",disp.a[j]);
            for (j = 0; j < disp.blength; j++) fprintf(fd2,"  %5.3f",disp.b[j]);
            fprintf(fd2,"\n");
            printf("%10d  %5.3f  %5.3f",disp.time,disp.u,disp.y);
            printf("  %5.3f  %5.3f",disp.s,disp.sf);
            for (j = 0; j < disp.alength; j++) printf("  %5.3f",disp.a[j]);
            for (j = 0; j < disp.blength; j++) printf("  %5.3f",disp.b[j]);
            printf("\n");
            
            if (oldmode != disp.mode) {
                printf("\n current mode is %d\n",disp.mode);
                fprintf(fd2,"\n current mode is %d\n",disp.mode);
            }
            oldmode = disp.mode;
            fclose(fd2);
        }
        fclose(fd);
    }
    return 0;
}
