/*

gpReadMouseTest.c

Test piped communication and mouse readback with gnuplot.
This code is published under the GNU Public License.

This example demonstrates how to wait for a mouse press and get the
mouse button number and mouse position, from a graph in gnuplot.


Is this still true? Maybe no more necessary:
The weird thing is that the FIFO apparently wants to receive an
initial linefeed from gnuplot before it works. If you uncomment the
one line designated below, it still _may_ work (with emphasis on
"may"). Any ideas why?

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#define wait (printf("Press the 'any' key\n"), fgets(buf, 10, stdin));
#define GPFIFO "./gpio" /* any unique name */

int
main(int argc, char *argv[])
{
FILE  *gp, *gpin;
float mx = 0.0, my = 0.0;
int   mb = 0;
int   n;
char  buf[80];

/* Create a FIFO we later use for communication gnuplot => our program. */
if (mkfifo(GPFIFO, 0600)) {
    if (errno != EEXIST) {
	perror(GPFIFO);
	unlink(GPFIFO);
	return 1;
    }
}

if (NULL == (gp = popen("gnuplot","w"))) {
    perror("gnuplot");
    pclose(gp);
    return 1;
}
puts("Connected to gnuplot.\n");

/* Init mouse and redirect all gnuplot printings to our FIFO */
fprintf(gp, "set mouse; set print \"%s\"\n", GPFIFO);
fflush(gp);

/* Sometimes it was necessary to print \n from gnuplot to avoid a block.
   Probably it is no more necessary.
*/
#if 0
fprintf(gp, "print \"\\n\"\n");
fflush(gp);
#endif

/* Open the FIFO (where gnuplot is writing to) for reading. */
if (NULL == (gpin = fopen(GPFIFO,"r"))) {
    perror(GPFIFO);
    pclose(gp);
    return 1;
}

puts("FIFO open for reading.\n");


/* Now do the work. */
fprintf(gp, "plot sin(x)\n");
fflush(gp);

/* Do it 5 times. */
for (n=0; n<5; n++) {
    printf("\n%i/5. -- Click anywhere in the graph by mouse button 1.\n", n+1);
    fprintf(gp, "pause mouse 'Click mouse!'\n");
    fflush(gp);

    fprintf(stdout,"I'M HERE: %i\n", __LINE__);
    /* Let gnuplot write to coordinates values (to the FIFO). */
    fprintf(gp, "print MOUSE_X, MOUSE_Y, MOUSE_BUTTON\n");
    fflush(gp);

    fprintf(stdout,"I'M HERE: %i\n", __LINE__);
    /* Read from the FIFO. */
    fscanf(gpin, "%f %f %d", &mx, &my, &mb);
    fprintf(stdout,"I'M HERE: %i\n", __LINE__);
    printf("You pressed mouse button %d at x=%f y=%f\n", mb, mx, my);
/*  wait; */
}

fclose(gpin);
pclose(gp);
unlink (GPFIFO);
return 0;
}

/* eof gpReadMouseTest.c */
