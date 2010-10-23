#include <stdio.h>
#include <stdlib.h>
#include <X11/Xlib.h>

int main(char argc,char *argv[]){
    int x,y;
    Display *display;
    Window root;

    display = XOpenDisplay(0);
    root = XRootWindow(display,0);
   // struct input_dev *input = dev->input;
    if (argc != 3){
	    printf("usage: %s xcoordinate ycoordinate\n",argv[0]);
	    exit(0);
    }
    x=atoi(argv[1]);
    y=atoi(argv[2]);

    printf("xcoord = %d , ycoord = %d \n",x,y);
 
    XWarpPointer(display,None,root,0,0,0,0,x,y);
    
    XCloseDisplay(display);
    

   // input_report_abs(input, ABS_X, x);
   // input_report_abs(input, ABS_Y, y);
    return 0;
}
