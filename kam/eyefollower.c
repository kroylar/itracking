/*********************************************************
 * Simple opencv facetracker with dual camera support
 *
 * -KRL
 *  *****************************************************/
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "dc1394/control.h"
#include "dc1394/dc1394.h"
#include <termios.h>
#include <fcntl.h>

// boundary constants found using seperate "boundary" program
#define PWMYMAX 1602
#define PWMYMIN 1339
#define PWMXMAX 1570
#define PWMXMIN 1192
#define DELAY 50000

// constants for detections
#define MINDIF 20
#define NUMFRAMES 7
#define NEEDEDFRAMES 4
#define NEEDEDEFRAMES 6

// serial stuff
#define BAUDRATE B9600            
#define MODEMDEVICE "/dev/ttyUSB5"

// serial
struct termios oldtio;
int fd;
int sent = 0;

//eyedetect stuff
CvPoint eye;
CvHaarClassifierCascade *cascade_f;
CvHaarClassifierCascade *cascade_e;
CvMemStorage			*storage;
CvMemStorage			*storage1[NUMFRAMES];

IplImage *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;
CvHistogram *hist = 0;

int backproject_mode = 0;
int select_object = 0;
int track_object = -1;
int show_hist = 1;
CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims = 16;
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 150, vmax = 256, smin = 30;


void init_uart(void);
void restore_uart(void);
void sendcamera(CvPoint pt);

CvRect* detectFaces(IplImage *);
CvRect* camshift_f(CvRect *, IplImage *);
CvPoint* eyedetect(CvRect *r, IplImage *img);
CvScalar hsv2rgb( float );

int main( int argc, char** argv ) {
    IplImage *frame3;
    int i;
    CvRect* face = NULL;
    char *file1 = "haarcascade_frontalface_alt.xml";
    char *file2 = "haarcascade_eye.xml";
#if DEBUG
    int p[3];
    p[0] = CV_IMWRITE_JPEG_QUALITY;
    p[1] = 95;
    p[2] = 0;
#endif

    /* load the face classifier */
    cascade_f = (CvHaarClassifierCascade*)cvLoad(file1, 0, 0, 0);

    /* load the eye classifier */
    cascade_e = (CvHaarClassifierCascade*)cvLoad(file2, 0, 0, 0);

    /* allocate the memory storage, needed by the object detector */
    storage = cvCreateMemStorage(0);
    for (i = 0; i < NUMFRAMES; i++) {
        storage1[i] = cvCreateMemStorage(0);
    }

    // initialize the UART
    init_uart();

    // firewire stuff
    // clear DMA ring buffer stop ISO transmission
    dc1394_t* dcenv = dc1394_new();
    dc1394error_t err;
    dc1394video_frame_t* tmpframe[10];
    // TODO: how do I know how many lists will be in this array?
    dc1394camera_list_t* camlist[1];
    dc1394_camera_enumerate(dcenv, camlist);

    dc1394camera_t* camera[camlist[0]->num];
    for (i = 0; i < camlist[0]->num; i++) {
        camera[i] = dc1394_camera_new_unit(dcenv,camlist[0]->ids[i].guid, camlist[0]->ids[i].unit);
    }

    dc1394_camera_free_list(camlist[0]);

    // TODO: only stop if its been started    
    dc1394_capture_stop(camera[0]);
    dc1394_capture_stop(camera[1]);
    
    dc1394_reset_bus(camera[0]);
    // TODO: figure out how to actually clear the DMA ring buffer.
#if TODO
    err = dc1394_capture_dequeue(camera[0],DC1394_CAPTURE_POLICY_POLL, tmpframe);
    DC1394_ERR_RTN(err, "failed to dequeue\n");
    while (dc1394_capture_dequeue(camera[0],DC1394_CAPTURE_POLICY_POLL, tmpframe) != NULL);
    do {
        dc1394_capture_enqueue(camera[1],tmpframe[0]);
    } while (dc1394_capture_dequeue(camera[1],DC1394_CAPTURE_POLICY_POLL, tmpframe) != NULL);

    dc1394video_modes_t* modes;
    if (dc1394_video_get_supported_modes((camlist[0])->camera_info,modes) < 0) {
        printf("Could not query supported formats\n");
        exit(1);
    }
    printf("video modes: ");
    for ( int i = 0; i < modes->num; i++ ) {
        printf(" %d,", modes->modes[i]);
    }
#endif
    cvNamedWindow("side-by-side", CV_WINDOW_AUTOSIZE);

    CvCapture* capture1;
    CvCapture* capture2;
    if (argc == 1) {
        capture1 = cvCaptureFromCAM(1);
        capture2 = cvCaptureFromCAM(0);
    } else {
        capture1 = cvCaptureFromFile( argv[1] );
        capture2 = cvCaptureFromFile( argv[2] );
    }
    IplImage* zoom = cvQueryFrame( capture1 );
    IplImage* zoomg = cvCreateImage(cvGetSize(zoom), IPL_DEPTH_8U, 1);
    IplImage* zsmallg = cvCreateImage(cvSize((zoom->width)/5.2, (zoom->height)/5.2), IPL_DEPTH_8U, 1);
    IplImage* zsmall = cvCreateImage(cvGetSize(zoom), IPL_DEPTH_8U, 3);
    IplImage* wide = cvQueryFrame( capture2 );
    IplImage* wideg = cvCreateImage(cvGetSize(wide), IPL_DEPTH_8U, 1);
    frame3 = cvCreateImage(cvSize(zoom->width+wide->width, zoom->height), IPL_DEPTH_8U, 3);

    while(1) {
        wide = cvQueryFrame( capture1 );
        zoom = cvQueryFrame( capture2 );
        if( !zoom || !wide ) break;

        if( !image ) {   
            /* allocate all the buffers */
            image = cvCreateImage( cvGetSize(wide), 8, 3 );
            image->origin = wide->origin;
            hsv = cvCreateImage( cvGetSize(wide), 8, 3 );
            hue = cvCreateImage( cvGetSize(wide), 8, 1 );
            mask = cvCreateImage( cvGetSize(wide), 8, 1 );
            backproject = cvCreateImage( cvGetSize(wide), 8, 1 );
            hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

            histimg = cvCreateImage( cvSize(320,200), 8, 3 );
            cvZero( histimg );
        }

        cvCopy(wide, image, NULL);
        cvCvtColor( image, hsv, CV_BGR2HSV );

        /* detect eyes and display image */
        if(face == NULL) {
            face = detectFaces(image);
        } else {
            face = camshift_f(face,image);
            eyedetect(face, image);
        }

        sendcamera(eye);

        cvSetImageROI(frame3, cvRect(0,0,640,480));
        cvCopy(image, frame3, NULL);
        cvSetImageROI(frame3, cvRect(640,0,640,480));
        cvCopy(zoom, frame3, NULL);
        cvResetImageROI(frame3);
        cvShowImage("side-by-side", frame3);

        char c = cvWaitKey(33);
        if (c == 27) {
            break;
        }
#if DEBUG
        switch(c) {
            case 83:
            case 115:
                stringstream ss;
                ss << time(NULL);
                cvSaveImage((file1 + ss.str() + ".jpg").c_str(), zoomg, p);
                cvSaveImage((file2 + ss.str() + ".jpg").c_str(), wideg, p);
        }
#endif
    }
    cvReleaseCapture( &capture1 );
    cvReleaseCapture( &capture2 );

    cvDestroyWindow( "side-by-side" );

    for (i = 0; i < NUMFRAMES; i++)
        cvClearMemStorage(storage1[i]);

    dc1394_camera_set_power(camera[0], DC1394_OFF);
    dc1394_camera_set_power(camera[1], DC1394_OFF);

    dc1394_camera_free(camera[0]);
    dc1394_camera_free(camera[1]);
    dc1394_free(dcenv);

    restore_uart();
}

void init_uart(void) {
    struct termios newtio;
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
    if (fd <0) {perror(MODEMDEVICE); exit(-1); }
    tcgetattr(fd,&oldtio); /* save current serial port settings */
    memset(&newtio, 0, sizeof(newtio)); /* clear struct for new port settings */
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | HUPCL | CREAD;
    newtio.c_iflag = IGNBRK;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 5;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */
    newtio.c_ispeed = 0xd;
    newtio.c_ospeed = 0xd;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
}



void restore_uart(void) {
    tcsetattr(fd,TCSANOW,&oldtio);
}

void sendcamera(CvPoint pt) {
    char tmp[10];
    int pwmx;
    int pwmy;
    
    pwmx = PWMXMIN + (PWMXMAX-PWMXMIN)*((double)(640-pt.x)/(double)640);
    pwmy = PWMYMIN + (PWMYMAX-PWMYMIN)*((double)(480-pt.y)/(double)480);

    //printf("pwmx = %d\n", pwmx);
    //printf("pwmy = %d\n", pwmy);

    sprintf(tmp,"%d\n",pwmx);
    write(fd,tmp,5);
    usleep(DELAY);
    sprintf(tmp,"%d\n",pwmy);
    write(fd,tmp,5);
    usleep(DELAY);
}

CvRect * detectFaces(IplImage *img)
{ 
    int i, j, k;
    static int ringpos = 0;
    static CvSeq *prev_faces[NUMFRAMES];
    int possface[10]={0,0,0,0,0,0,0,0,0,0}; // max 10 detected faces per frame
    int diffx, diffy;
    CvSeq *faces;
    CvRect *r;

    /* detect faces */
    faces = cvHaarDetectObjects(
        img, cascade_f, storage,
        1.1, 3, 0, cvSize( 40, 40 ) );

    /* return if not found */
    if (faces->total == 0) return NULL;

    for (k = ringpos; k > 0; k--) {
        prev_faces[k] = cvCloneSeq(prev_faces[k-1], storage1[k]);
    }
    prev_faces[0] = cvCloneSeq(faces, storage1[0]);

    for (i = 0; i < faces->total; i++) {
        CvRect *cur = (CvRect*)cvGetSeqElem(faces, i);
        for (k = 0; k < ringpos; k++) { 
            for (j = 0; j < prev_faces[k]->total; j++) {
                CvRect *prev = (CvRect*)cvGetSeqElem(prev_faces[k], j);
                if (cur->x > prev->x)
                    diffx = cur->x - prev->x;
                else
                    diffx = prev->x - cur->x;

                if (cur->y > prev->y)
                    diffy = cur->y - prev->y;
                else
                    diffy = prev->y - cur->y;

                if (diffx < MINDIF && diffy < MINDIF){
                    possface[i]++;
                }
            }
        }
    }

    for (i = 0; i < faces->total; i++) {
        if (possface[i] >= NEEDEDFRAMES) {
            /* draw a rectangle */
            r = (CvRect*)cvGetSeqElem(faces, i);
            cvRectangle(img,
                        cvPoint(r->x, r->y),
                        cvPoint(r->x + r->width, r->y + r->width),
                        CV_RGB(255, 0, 0), 3, 8, 0);
            return r;
        }
    }

    if (ringpos < 4){
        ringpos++;
    }
    /* reset buffer for the next object detection */
    cvClearMemStorage(storage);
    return NULL;
}

CvRect* camshift_f(CvRect *face, IplImage * img)
{
        int _vmin = vmin, _vmax = vmax;
        static int bin_w, i;

        cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
        cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
        cvSplit( hsv, hue, 0, 0, 0 );

        if( track_object < 0 )
        {
            float max_val = 0.f;
            cvSetImageROI( hue, *face );
            cvSetImageROI( mask, *face );
            cvCalcHist( &hue, hist, 0, mask );
            cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
            cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
            cvResetImageROI( hue );
            cvResetImageROI( mask );
            track_window = *face;
            track_object = 1;

            bin_w = histimg->width / hdims;
            for( i = 0; i < hdims; i++ )
            {
                int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
                CvScalar color = hsv2rgb(i*180.f/hdims);
                cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                             cvPoint((i+1)*bin_w,histimg->height - val),
                             color, -1, 8, 0 );
            }
        }

        cvCalcBackProject( &hue, backproject, hist );
        cvAnd( backproject, mask, backproject, 0 );
        cvCamShift( backproject, track_window,
                    cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
                    &track_comp, &track_box );
        track_window = track_comp.rect;

        if( backproject_mode )
            cvCvtColor( backproject, image, CV_GRAY2BGR );
        if( !image->origin )
            track_box.angle = -track_box.angle;
        cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
        return &track_window;

}

CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

CvPoint* eyedetect(CvRect *r, IplImage *img )
{
    int i, j, k;
    static int ringpos = 0;
    static CvSeq *prev_eyes[NUMFRAMES];
    int posseye[10]={0,0,0,0,0,0,0,0,0,0}; // max 10 detected faces per frame
    int diffx, diffy;
    CvSeq *eyes;
    CvRect *r1;

    if (r->width > 10 && r->height > 10) {
        /* Set the Region of Interest: estimate the eyes' position */
        cvSetImageROI(img, *r);

        /* detect eyes */
        eyes = cvHaarDetectObjects( 
            img, cascade_e, storage,
            1.15, 3, 0, cvSize(25, 15));

        for (k = ringpos; k > 0; k--) {
            prev_eyes[k] = cvCloneSeq(prev_eyes[k-1], storage1[k]);
        }
        prev_eyes[0] = cvCloneSeq(eyes, storage1[0]);

        for (i = 0; i < eyes->total; i++) {
            CvRect *cur = (CvRect*)cvGetSeqElem(eyes, i);
            for (k = 0; k < ringpos; k++) { 
                for (j = 0; j < prev_eyes[k]->total; j++) {
                    CvRect *prev = (CvRect*)cvGetSeqElem(prev_eyes[k], j);
                    if (cur->x > prev->x)
                        diffx = cur->x - prev->x;
                    else
                        diffx = prev->x - cur->x;

                    if (cur->y > prev->y)
                        diffy = cur->y - prev->y;
                    else
                        diffy = prev->y - cur->y;

                    if (diffx < MINDIF && diffy < MINDIF){
                        posseye[i]++;
                    }
                }
            }
        }

        /* draw a circle for each eye found */
#if 1
        for (i = 0; i < eyes->total; i++) {
            if (posseye[i] >= NEEDEDEFRAMES) {
                r1 = (CvRect*)cvGetSeqElem(eyes, i);
                eye = cvPoint((r1->x + r->x), (r1->y + r->y));
                cvCircle(img, eye, r1->width/3, CV_RGB(0, 0, 255), 3, 8, 0);
                printf("x = %d, y = %d\n", eye.x, eye.y);
                //cvClearMemStorage(storage);
                //return &eye;
            }
        }
#endif
        if (ringpos < 4){
            ringpos++;
        }
        /* reset buffer for the next object detection */
        cvClearMemStorage(storage);

        cvResetImageROI(img);
        return NULL; 
    }
}

