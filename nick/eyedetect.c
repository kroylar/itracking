#include <stdio.h>
#include "cv.h"
#include "highgui.h"

#define MINDIF 20
#define NUMFRAMES 7

//global variables
CvHaarClassifierCascade *cascade_f;
CvHaarClassifierCascade *cascade_e;
CvMemStorage			*storage;
CvMemStorage			*storage1[NUMFRAMES];

IplImage *img = 0, *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;
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
int vmin = 10, vmax = 256, smin = 30;


//function prototypes
CvRect* detectFaces(IplImage *);
CvRect* camshift_f(CvRect *, IplImage *);
void eyedetect(CvRect *r, IplImage *img );
CvScalar hsv2rgb( float );

int main()
{
    int i;
    CvCapture* capture = cvCaptureFromCAM(0);
    CvRect* face = NULL;
    char *file1 = "haarcascade_frontalface_alt.xml";
    char *file2 = "haarcascade_eye.xml";

    /* load the face classifier */
    cascade_f = (CvHaarClassifierCascade*)cvLoad(file1, 0, 0, 0);

    /* load the eye classifier */
    cascade_e = (CvHaarClassifierCascade*)cvLoad(file2, 0, 0, 0);

    /* setup memory storage, needed by the object detector */
    storage = cvCreateMemStorage(0);
    for (i = 0; i < NUMFRAMES; i++) {
        storage1[i] = cvCreateMemStorage(0);
    }

    while(1){

        /* load image */
        img = cvQueryFrame(capture);

        cvNamedWindow("EyeDetect", CV_WINDOW_AUTOSIZE);

        if( !image )
        {   
            /* allocate all the buffers */
            image = cvCreateImage( cvGetSize(img), 8, 3 );
            image->origin = img->origin;
            hsv = cvCreateImage( cvGetSize(img), 8, 3 );
            hue = cvCreateImage( cvGetSize(img), 8, 1 );
            mask = cvCreateImage( cvGetSize(img), 8, 1 );
            backproject = cvCreateImage( cvGetSize(img), 8, 1 );
            hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

            histimg = cvCreateImage( cvSize(320,200), 8, 3 );
            cvZero( histimg );
        }
        cvCopy(img, image, 0);
        cvCvtColor( image, hsv, CV_BGR2HSV );
        /* detect eyes and display image */
        if(face == NULL){
            face = detectFaces(image);
            cvShowImage("EyeDetect",image);
        }
        else {
            face = camshift_f(face,image);
            eyedetect(face, image);
            cvShowImage("EyeDetect", image);
        }
        if((cvWaitKey(10) & 255) == 27) break;
    }
    cvDestroyWindow("EyeDetect");
    cvReleaseCapture(&capture);

    for (i = 0; i < NUMFRAMES; i++)
        cvClearMemStorage(storage1[i]);

    return 0;
}


CvRect * detectFaces(IplImage *img)
{ 
    int i, j, k;
    static int ringpos = 0;
    static int face_count = 0;
    static int face_uncount = 0;
    static CvSeq *prev_faces[NUMFRAMES];
    int possface[10]={0,0,0,0,0,0,0,0,0,0}; // max 10 detected faces per frame
    int totalf;
    int diffx, diffy;
    CvSeq *faces;
    CvRect *r;

    /* detect faces */
    faces = cvHaarDetectObjects(
        img, cascade_f, storage,
        1.1, 3, 0, cvSize( 40, 40 ) );

    /* return if not found */
    if (faces->total == 0) { 
   /*     face_uncount++;
        if (face_count > 3 && face_uncount > 3) {
            face_count = 0;
        } 
        */
        return NULL;
    }

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
        if (possface[i] >= 1) {
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

void eyedetect(CvRect *r, IplImage *img )
{
    int i;

    /* Set the Region of Interest: estimate the eyes' position */
    cvSetImageROI(img, cvRect(r->x, r->y + (r->height/5.5), r->width, r->height/3.0));

    /* detect eyes */
	CvSeq* eyes = cvHaarDetectObjects( 
        img, cascade_e, storage,
		1.15, 3, 0, cvSize(25, 15));

    /* draw a rectangle for each eye found */
	for( i = 0; i < (eyes ? eyes->total : 0); i++ ) {
		CvRect *r1 = (CvRect*)cvGetSeqElem( eyes, i );
		cvCircle(img, cvPoint((r1->x + 20), (r1->y + 20)), 10, CV_RGB(0, 0, 255), 3, 8, 0);
           printf("point %d x = %d, y = %d\n",i,r1->x + r->x + 20,r1->y+ r->x +20);
	}

    cvResetImageROI(img);
}


