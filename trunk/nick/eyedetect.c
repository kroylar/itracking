#include <stdio.h>
#include "cv.h"
#include "highgui.h"

CvHaarClassifierCascade *cascade_f;
CvHaarClassifierCascade *cascade_e;
CvMemStorage			*storage;

void detectEyes(IplImage *img);

int main()
{
    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY);

    IplImage *img;
    char *file1 = "../../data/haarcascades/haarcascade_frontalface_alt.xml";
    char *file2 = "../../data/haarcascades/haarcascade_eye.xml";

    /* load the face classifier */
    cascade_f = (CvHaarClassifierCascade*)cvLoad(file1, 0, 0, 0);

    /* load the eye classifier */
    cascade_e = (CvHaarClassifierCascade*)cvLoad(file2, 0, 0, 0);

    /* setup memory storage, needed by the object detector */
    storage = cvCreateMemStorage(0);

    while(1){
        /* load image */
        img = cvQueryFrame(capture);


        cvNamedWindow("EyeDetect", CV_WINDOW_AUTOSIZE);

        /* detect eyes and display image */
        detectEyes(img);
        cvShowImage("EyeDetect", img);

        if((cvWaitKey(10) & 255) == 27) break;
    }
    cvDestroyWindow("EyeDetect");
    cvReleaseCapture(&capture);

    return 0;
}

void detectEyes(IplImage *img)
{
	int i;

    /* detect faces */
	CvSeq *faces = cvHaarDetectObjects(
		img, cascade_f, storage,
		1.1, 3, 0, cvSize( 40, 40 ) );

    /* return if not found */
    if (faces->total == 0) return;

    /* draw a rectangle */
	CvRect *r = (CvRect*)cvGetSeqElem(faces, 0);
	cvRectangle(img,
				cvPoint(r->x, r->y),
				cvPoint(r->x + r->width, r->y + r->height),
				CV_RGB(255, 0, 0), 1, 8, 0);

    /* reset buffer for the next object detection */
    cvClearMemStorage(storage);

    /* Set the Region of Interest: estimate the eyes' position */
    cvSetImageROI(img, cvRect(r->x, r->y + (r->height/5.5), r->width, r->height/3.0));

    /* detect eyes */
	CvSeq* eyes = cvHaarDetectObjects( 
        img, cascade_e, storage,
		1.15, 3, 0, cvSize(25, 15));

    /* draw a rectangle for each eye found */
	for( i = 0; i < (eyes ? eyes->total : 0); i++ ) {
		CvRect *r1 = (CvRect*)cvGetSeqElem( eyes, i );
		cvCircle(img, cvPoint((r1->x + 20), (r1->y + 20)), 10, CV_RGB(0, 0, 255), 1, 8, 0);
           printf("point %d x = %d, y = %d\n",i,r1->x + r->x + 20,r1->y+ r->x +20);
	}

    cvResetImageROI(img);
}

