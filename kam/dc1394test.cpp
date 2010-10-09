#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "dc1394/control.h"
#include "dc1394/dc1394.h"
#include <iostream>
#include <string>
#include <time.h>
using namespace std;

// Create memory for calculations
static CvMemStorage* storage = 0;

// Create a new Haar classifier
static CvHaarClassifierCascade* cascade = 0;

// Function prototype for detecting and drawing an object from an image
void detect_and_draw( IplImage* image, int do_pyramids );

// Create a string that contains the cascade name
const char* cascade_name =
    "haarcascade_frontalface_alt.xml";
/*    "haarcascade_profileface.xml";*/

int main( int argc, char** argv ) {

    // Load the HaarClassifierCascade
    cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );

    // Check whether the cascade has loaded successfully. Else report and error and quit
    if( !cascade )
    {
        fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
        return -1;
    }

    // Allocate the memory storage
    storage = cvCreateMemStorage(0);

    // firewire stuff
    // clear DMA ring buffer stop ISO transmission
    dc1394_t* dcenv = dc1394_new();
    dc1394error_t err;
    dc1394video_frame_t* tmpframe[10];
    // TODO: how do I know how many lists will be in this array?
    dc1394camera_list_t* camlist[1];
    dc1394_camera_enumerate(dcenv, camlist);

    dc1394camera_t* camera[camlist[0]->num];
    for (int i = 0; i < camlist[0]->num; i++) {
        camera[i] = dc1394_camera_new_unit(dcenv,camlist[0]->ids[i].guid, camlist[0]->ids[i].unit);
    }

    dc1394_camera_free_list(camlist[0]);

    // TODO: only stop if its been started    
    dc1394_capture_stop(camera[0]);
    dc1394_capture_stop(camera[1]);
    
    dc1394_reset_bus(camera[0]);
    // TODO: figure out how to actually clear the DMA ring buffer.
//    err = dc1394_capture_dequeue(camera[0],DC1394_CAPTURE_POLICY_POLL, tmpframe);
//    DC1394_ERR_RTN(err, "failed to dequeue\n");
//    while (dc1394_capture_dequeue(camera[0],DC1394_CAPTURE_POLICY_POLL, tmpframe) != NULL);
//    do {
//        dc1394_capture_enqueue(camera[1],tmpframe[0]);
//    } while (dc1394_capture_dequeue(camera[1],DC1394_CAPTURE_POLICY_POLL, tmpframe) != NULL);
/*
    dc1394video_modes_t* modes;
    if (dc1394_video_get_supported_modes((camlist[0])->camera_info,modes) < 0) {
        printf("Could not query supported formats\n");
        exit(1);
    }
    printf("video modes: ");
    for ( int i = 0; i < modes->num; i++ ) {
        printf(" %d,", modes->modes[i]);
    }
*/
    cvNamedWindow("side-by-side", CV_WINDOW_AUTOSIZE);
    //cvNamedWindow( "Wide FOV", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "Zoom", CV_WINDOW_AUTOSIZE );

    CvCapture* capture1;
    CvCapture* capture2;
    if (argc == 1) {
        capture1 = cvCaptureFromCAM(0);
        capture2 = cvCaptureFromCAM(1);
    } else {
        capture1 = cvCaptureFromFile( argv[1] );
        capture2 = cvCaptureFromFile( argv[1] );
    }
    IplImage* frame1 = cvQueryFrame( capture1 );
    IplImage* frame2 = cvQueryFrame( capture2 );
    IplImage* frame3 = cvCreateImage(cvSize(frame1->width + frame2->width, frame1->height), frame1->depth, frame1->nChannels);
    IplImage* mask1 = cvCreateImage(cvSize(1280,480), IPL_DEPTH_8U, 1);
    IplImage* mask2 = cvCreateImage(cvSize(1280,480), IPL_DEPTH_8U, 1);
    //cvSetImageROI(mask1, cvRect(0,0,640,480));
    //cvSet(mask1, cvScalar(0xff));
    //cvResetImageROI(mask1);
    //cvNot(mask1, mask2);
    cvSetImageROI(mask1, cvRect(0,0,640,480));
    cvSetImageROI(mask2, cvRect(640,0,640,480));
    int p[3];
    p[0] = CV_IMWRITE_JPEG_QUALITY;
    p[1] = 95;
    p[2] = 0;
    string file1 ("saves/Wide");
    string file2 ("saves/Zoom");

    while(1) {
        frame1 = cvQueryFrame( capture1 );
        detect_and_draw(frame1, 1);
        frame2 = cvQueryFrame( capture2 );
        if( !frame1 || !frame2 ) break;
        //cvShowImage( "Wide FOV", frame1 );
        //cvShowImage( "Zoom", frame2 );
        cvSetImageROI(frame3, cvRect(0,0,640,480));
        cvCopy(frame1, frame3);
        cvSetImageROI(frame3, cvRect(640,0,640,480));
        cvCopy(frame2, frame3);
        cvResetImageROI(frame3);
        cvShowImage("side-by-side", frame3);
        char c = cvWaitKey(3);
        if (c == 27) {
            break;
        }

        switch(c) {
            case 83:
            case 115:
                stringstream ss;
                ss << time(NULL);
                cvSaveImage((file1 + ss.str() + ".jpg").c_str(), frame1, p);
                cvSaveImage((file2 + ss.str() + ".jpg").c_str(), frame2, p);
        }
    }
    cvReleaseCapture( &capture1 );
    cvReleaseCapture( &capture2 );
    //cvDestroyWindow( "Wide FOV" );
    //cvDestroyWindow( "Zoom" );

    dc1394_camera_set_power(camera[0], DC1394_OFF);
    dc1394_camera_set_power(camera[1], DC1394_OFF);

    dc1394_camera_free(camera[0]);
    dc1394_camera_free(camera[1]);
    dc1394_free(dcenv);
}

void detect_and_draw( IplImage* image,
                              int do_pyramids )
{
    IplImage* small_image = image;
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* faces;
    int i, scale = 1;

    /* if the flag is specified, down-scale the input image to get a
       performance boost w/o loosing quality (perhaps) */
    if( do_pyramids )
    {
        small_image = cvCreateImage( cvSize(image->width/2,image->height/2), IPL_DEPTH_8U, 3 );
        cvPyrDown( image, small_image, CV_GAUSSIAN_5x5 );
        scale = 2;
    }

    /* use the fastest variant */
    faces = cvHaarDetectObjects( small_image, cascade, storage, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING );

    /* draw all the rectangles */
    for( i = 0; i < faces->total; i++ )
    {
        /* extract the rectanlges only */
        CvRect face_rect = *(CvRect*)cvGetSeqElem( faces, i );
        cvRectangle( image, cvPoint(face_rect.x*scale,face_rect.y*scale),
                     cvPoint((face_rect.x+face_rect.width)*scale,
                             (face_rect.y+face_rect.height)*scale),
                     CV_RGB(255,0,0), 3 );
    }

    if( small_image != image )
        cvReleaseImage( &small_image );
    cvReleaseMemStorage( &storage );
}


/*
void detect_and_draw( IplImage* img )
{
    int scale = 1.5;

    // Create a new image based on the input image
    IplImage* temp = cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 3 );

    // Create two points to represent the face locations
    CvPoint pt1, pt2;
    int i;

    // Clear the memory storage which was used before
    cvClearMemStorage( storage );

    // Find whether the cascade is loaded, to find the faces. If yes, then:
    if( cascade )
    {

        // There can be more than one face in an image. So create a growable sequence of faces.
        // Detect the objects and store them in the sequence
        CvSeq* faces = cvHaarDetectObjects( img, cascade, storage,
                                            1.1, 2, CV_HAAR_DO_CANNY_PRUNING,
                                            cvSize(40, 40) );

        // Loop the number of faces found.
        for( i = 0; i < (faces ? faces->total : 0); i++ )
        {
           // Create a new rectangle for drawing the face
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );

            // Find the dimensions of the face,and scale it if necessary
            pt1.x = r->x*scale;
            pt2.x = (r->x+r->width)*scale;
            pt1.y = r->y*scale;
            pt2.y = (r->y+r->height)*scale;

            // Draw the rectangle in the input image
            cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
        }
    }

    // Show the image in the window named "result"
    //cvShowImage( "result", img );

    // Release the temp image created.
    cvReleaseImage( &temp );
}*/


