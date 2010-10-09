#include "opencv/highgui.h"
#include "dc1394/dc1394.h"

int main( int argc, char** argv ) {
    dc1394error_t error;
    // firewire stuff
    // clear DMA ring buffer stop ISO transmission
    dc1394_t* dcenv = dc1394_new();
    dc1394video_frame_t* tmpframe[10];
    dc1394camera_list_t* camlist[2];
    dc1394_camera_enumerate(dcenv, camlist);

    printf("%d cameras\n", sizeof camlist / sizeof camlist[0]);

    dc1394camera_t* camera0 = dc1394_camera_new(dcenv,camlist[0]->ids->guid);
//    dc1394camera_t* camera1 = dc1394_camera_new(dcenv,camlist[1]->ids->guid);

    dc1394_camera_free_list(camlist[0]);
    dc1394_camera_free_list(camlist[1]);
    
    dc1394_capture_stop(camera0);
//    dc1394_capture_stop(camera1);
    
    dc1394_reset_bus(camera0);

/*    do {
        dc1394_capture_enqueue(camera0, tmpframe[0]);
    } while (dc1394_capture_dequeue(camera0,DC1394_CAPTURE_POLICY_POLL, tmpframe) != NULL); */
/*    do {
        dc1394_capture_enqueue(camera1,tmpframe[0]);
    } while (dc1394_capture_dequeue(camera1,DC1394_CAPTURE_POLICY_POLL, tmpframe) != NULL);
*/
    //dc1394_reset_camera(camera0);
    //dc1394_reset_camera(camera1);

    cvNamedWindow("side-by-side", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "Wide FOV", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Zoom", CV_WINDOW_AUTOSIZE );

    CvCapture* capture1;
   // CvCapture* capture2;
    if (argc == 1) {
        capture1 = cvCaptureFromCAM(0);
     //   capture2 = cvCaptureFromCAM(1);
    } else {
        capture1 = cvCaptureFromFile( argv[1] );
       // capture2 = cvCaptureFromFile( argv[1] );
    }
    IplImage* frame1;
    //IplImage* frame2;
    IplImage frame3;
    while(1) {
        frame1 = cvQueryFrame( capture1 );
      //  frame2 = cvQueryFrame( capture2 );
        //if( !frame1 || !frame2 ) break;
        if( !frame1) break;
        cvShowImage( "Wide FOV", frame1 );
        //cvShowImage( "Zoom", frame2 );
        frame3 = *frame1;
        frame3.width = frame1->width*2;
        cvShowImage("side-by-side",&frame3);
        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }
    cvReleaseCapture( &capture1 );
    //cvReleaseCapture( &capture2 );
    cvDestroyWindow( "Wide FOV" );
    cvDestroyWindow( "Zoom" );

    dc1394_camera_set_power(camera0, DC1394_OFF);
    //dc1394_camera_set_power(camera1, DC1394_OFF);

    dc1394_camera_free(camera0);
    //dc1394_camera_free(camera1);
    dc1394_free(dcenv);
}

