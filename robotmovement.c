//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "rec/robotino/com/c/Com.h"

#ifdef WIN32
#include <windows.h>
// _getch
#include <conio.h>
#else
// getchar
#include <stdio.h>
// usleep
#include <unistd.h>
#endif

ComId com;
CameraId camera;
BumperId bumper;
OmniDriveId omniDrive;

void msleep( unsigned int ms )
{
#ifdef WIN32
	SleepEx( ms, FALSE );
#else
	usleep( ms * 1000 );
#endif
}

void waitForKey()
{
#ifdef WIN32
	_getch();
#else
	getchar();
#endif
}

//rotate vector in by deg degrees and store the output in out
void rotate( const float* in, float* out, float deg )
{
	const float pi = 3.14159265358979f;

	float rad = 2 * pi / 360.0f * deg;

	out[0] = (float)( cos( rad ) * in[0] - sin( rad ) * in[1] );
	out[1] = (float)( sin( rad ) * in[0] + cos( rad ) * in[1] );
}

int som(unsigned char m[3]){

	return (m[0] + m[1] + m[2]);
}

void grab()
{
	const unsigned int imageBufferSize = 0xfffff;
	unsigned char imageBuffer[0xfffff];
	unsigned int imageSize = 0;

	char filename[256];
	const unsigned int filenameSize = 256;
	FILE* f;

	unsigned int imageWidth;
	unsigned int imageHeight;

	unsigned int imageCounter = 0;
	int mhigh = 60;
	int i=0;
	int cnt2 = 0;
	unsigned char m[4][3];

	while( Com_isConnected( com ) && FALSE == Bumper_value( bumper ) )
  {

		if( Camera_grab( camera ) )
		{
			cnt2++;
			Camera_imageSize( camera, &imageWidth, &imageHeight );

			//printf( "Received image %04d\n  width:%d  height:%d\n", imageCounter, imageWidth, imageHeight );

			Camera_getImage( camera, imageBuffer, imageBufferSize, &imageWidth, &imageHeight );

			sprintf( filename, "image_%04d.raw", imageCounter );
			
			int topcnt = 0;
			int dncnt = 0;
			int cnt = 0;
			int side = 0;
			if(cnt2 % 2) side = imageWidth/4;
			
			for( i=0; i<(imageWidth*imageHeight*3); i+=3){
				
				if( (i % (imageWidth*3)) < side ){
					
					continue;
					
				}
				
				
				if( i < ((imageWidth*imageHeight*3)/2) ){
				
					topcnt += imageBuffer[i];
				
				}else{
					
					dncnt += imageBuffer[i];
				}
				
				
			
				m[0][0] = imageBuffer[i];
				m[0][1]  = imageBuffer[i+1];
				m[0][2]  = imageBuffer[i+2];
				

				m[1][0]  = imageBuffer[i+3]; 
				m[1][1]  = imageBuffer[i+3+1];
				m[1][2]  = imageBuffer[i+3+2];
	
				m[2][0]  = imageBuffer[i+2*3]; 
				m[2][1]  = imageBuffer[i+2*3+1];
				m[2][2]  = imageBuffer[i+2*3+2];
		
				m[3][0]  = imageBuffer[i+3*3]; 
				m[3][1]  = imageBuffer[i+3*3+1];
				m[3][2]  = imageBuffer[i+3*3+2];
				
				int smllest = 3*255+1;
				int highest = -1;
				int highestidx = -1;
				int smllestidx = 10000;

				int n =0;
				for (n = 0; n < 4; n++){
					if (som(m[n]) > highest){
						highest = som(m[n]);
						highestidx = n;
					}
					if (som(m[n]) < smllest){
						smllest = som(m[n]);
						smllestidx = n;
					}
				}

				
				if(abs(som(m[smllestidx]) - som(m[highestidx]))>(mhigh * 3)){
						
					//printf( "o %d %d\n", som(m[smllestidx]), som(m[highestidx]) );
					cnt++;
					imageBuffer[i] = 0; 
					imageBuffer[i+1] = 0;
					imageBuffer[i+2] = 255;
					
				}
		
					
				}
				
				if( (topcnt > 1.2*dncnt) && ((cnt/100) > 100) ){
				
					printf("o1 detected");
					OmniDrive_setVelocity( omniDrive, 0, 60, 0 );
				
				}
				else{
					
					OmniDrive_setVelocity( omniDrive, 0, 0, 10 );
					
				}
				
				printf( "m %d %d %d\n", topcnt, dncnt, cnt/100 );
								
				f = fopen( filename, "wb" );
				if( NULL != f )
				{

					fwrite( (const void*)imageBuffer, 1, imageWidth*imageHeight*3, f );
					fclose( f );
				}
			
		}

		msleep( 50 );
  }
}

void error( const char* message )
{
	printf( "%s\n", message );
	printf( "Press any key to exit..." );
  waitForKey();
	exit( 1 );
}

int main( int argc, char **argv )
{
	com = Com_construct();

	if( argc > 1 )
	{
		Com_setAddress( com, argv[1] );
	}
	else
	{
		//Com_setAddress( com, "172.26.1.1" );
		Com_setAddress( com, "192.168.101.101" );
		//Com_setAddress( com, "192.168.3.2:8080" );
	}

	if( FALSE == Com_connect( com ) )
	{
		error( "Error on connect" );
	}
	else
	{
		char addressBuffer[256];
		Com_address( com, addressBuffer, 256 );
		printf( "Connected to %s\n", addressBuffer );
	}

	camera = Camera_construct();
	Camera_setComId( camera, com );

	Camera_setStreaming( camera, TRUE );

	omniDrive = OmniDrive_construct();
	OmniDrive_setComId( omniDrive, com );
	
	bumper = Bumper_construct();
	Bumper_setComId( bumper, com );

	grab();

	OmniDrive_destroy( omniDrive );
	Camera_destroy( camera );
  Com_destroy( com );

	printf( "Press any key to exit...\n" );

  waitForKey();
  
  return 0;
}
