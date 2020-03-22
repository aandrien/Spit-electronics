#include <stdio.h>
#include <wiringPi.h>


int main (void)
{
	wiringPiSetup ();
	pinMode(0, OUTPUT);
	for (;;)
	{
		printf("hallo");
	}
	return 0;
}
