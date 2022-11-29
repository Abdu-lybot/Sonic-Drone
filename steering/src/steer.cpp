//
// Created by lybot on 24/11/22.
//
#include<stdio.h>
#include<bcm2835.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>


void move(int angle, int prevangle, int hold, int align)
{
    int pulse;
    int j;
    float hightime = 10.3*(float)angle + 546;
    char c;
    if(hold == 1)
    {
        printf("Holding indefinitely\n");
        while(1)
        {
            bcm2835_gpio_set(18);//pin18 high
            bcm2835_delayMicroseconds(hightime);
            bcm2835_gpio_clr(18);//pin18 low
            bcm2835_delayMicroseconds(20000 - hightime);//each pulse is 20ms
        }
    }
    if(hold == 0)
    {
        if(align == 1)
        {
            //it takes roughly 33 for 180 degrees of rotation, using 40 for good measure
            pulse = 40;
        }
        else
        {
            //calculate number of required pulses + 10 for good measure
            pulse = (int)abs(((angle - prevangle)*33)/180) + 10;
        }
        //printf("Hightime: %f\n", hightime);
        //printf("Pulse: %d\n", pulse);
        for(j=0; j<pulse; j++)//exucting pulses
        {
            //printf("%d", j);
            bcm2835_gpio_set(18);//pin18 high
            bcm2835_delayMicroseconds((int)hightime);
            bcm2835_gpio_clr(18);//pin18 low
            bcm2835_delayMicroseconds(20000 - (int)hightime);//each pulse is 20ms
        }
    }

}

int input_check(char* input)
{   int error = 0;
    int i;
    int length = strlen(input);
    if (length==4)
    {
        input[5] = '\n';
    }

    if(length>4 || input[length-1]!='\n' || (length!=2 && input[0]=='0') || input[0]=='\n')
    {
        error=1;
    }
    if(length==2 && input[0]=='q' && input[1]=='\n' && error==0)
    {
        error = 2;
    }
    if(error==0)
    {
        for (i=0;i<(length-1); i++)
        {
            if (!isdigit(input[i]) || isspace(input[i]))
            {
                error=1;
                continue;
            }
        }
        int angle = (int)strtol(input,NULL,10);
        if ( (angle>180) || (angle<0) || (error==1)) //invalid input
        {
            error = 1;
        }
    }

    return error;
}

int main(int argc, char **argv)
{

    if(!bcm2835_init()) return 1;
    int angle, prevangle, pulse, i;
    char input[5];


    bcm2835_gpio_fsel(18, BCM2835_GPIO_FSEL_OUTP); //set pin 18 as output

    move(0, 0, 0, 1); //align to 0 degree point
    prevangle = 0;
    while(1)
    {
        printf("Enter an integer degree of rotation (0 to 180) or q to quit:\n");
        fgets(input,5,stdin);

        if (input_check(input)==2)
        {
            return 0;
        }
        if(input_check(input) == 1)
        {
            printf("Invalid input, try again.\n");
        }
        if (input_check(input) == 0)
        {
            angle = (int)strtol(input,NULL,10);
            move(angle, prevangle, 0, 0);
            prevangle = angle; //keeping track of previous angle to later calculate number of pulses
        }
    }

    return 0;
}