#include "mbed.h"
#include "bbcar.h"
#include <cmath>
#include <cstdio>
#define SAMPLE 128
Ticker servo_ticker;
Ticker servo_feedback_ticker;
PwmIn servo0_f(D9), servo1_f(D10);
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);
DigitalInOut pin8(D8);
Thread t1, t2, t3;
EventQueue queue(32 * EVENTS_EVENT_SIZE);
DigitalOut Dout(D5);
InterruptIn interrupt(D3);
parallax_laserping  ping1(pin8);

int angles[SAMPLE] = {0};
float dist[SAMPLE] = {0};
float original_rot;
int original_ang;

int check(int errorDistance_Range){
    int speed, offset;                                                              // Control system variables
    float errorDistance, factor=1;                 

    errorDistance = (car.servo0.targetAngle - car.servo0.angle)*6.5*3.14/360;       // Calculate error
    
    speed = int(errorDistance);

    if(errorDistance > 0)                        // Add offset
        offset = 40;
    else if(errorDistance < 0)
        offset = -40;
    else
        offset = 0;    

    car.servo0.set_factor(factor);
    car.servo1.set_factor(factor);
    car.servo0.set_speed(speed + offset);  
    car.servo1.set_speed(speed + offset);

    if ( abs(errorDistance) > (errorDistance_Range) )
        return 1;   
    else return 0;
}

void spin(float distance) {
    car.servo0.targetAngle = (int)(distance*360/(6.5*3.14)) + car.servo0.angle;
    car.servo1.targetAngle = (int)(distance*360/(6.5*3.14)) - car.servo1.angle;
    while (check(1)) {
        ThisThread::sleep_for(1ms);
    }
    car.stop();
}

void showPing() {
    int i=16;
    //ThisThread::sleep_for(100ms);
    while (i--) {
        printf("%d pin8: %f\n",i,(float)ping1);
        printf("%d wheel angle: %d\n",i,car.servo0.angle);

        ThisThread::sleep_for(100ms);
    }
}

void DeterDist() {
    int i=SAMPLE;
    int len;    
    ThisThread::sleep_for(100ms);
    float d_temp;
    int ang_temp;
    float temp_rot;
    while (i--) {
        d_temp = ping1;
        ang_temp = car.servo0.angle;
        //len = abs(10/cos(((ang_temp - original_ang)*6.5/10)*3.14159/180));
        temp_rot = abs((ang_temp - original_ang)*6.5/10);
        len = (8/cos((abs(original_rot - temp_rot)*6.5/10)*3.14159/180));
        if ((int)d_temp < len) {
            angles[i-1] = ang_temp;
            dist[i-1] = d_temp;
        }
        ThisThread::sleep_for(20ms);
    }
    ThisThread::sleep_for(1s);
    for (i = 0; i < SAMPLE; i++) {
        printf("%d angle: %d\n", i, angles[SAMPLE-1-i]);
    }
    for (i = 0; i < SAMPLE; i++) {
        printf("%d dist: %f\n", i, dist[SAMPLE-1-i]);
    }
}


// main() runs in its own thread in the OS
int main()
{
    int i, cnt = 0;
    float avg_dist[2] = {0};
    int avg_ang[2] = {0};
    float temp_rot;
    float tan0, tan1;

    t3.start(callback(&queue, &EventQueue::dispatch_forever));
    float deg = 90, rotate = -140;
    
    /*
    printf("input deg:");
    scanf("%f", &deg);
    printf("%f\n", deg);

    printf("input rotate:");
    scanf("%f", &rotate);
    printf("%f\n", rotate);
    */
    int initial_ang = car.servo0.angle;
    
    printf("initial angle = %d\n", initial_ang);
    spin(10*3.14159*(deg)/360);
    printf("target distance = %f\n", (car.servo0.targetAngle)*6.5*3.14/360);
    printf("actual distance = %f\n", (car.servo0.angle)*6.5*3.14/360);
    printf("error distance = %f\n", (car.servo0.targetAngle - car.servo0.angle)*6.5*3.14/360);
    original_rot = (car.servo0.angle - initial_ang)*6.5/10;
    printf("actual rotate = %f\n", original_rot);
    ThisThread::sleep_for(2s);

    initial_ang = car.servo0.angle;
    original_ang = initial_ang;
    //t1.start(showPing);
    t2.start(DeterDist);

    spin(10*3.14159*(rotate)/360);
    
    ThisThread::sleep_for(7s);
    printf("target distance = %f\n", (car.servo0.targetAngle)*6.5*3.14/360);
    printf("actual distance = %f\n", (car.servo0.angle)*6.5*3.14/360);
    printf("error distance = %f\n", (car.servo0.targetAngle - car.servo0.angle)*6.5*3.14/360);
    printf("actual rotate = %f\n", (car.servo0.angle - initial_ang)*6.5/10);
    for (i = 0; i < SAMPLE; i++) {
        if (int(dist[SAMPLE - 1 - i]) > 0) {
            for (int j = 0; j < int(dist[SAMPLE - 1 - i]); j++) {
                printf("=");
            }
            printf("\n");
        } else {
            printf("\n");
        }
    }
    /*
    while (1){
        printf("pin8: %f\n",(float)ping1);
        ThisThread::sleep_for(100ms);
        }*/

    int num = 0;
    for (i = 0; i < SAMPLE; i++) {
        
        if (!int(dist[SAMPLE - 1 - i])) {
            avg_ang[1] = avg_ang[0];
            avg_dist[1] = avg_dist[0];

            cnt = 0;
            while (int(dist[SAMPLE - 1 - i])) {
                cnt = cnt + 1;
                avg_dist[0] = avg_dist[0] + dist[SAMPLE - 1 - i];
                avg_ang[0] = avg_ang[0] + angles[SAMPLE - 1 - i];
                i++;
            }
            avg_dist[0] = avg_dist[0]/cnt;
            avg_ang[0] = avg_ang[0]/cnt;

            if (int(avg_dist[1]) && int(avg_dist[0])) {
                temp_rot = abs((avg_ang[1] - original_ang)*6.5/10);
                tan1 = avg_dist[1]*sin((abs(original_rot - temp_rot)*6.5/10)*3.14159/180);

                temp_rot = abs((avg_ang[0] - original_ang)*6.5/10);
                tan0 = avg_dist[0]*sin((abs(original_rot - temp_rot)*6.5/10)*3.14159/180);

                printf("%d Difference = %f\n",num, abs(tan0-tan1));

                num++;
            }
        }
    }

    /*
    int num = 0;
    for (i = 0; i < SAMPLE; i++) {
        
        if (!int(avg_dist[1])) {
            cnt = 0;
            while (dist[SAMPLE - 1 - i]) {
                cnt = cnt + 1;
                avg_dist[1] = avg_dist[1] + dist[SAMPLE - 1 - i];
                avg_ang[1] = avg_ang[1] + angles[SAMPLE - 1 - i];
                i++;
            }
            avg_dist[1] = cnt > 0 ? avg_dist[1]/cnt : 0;
            avg_ang[1] = cnt > 0 ? avg_ang[1]/cnt : 0;
        } else if (!int(avg_dist[0])) {
            cnt = 0;
            while (dist[SAMPLE - 1 - i]) {
                cnt = cnt + 1;
                avg_dist[0] = avg_dist[0] + dist[SAMPLE - 1 - i];
                avg_ang[0] = avg_ang[0] + angles[SAMPLE - 1 - i];
                i++;
            }
            avg_dist[0] = cnt > 0 ? avg_dist[0]/cnt : 0;
            avg_ang[0] = cnt > 0 ? avg_ang[0]/cnt : 0;
        }
        if (int(avg_dist[0]) && int(avg_dist[1])) {
            temp_rot = abs((avg_ang[1] - original_ang)*6.5/10);
            tan1 = avg_dist[1]*sin((abs(original_rot - temp_rot)*6.5/10)*3.14159/180);

            temp_rot = abs((avg_ang[0] - original_ang)*6.5/10);
            tan0 = avg_dist[0]*sin((abs(original_rot - temp_rot)*6.5/10)*3.14159/180);

            printf("%d Difference = %f\n",num, abs(tan0-tan1));

            avg_ang[1] = avg_ang[0];
            avg_dist[1] = avg_dist[0];

            avg_ang[0] = 0;
            avg_dist[0] = 0;
            num++;
        }
            cnt = 0;
    }*/



    
}

