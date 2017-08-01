//
// sample.cpp
//
// A sample program for Steve Hassenplug's speed benchmark test 
// (http://www.teamhassenplug.org/NXT/NXTSoftwareSpeedTest.html)
//
#include<math.h>
#include <stdlib.h> 

// ECRobot++ API
#include "LightSensor.h"
#include "SonarSensor.h"
#include "Motor.h"
#include "Lcd.h"
#include "Clock.h"
using namespace ecrobot;


extern "C"
{
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

// nxtOSEK hook to be invoked from an ISR in category 2
void user_1ms_isr_type2(void)
{
	SleeperMonitor(); // needed for I2C device and Clock classes
}
double q1,q2;
double t1,t2,c2,s21,s22,ans=0,atan_ans=0;
double t1s =-90;
double t2s = -90;
double x_out,y_out;
//atan lookup values
double data[91][2] = {{0,0.00},{1,.0175},{2,.0349},{3,.0524},{4,.0699},{5,.0875},{6,.1051},{7,.1228},{8,.1405},{9,.1584},{10,.1763},{11,.1944},{12,.2126},{13,.2309},{14,.2493},{15,.2679},{16,.2867},{17,.3057},{18,.3249},{19,.3443},{20,.3640},{21,.3839},{22,.4040},{23,.4245},{24,.4452},{25,.4663},{26,.4877},{27,.5095},{28,.5317},{29,.5543},{30,.5773},{31,.6009},{32,.6249},{33,.6494},{34,.6745},{35,.7002},{36,.7265},{37,.7535},{38,.7813},{39,.8098},{40,.8391},{41,.8693},{42,.9004},{43,.9325},{44,.9657},{45,1.00},{46,1.0355},{47,1.0724},{48,1.1106},{49,1.1504},{50,1.1918},{51,1.2349},{52,1.2799},{53,1.3270},{54,1.3764},{55,1.4281},{56,1.4826},{57,1.5399},{58,1.6003},{59,1.6643},{60,1.7321},{61,1.8040},{62,1.8907},{63,1.9626},{64,2.0503},{65,2.1445},{66,2.2460},{67,2.3559},{68,2.4751},{69,2.6051},{70,2.7475},{71,2.9042},{72,3.0777},{73,3.2709},{74,3.4874},{75,3.7321},{76,4.0108},{77,4.3315},{78,4.7046},{79,5.1446},{80,5.6713},{81,6.3138},{82,7.1154},{83,8.1443},{84,9.1544},{85,11.430},{86,14.301},{87,19.081},{88,28.636},{89,57.290},{90,57.291}};
/**************************************************************************************************************************
******************************* Fuctions For performing kineamtics and turns***********************************************
***************************************************************************************************************************/

void ToStart()
{

}

double absolute(double a )
{
	
	if (a > 0)
		return a;
	else
		return -a;
}
//atan using lookup
double aTan(double x) {
	int iter = 0, count = 0;
    	if (x > 0){
		while((iter<91) && (data[iter][1])<x){
			iter = iter + 1;
		}
	} 
	return data[iter-1][0]*M_PI/180;
}

//custom atan2 function
double aTan2(double y, double x) {
	double u = aTan(absolute(y / x));
	
	if ((y/x) < 0){
		u = -u;
	}
    	atan_ans = u;
	/*if (x>0){
		ans = u;	
	}
	else if (((y>0) || (y=0)) && (x<0)){
		ans = u+M_PI;	
	}
	else if ((y<0) && (x<0)){
		ans = u-M_PI;	
	}
	else if ((y>0) && (x=0)){
		ans = M_PI/2;	
	}
	else if ((y<0) && (x=0)){
		ans = -M_PI/2;	
	}
	else {
		ans = 0;
	}
    return ans;*/
	if(x<0.0){	
	if (u>0.0){
		u =u -M_PI;
	}
	else{
		u =u + M_PI;
	}
	}
	return u;
 }

//Square root function
int our_ceil(double N) {
    int ceil = (int) N;
    if (N > ceil)
        return (int) (N + 1);
    return ceil;
 }

 int our_floor(double N){
    return (int)N;
 }
 int our_pow(int X, int Y)
 {
    int sum = X;
    for(int i = 1; i < Y; i++)
    {
        sum *= X;
    }
    return sum;
 }
double our_sqrt(double N)
 {
    double x = our_pow(2, our_ceil((sizeof(N)*8.0)/4.0));
    double y = 0;
    while(1){
        y = (x + N/x)/2;
        if (y >= x)
            return x;
        x = y;
    }
 }
 

 // End sqrt implementation

//Inversie Kinematics code
void InverseKinematics(double px,double py,double pz)
{ 
	double l1,l2,k1, val_float;
	double gama1,gama2, temp, temp_val,k22,k21;
	int val_int;
	//Links Length
	l1 = 0.105;
	l2 = 0.155;
	// Find q2
	c2 = ((px*px)+(py*py)-(l1*l1)-(l2*l2))/(2*l1*l2);
	temp_val = 1.0-(c2*c2);
	s21 = our_sqrt(temp_val) ;
	s22 =-our_sqrt(temp_val) ;
	//We have two solutions for q2

	q2 = aTan2((double)s22,(double)c2) ;//2nd
	//q2 = aTan2((double)s21,(double)c2) ;//1st
	//We define the constants
	k1 = l1+l2*c2 ;

	k22 = l2*s22 ;
	//gama1 = aTan2((double)k21,(double)k1) ;//1st
	gama2 = aTan2((double)k22,(double)k1) ; //2nd
	

	//We find q1 depending on the value of q2
	
	//q1 = aTan2((double)py,(double)px) - gama1 ; //1st
	q1 = aTan2((double)py,(double)px) - gama2 ;   //2nd	
	
	q1 = q1 * 180/3.14 ;
	
	q2 = q2 * 180/3.14 ;
	
}

//Turn motor C(q1)
void TurnC(double old ,double new_val)
{
	float tick = 7.32;
	int count =0;
	int val = (int)(tick * absolute(old - new_val));
		if(old > new_val)
		{
			nxt_motor_set_speed(NXT_PORT_C, -40, 1);			
			while(count > -val)
			{
				
				count =	nxt_motor_get_count(NXT_PORT_C);			
			}
			nxt_motor_set_speed(NXT_PORT_C, 0, 1);
			nxt_motor_set_count(NXT_PORT_C,0);	
		}
		if(new_val > old)
		{
			nxt_motor_set_speed(NXT_PORT_C, 40, 1);			
			while(count < val)
			{
				
				count =	nxt_motor_get_count(NXT_PORT_C);
			}
			nxt_motor_set_speed(NXT_PORT_C, 0, 1);
			nxt_motor_set_count(NXT_PORT_C,0);
		}
			
}

//Turn motor b(q2)
void TurnB(double old, double new_val)
{
	float tick = 93.6;
	int count =0;
	int val = (int)(tick * absolute(old -new_val));
		if( old > new_val)
		{
			nxt_motor_set_speed(NXT_PORT_B, -80, 1);			
			while(count > -val)
			{
				
				count =	nxt_motor_get_count(NXT_PORT_B);			
			}
			nxt_motor_set_speed(NXT_PORT_B, 0, 1);
			nxt_motor_set_count(NXT_PORT_B,0);
		}
		if(new_val > old)
		{
			nxt_motor_set_speed(NXT_PORT_B, 80, 1);			
			while(count < val)
			{
				
				count =	nxt_motor_get_count(NXT_PORT_B);
			}
			nxt_motor_set_speed(NXT_PORT_B, 0, 1);
			nxt_motor_set_count(NXT_PORT_B,0);
		}
				
}
//move pen down
void DownA()
{
	nxt_motor_set_speed(NXT_PORT_A,-30,1);
	systick_wait_ms(1000);
	nxt_motor_set_speed(NXT_PORT_A,0,1);
}
//move pen up
void UpA()
{
	nxt_motor_set_speed(NXT_PORT_A,30,1);
        systick_wait_ms(1000);
        nxt_motor_set_speed(NXT_PORT_A,0,1);
}

//display value on bot
void disp()
{
	int a = x_out * 1000;
	int b = y_out * 1000;
	int c = q1 * 1000;
	int d = q2 * 1000;	
	int e = s21 * 1000;
	int f = s22 * 1000;
	//int g = atan_ans * 1000;
	int g = c2 * 1000;
	display_clear(0);
	display_goto_xy(0, 0);
	display_string(" px: ");
	display_int(a, 0);
	display_goto_xy(0, 1);
	display_string("py: ");
	display_int(b, 0);
	display_goto_xy(0, 2);
	display_string("q1: ");
	display_int(c, 0);
	display_goto_xy(0, 3);
	display_string("q2: ");
	display_int(d, 0);
	display_goto_xy(0, 4);
	//display_string("atan_ans: ");
	display_string("c2: ");
	display_int(g, 0);
	display_goto_xy(0, 5);
	display_string("s21: ");
	display_int(e, 0);
	display_goto_xy(0, 6);
	display_string("s22: ");
	display_int(f, 0);
	display_update();
	//systick_wait_ms(10000);
}

void LineCreate()
{
	int size = 4; 	
	float x1,y1,x2,y2,a,b,c,x,y;      
	int len, i , iter_mult;
 
        float x_input[4]={-0.1471,-1.0129,-1.0129,-0.1471};
	float y_input[4]={0.05882, 0.05882, 0.1765, 0.1765};
	
	/*for(i = 0;i < size ;i++)
	{	
		x_input[i] = x_input[i]/34;
		y_input[i] = y_input[i]/34;
	*/

	len = size;
	i = 0;
	DownA();
	while (i < len)
	{
	    x1 = x_input[i];
	    y1 = y_input[i];
	    if (i == len-1)
	    {
		x2 = x_input[0];
		y2 = y_input[0];
   	    }
	    else
	    {
	    	x2 = x_input[i+1];
	    	y2 = y_input[i+1];
	    }
	    //Get the parametes for line 
	    a = y1-y2;
	    b = x2-x1;
	    c = (x1-x2)*y1 + (y2-y1)*x1;
	    
	    if (absolute(x2-x1) > absolute(y2-y1))  // if variance of x-cordinates>y co-ordinates, vary x, then get y
	     {
		if (x1>x2)
		{
		    iter_mult = -1;
		    for(x = x1; x>=x2 ; x = x+iter_mult*0.001)
		    {
		   	   
		    	    x_out = x;
			    y_out = (-c - (a*x))/b;
			    InverseKinematics(x_out,y_out,0);
			    TurnC(t1,q1);
			    TurnB(t2,q2);
			    disp();
			    t1 = q1;
			    t2 = q2;
		   
		    }
		}
		else              // if variance of x-cordinates<y co-ordinates, vary y, then get x
		{		    
			iter_mult = 1;
			for(x = x1; x<=x2 ; x = x+iter_mult*0.001)
			{
			    
			    x_out = x;
			    y_out = (-c - (a*x))/b;
			    InverseKinematics(x_out,y_out,0);
			     TurnC(t1,q1);
			    TurnB(t2,q2);
			    disp();
			    t1 = q1; //update t1
			    t2 = q2; //update t2
			   
			}
		}
		
	     }
	    else
	    {
		if (y1>y2)
		{
		    iter_mult = -1;
		    for (y = y1;y >= y2; y = y+ iter_mult*0.001)
			{
			  
			    y_out = y;
			    x_out = (-c - (b*y))/a;
			    InverseKinematics(x_out,y_out,0);
			    TurnC(t1,q1);
			    TurnB(t2,q2);
			    //disp();
			    t1 = q1;
			    t2 = q2;
			}		
		}
		else
		{
		    iter_mult = 1;
		    for (y = y1;y <= y2; y = y+ iter_mult*0.001)
			{
			    
			    y_out = y;
			    x_out = (-c - (b*y))/a;
			    InverseKinematics(x_out,y_out,0);
			    TurnC(t1,q1);
			    TurnB(t2,q2);
			    //disp();
			    t1 = q1;
			    t2 = q2;
			}
		}
	    }
	    i = i+1;
	}
UpA();

}


TASK(TaskMain)
{
	//Initial angle setup
	t1 =180;
	t2 = -50;
	LineCreate();	
		
}
}