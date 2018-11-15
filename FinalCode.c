#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

float x=0;
float y=0;
float z=0;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

float thetaz = 0;
float thetax = 0;
float thetay = 0;

float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;

float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

extern float Main_IntegralSum;

void assignment(float thetamotor1, float thetamotor2, float thetamotor3){

	// Rotation zxy and its Transpose
	cosz = cos(thetaz);
	sinz = sin(thetaz);
	cosx = cos(thetax);
	sinx = sin(thetax);
	cosy = cos(thetay);
	siny = sin(thetay);

	RT11 = R11 = cosz*cosy-sinz*sinx*siny;
	RT21 = R12 = -sinz*cosx;
	RT31 = R13 = cosz*siny+sinz*sinx*cosy;
	RT12 = R21 = sinz*cosy+cosz*sinx*siny;
	RT22 = R22 = cosz*cosx;
	RT32 = R23 = sinz*siny-cosz*sinx*cosy;
	RT13 = R31 = -cosx*siny;
	RT23 = R32 = sinx;
	RT33 = R33 = cosx*cosy;

	// Jacobian Transpose
	cosq1 = cos(thetamotor1);
	sinq1 = sin(thetamotor1);
	cosq2 = cos(thetamotor2);
	sinq2 = sin(thetamotor2);
	cosq3 = cos(thetamotor3);
	sinq3 = sin(thetamotor3);

	JT_11 = -10*sinq1*(cosq3 + sinq2);
	JT_12 = 10*cosq1*(cosq3 + sinq2);
	JT_13 = 0;
	JT_21 = 10*cosq1*(cosq2 - sinq3);
	JT_22 = 10*sinq1*(cosq2 - sinq3);
	JT_23 = -10*(cosq3 + sinq2);
	JT_31 = -10*cosq1*sinq3;
	JT_32 = -10*sinq1*sinq3;
	JT_33 = -10*cosq3;

}

int counter = 0;
long mycount = 0;

//Desired Joint Variables
float xd = 10;	// xd
float yd = 0;  // yd
float zd = 20;  // zd

float dxd=0;		// similarly...
float dyd=0;
float dzd=0;

float ddxd=0;
float ddyd=0;
float ddzd=0;

//PID Gains
float KP1=0.5;		// Similarly Kpx,...
float KD1=0.05;


float KP2=0.5;
float KD2=0.05;

float KP3=0.5;
float KD3=0.05;

//Different Friction Types
float u_fric1=0;
float u_fric2=0;
float u_fric3=0;

float Vis_pos1=0.2513;
float Vis_neg1=0.2477;
float Coul_pos1=0.3637;
float Coul_neg1=-0.2948;

float Vis_pos2=0.25;
float Vis_neg2=0.287;
float Coul_pos2=0.4759;
float Coul_neg2=-0.5031;

float Vis_pos3=0.1922;
float Vis_neg3=0.2132;
float Coul_pos3=0.5339;
float Coul_neg3=-0.5190;

float slope1=3.6;
float slope2=3.6;
float slope3=3.6;

float min_vel1=0.1;
float min_vel2=0.05;
float min_vel3=0.05;

//Global motor outputs

float Theta3_old = 0;
float Omega3raw = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

float Theta2_old = 0;
float Omega2raw = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta1_old = 10;
float Omega1raw = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

//Global Motor speed for friction comp

float Theta3_oldf = 0;
float Omega3rawf = 0;
float Omega3_old1f = 0;
float Omega3_old2f = 0;
float Omega3f = 0;

float Theta2_oldf = 0;
float Omega2rawf = 0;
float Omega2_old1f = 0;
float Omega2_old2f = 0;
float Omega2f = 0;

float Theta1_oldf = 10;
float Omega1rawf = 0;
float Omega1_old1f = 0;
float Omega1_old2f = 0;
float Omega1f = 0;
//Generating Circle Trajectory
/*void CircTraj(float time)
{
	float x,y,z;
	float r = 1.5;

	x = (14.5-r)+(r*sin(time/500));
	//y=r*sin(time/500);
	y=0;
	//z = 10+(r*sin(2*time/500));
	z = 10+(r*cos(time/500));


}*/

//Calculating friction
float fric_comp(float joint_vel, float min_vel,float Viscous_positive, float Viscous_negative, float coulomb_positive, float coulomb_negative, float slope_btw_min)
{
	float u_fric;

	if (joint_vel > min_vel)
	{
		u_fric=Viscous_positive*joint_vel+coulomb_positive;
	}
	else if(joint_vel < -min_vel)
	{
		u_fric=Viscous_negative*joint_vel+coulomb_negative;
	}
	else
	{
		u_fric=slope_btw_min*joint_vel;
	}

	return u_fric;
}

// This function is called every 1 ms
static float Speed1(float thetamotor)
{
	

	// This function is called every 1 ms
	Omega1raw = (thetamotor - Theta1_old)/0.001;
	Omega1 = (Omega1raw + Omega1_old1 + Omega1_old2)/3.0;
	Theta1_old = thetamotor;
	
	//order matters here. Why??
	Omega1_old2 = Omega1_old1;
	Omega1_old1 = Omega1raw;

	return Omega1;
}

// This function is called every 1 ms
static float Speed2(float thetamotor)
{

	// This function is called every 1 ms

	Omega2raw = (thetamotor - Theta2_old)/0.001;
	Omega2 = (Omega2raw + Omega2_old1 + Omega2_old2)/3.0;
	Theta2_old = thetamotor;
	//order matters here. Why??
	Omega2_old2 = Omega2_old1;
	Omega2_old1 = Omega2raw;

	return Omega2;
}

// This function is called every 1 ms
static float Speed3(float thetamotor)
{

	// This function is called every 1 ms

	Omega3raw = (thetamotor - Theta3_old)/0.001;
	Omega3 = (Omega3raw + Omega3_old1 + Omega3_old2)/3.0;
	Theta3_old = thetamotor;
	//order matters here. Why??
	Omega3_old2 = Omega3_old1;
	Omega3_old1 = Omega3raw;

	return Omega3;

}
//---------------------------------------
static float MotorSpeed1(float thetamotor)
{


	// This function is called every 1 ms
	Omega1rawf = (thetamotor - Theta1_oldf)/0.001;
	Omega1f = (Omega1rawf + Omega1_old1f + Omega1_old2f)/3.0;
	Theta1_oldf = thetamotor;

	//order matters here. Why??
	Omega1_old2f = Omega1_old1f;
	Omega1_old1f = Omega1rawf;

	return Omega1f;
}

// This function is called every 1 ms
static float MotorSpeed2(float thetamotor)
{

	// This function is called every 1 ms

	Omega2rawf = (thetamotor - Theta2_oldf)/0.001;
	Omega2f = (Omega2rawf + Omega2_old1f + Omega2_old2f)/3.0;
	Theta2_oldf = thetamotor;
	//order matters here. Why??
	Omega2_old2f = Omega2_old1f;
	Omega2_old1f = Omega2rawf;

	return Omega2f;
}

// This function is called every 1 ms
static float MotorSpeed3(float thetamotor)
{

	// This function is called every 1 ms

	Omega3rawf = (thetamotor - Theta3_oldf)/0.001;
	Omega3f = (Omega3rawf + Omega3_old1f + Omega3_old2f)/3.0;
	Theta3_oldf = thetamotor;
	//order matters here. Why??
	Omega3_old2f = Omega3_old1f;
	Omega3_old1f = Omega3rawf;
	return Omega3f;

}

//---------------------------------------

float Controller1(float posx,float posxd)
{
	float t1;
	float error = posxd-posx;
	t1 = KP1*error-KD1*Speed1(posx);
	return t1;
}


float Controller2(float posy,float posyd)
{
	float error = posyd-posy;
	float t2;
	t2 = KP2*error-KD2*Speed2(posy);
	return t2;
}

float Controller3(float posz,float poszd)
{
	float error = poszd-posz;
	float t3;
	t3 = KP3*error-KD3*Speed3(posz);
	return t3;
}

void forward(){

	x = 10*cosq1*(cosq3+sinq2);
	y = 10*sinq1*(cosq3+sinq2);
	z = 10*(1 + cosq2-sinq3);
}

void taskspace(float *tau1, float *tau2, float *tau3,float th1, float th2, float th3,float condition){

	float fx,fy,fz;
	float tout1,tout2,tout3;

	assignment(th1, th2, th3);

	forward();

	fz = Controller3(z,zd);
    fy = Controller2(y,yd);
	fx = Controller1(x,xd);

	u_fric1 = fric_comp(MotorSpeed1(th1),min_vel1,Vis_pos1,Vis_neg1,Coul_pos1,Coul_neg1,slope1);
	u_fric2 = fric_comp(MotorSpeed2(th2),min_vel2,Vis_pos2,Vis_neg2,Coul_pos2,Coul_neg2,slope2);
	u_fric3 = fric_comp(MotorSpeed3(th3),min_vel3,Vis_pos3,Vis_neg3,Coul_pos3,Coul_neg3,slope3);

	tout1 = JT_11*fx + JT_12*fy + JT_13*fz + u_fric1;
	tout2 = JT_21*fx + JT_22*fy + JT_23*fz + u_fric2;
	tout3 = JT_31*fx + JT_32*fy + JT_33*fz + u_fric3;


		*tau1 = tout1;
		*tau2 = tout2;
		*tau3 = tout3;




}
int function_traj(){
	int flag=0;
	int condition = 0;
	if(mycount>250 && mycount<=500)
	{condition =1;
		//xd=((12.26-10)/1000)*(mycount-500)+10;
		//yd=((9.14)/1000)*(mycount-500);
		xd=12.0;
		yd=9.5;
	//	zd=((17-20)/2500)*(mycount-500)+20;
		KP3 = 0;
		KD3 = 0.05;

	}

	else if(mycount>500 && mycount<=3200)
	{condition =2;
		//KP1 = KP2 =0.6;

		//zd=((5.2-14)/5000)*(mycount-4000)+14;

		KD3=0.007;

	}
	else if(mycount>3200 && mycount<=3300)
		{condition =3;

			//xd=12.0;
			//yd=9.4;
			KP3 = 1.0;
			zd = 4;

		}
	else if(mycount>3300 && mycount<=3500)
		{condition =4;
			KP3 = 0.2;
			KD3=0.025;
			zd = 8.2;
			//zd=((8.2-5.2)/2000)*(mycount-14000)+5.2;

		}
	else if(mycount>3500 && mycount<= 3750)
	{condition =5;

		xd=8.5;


	}
	else if(mycount>3750 && mycount<= 4250)
	{condition =6;

		//yd=((-1.3-9.4)/750)*(mycount-4250)+9.4;
		yd = -1.3;

	}
	else if(mycount>4250 && mycount<= 4500)
		{condition =7;

			KP3 = 0.5;
			xd=10;
			yd=-1.3;
			//zd=((7.5-8.2)/1000)*(mycount-11000)+8.2; // 7.9
			zd = 7.5;

		}
	else if(mycount>4500 && mycount<= 6000)
		{
		xd = 16.5;
		//serial_printf(&SerialA, "xd %.2f %.2f %.2f \n\r",xd,yd,zd);
		condition =8;

			KP1=0.15;		// Similarly Kpx,...
			KD1=0.10;
			KP2=0.000;
			KD2=0;

		}
	else if(mycount>6000 && mycount<= 6500){
		KP1 = KP2 = KP3 = 0.5;
		KD1 = KD2 = KD3 = 0.035;
		zd=((16-7.5)/500)*(mycount-6000)+7.5;
		xd = ((13.5-16.5)/500)*(mycount-6000)+16.5;
		yd=((-6+1.3)/500)*(mycount-6000)-1.3;
		//serial_printf(&SerialA, "xd %.2f %.2f %.2f \n\r",xd,yd,zd);
		condition =9;


	}
	else if(mycount>6500 && mycount<= 8700){
		//	xd = 13;
				//yd-=6;
				//zd=11;

			condition =10;
			KP3=0;
			KD3=0.015;


		}

	else
	{condition =11;
//	serial_printf(&SerialA, "xd %.2f %.2f %.2f \n\r",xd,yd,zd);
		KP1 = KP2=KP3 = 0.5;
	KD1=KD2 = KD3=0.05;
		xd=10;
		yd=0;
		zd=20.1;
		//xd=8.5;
		//yd=9.59;
		//zd=8.2;

	}
return condition;

}


// This function is called every 1 ms
void lab(float thetamotor1,float thetamotor2,float thetamotor3,float *tau1,float *tau2,float *tau3) {

	int flag=0;
	int condition=0;
	mycount++;
	condition=function_traj();
	taskspace(tau1,tau2,tau3,thetamotor1,thetamotor2,thetamotor3,condition);

	if((mycount%50)==0){
		//serial_printf(&SerialA, "Condition %d mycount %d\n\r",condition,mycount);
		//serial_printf(&SerialA, "x y z %.2f %.2f %.2f \n\r",x,y,z);
		//serial_printf(&SerialA, "t1 t2 t3 %.2f %.2f %.2f \n\r",*tau1, *tau2, *tau3);
		//serial_printf(&SerialA, "xd q1 q2 q3 %.2f %.2f %.2f %.2f \n\r",xd,thetamotor1, thetamotor2, thetamotor3);
		serial_printf(&SerialA, "sum %.2f \n\r",Main_IntegralSum);
		//serial_printf(&SerialA, "x y z %.2f %.2f %.2f t1 t2 t3 %.2f %.2f %.2f xd yd zd %.2f %.2f %.2f \n\r",x,y,z,*tau1, *tau2, *tau3, theta1desired, theta2desired, theta3desired);
		//serial_printf(&SerialA, "x y z %.2f %.2f %.2f \n\r",x,y,z);
	}

//	serial_printf(&SerialA, "%.2f %.2f,%.2f \n\r",x,y,z);

	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card

}
