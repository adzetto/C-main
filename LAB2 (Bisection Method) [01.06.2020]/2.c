#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#define MAX_ITERATIONS 1000
#define ln(x) log(x)

double func(double R)
{
    double T;
    T= (1.1292/1000)+(2.3410*ln(R)/10000)+8.7754*pow(ln(R),3)/100000000;
    T=1/T;
    T=T-273.15;
    return T;
}

double func2(double t2,double R1)
{
    double result;
    t2=t2+273.15;
    result= (1.1292/1000)+(2.3410*ln(R1)/10000)+8.7754*pow(ln(R1),3)/100000000;
    result=result*t2;
    result=result-1;
    return result;
}

double bisect(double temp, double epsilon, double resistor1, double resistor2)
{
    int sayac=0;
    double y;
    for(y=(resistor1+resistor2)/2.0; fabs(resistor1-y)>epsilon && sayac < MAX_ITERATIONS; y=(resistor1+resistor2)/2.0){
        sayac++;
        if(func2(temp,resistor1)*func2(temp,y)<=0.0){
            resistor2=y;
        }
        else
            resistor1=y;
    }
    printf("With %d iteration,",sayac);
    return(y);
}

int main ()
{
    double temp[4];
    printf("Enter the Temperature values in Celsius\n");
    int i;
    for(i=0; i<4; i++){
        scanf("%lf",&temp[i]);
    }
    double r1,r2;
    double startpoint,endpoint,tolerance;

    r1=5000;
    r2=25000;
    printf("For R_test_1: Temperature is calculated %f\n",func(r1));
    printf("For R_test_2: Temperature is calculated %f\n",func(r2));
    printf("Enter the start point: ");
    scanf("%lf",&startpoint);
    printf("Enter end point: ");
    scanf("%lf",&endpoint);
    printf("Enter temperature tolerance: ");
    scanf("%lf",&tolerance);
    double root;
    for(i=0; i<4; i++){
        printf("For Temperature=%f , Accurate Root calculated is %f\n",temp[i],bisect(temp[i],tolerance,startpoint,endpoint));
    }

    return 0;
}
