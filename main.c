#include "TivaUserLib/userLib.h"

double T=0.001;
double T_print,T_real;
double w=1;
uint32_t a_cycle = 35620*5;
int32_t positionA, positionB;
int32_t position_set;
double theta[2],theta_dot;
double theta_set;
double theta_dot_set;
long pwm;
double kp=8.5, ki=100, kd=0.01;
double  max;
double pi=3.14159265359;





void operation();
double PID(double rk, double yk, double kp, double ki, double kd);
void print_value(void);

void GPIO_int()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//enable Port A
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5); // set PA3 PA4 PA5 is output

}
void pwm_int()
{
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//enable Port B
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//enable Module PWM 0

        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7);//set pin PB6 PB7 are PWM Pin types
        GPIOPinConfigure(GPIO_PB6_M0PWM0);//set PB6 is PWM0 of module 0
        GPIOPinConfigure(GPIO_PB7_M0PWM1);//set PB7 is PWM1 of module 0


        PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN| PWM_GEN_MODE_NO_SYNC);//set Module_0 Gen_0 is count down and no synchronize;

        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);// PWM clock = sys_clock/1
        double pwmPeriod=SysCtlClockGet()/25000;//set PWM frequency 25KHz
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmPeriod);// set frequence of Module_0 Gen_0 is 25KHz


        PWMGenEnable(PWM0_BASE, PWM_GEN_0);//enable module0 Gen0
        PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT|PWM_OUT_1_BIT, true);//enable PWM0 PWM1 of module1 (PB6 PB7)

}

void systick_int()
{
    double sysPeriod=SysCtlClockGet()*T;

        SysTickIntRegister(&operation);
        SysTickPeriodSet(sysPeriod);
        SysTickIntEnable();
        SysTickEnable();
}

void uart_int()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);



    GPIOPinConfigure(GPIO_PA0_U0RX);//set PA0 is RX0
    GPIOPinConfigure(GPIO_PA1_U0TX);//set PA1 is TX0
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);//set pin PA0 and PA1 are UART Pins

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM); // use system clock for UART

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE);
    UARTIntRegister(UART0_BASE,&Read_UART);// register interrupt
    IntEnable(INT_UART0);// enable interrupt for UART0
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);//enable interrupt for specific pins for UART0
}


void qei_int()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//enable Port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//enable Port C

    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);//enable QEI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);//enable QEI1



    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //unlock Pin PD7
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;

    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 10000000);//config QEI0
    QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 10000000);//config QEI1

    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);//set PD6 PD7 are OEI Types
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5|GPIO_PIN_6);//set PC5 PC6 are OEI Types

    GPIOPinConfigure(GPIO_PC5_PHA1);//configure PC5 is PHA1
    GPIOPinConfigure(GPIO_PC6_PHB1);//configure PC6 is PHB1
    GPIOPinConfigure(GPIO_PD6_PHA0);//configure PD6 is PHA0
    GPIOPinConfigure(GPIO_PD7_PHB0);//configure PD7 is PHB0

    QEIEnable(QEI0_BASE);//enable QEI0
    QEIEnable(QEI1_BASE);//enable QEI1

}

void print_value()
{

        while(UARTSpaceAvail(UART0_BASE)!=0)
                {
                print_num(UART0_BASE, theta_dot_set);
                UARTCharPut(UART0_BASE, '\t');
                print_num(UART0_BASE, theta_dot);
                UARTCharPut(UART0_BASE, '\n');
                UARTCharPut(UART0_BASE, '\r');
                }

}


void main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL |SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//80Mhz
    GPIO_int();
    pwm_int();
    systick_int();
    uart_int();
    qei_int();
    IntMasterEnable();
    max=SysCtlClockGet()/25000;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
    while(1)
    {
        if(T_print>=0.05)
        {
            print_value();
            T_print=0;
        }
    }

}

void update_position_A()
{
    static int32_t dir;
    static uint32_t temp_position;
    temp_position=QEIPositionGet(QEI0_BASE);
    dir=QEIDirectionGet(QEI0_BASE);
    if(dir==1&&temp_position!=0)
        positionA+=temp_position;
    else if(dir==-1&&temp_position!=0)
        positionA-=10000000-temp_position+1;
    QEIPositionSet(QEI0_BASE, 0);
    theta[0]=(double)positionA/(double)a_cycle*2*pi;
}

void update_position_B()
{
        static int32_t dir;
        static uint32_t temp_position;
        temp_position=QEIPositionGet(QEI0_BASE);
        dir=QEIDirectionGet(QEI1_BASE);
        if(dir==1&&temp_position!=0)
            positionB+=temp_position;
        else if(dir==-1&&temp_position!=0)
            positionB-=10000000-temp_position+1;
        QEIPositionSet(QEI1_BASE, 0);
}

void update_speed_A()
{
    theta_dot=(theta[0]-theta[1])/T;
    theta[1]=theta[0];
}




double PID(double rk, double yk, double kp, double ki, double kd)
{

   static double error[3];
   static double output[2];
   double delta=2*T;
   double alpha=2*T*kp + T*T*ki + 2*kd;
   double beta=-2*T*kp + T*T*ki - 4*kd;
   double gamma=2*kd;

   error[0]=rk-yk;
   output[0]=(alpha*error[0] + beta*error[1] + gamma*error[2])/delta + output[1];

   if (output[0] > max)
        output[0] = max;
         else if (output[0] <-max)
        output[0] = -max;

   output[1]=output[0];
   error[1]=error[0];
   error[2]=error[1];

   return output[0];

}

void execute_PWM(long pwm)
{
            if (pwm > 0)
           {
                PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT, true);
                PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, false);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (uint32_t)abs(pwm));
                 GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
           }

           else if (pwm < 0)
           {
               PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT, true);
               PWMOutputInvert(PWM0_BASE, PWM_OUT_0_BIT, true);
               PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (uint32_t)abs(pwm));
               GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
           }
           else
               PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT, false);
}

void operation()
{
    T_real+=T;
    T_print+=T;
    update_position_A();
    update_speed_A();

    theta_dot_set=sin(T_real*w)*2*pi;
    position_set=theta_set/2/pi*a_cycle;

    pwm=PID(position_set,positionA,kp,ki,kd);
    execute_PWM(pwm);

}
