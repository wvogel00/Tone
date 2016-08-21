/*
 * File:   main.c
 * Author: eagle
 *
 * Created on 18 August 2016, 18:06
 */


#include <xc.h>
//#include <htc.h>
#include <math.h>

#define _XTAL_FREQ 32000000  //8MHz
#define N 11
#define LENGTH 750
#define PI 3.1415926535897
#define LED PORTAbits.RA5

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = OFF     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCDDIS = ON      // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON      // Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

double max,min;
char phone[N] = "0120828828";
char signal[LENGTH] = {0};

double LowFreq(char i)
{
    if('1' <= i && i <= '3')
        return 697;
    else if('4' <= i && i <= '6')
        return 770;
    else if('7' <= i && i <= '9')
        return 852;
    else if(i == '0')
        return 941;
    
    return 0;
}
double HighFreq(char i)
{
    if((i-'0')%3 == 1)
        return 1209;
    else if(i == '0' || (i-'0') % 3 == 2)
        return 1336;
    else if((i-'0') % 3 == 0)
        return 1447;
    return 0;
}

void GetGainRange(double low, double high, double interval)
{
    max = 0;
    min = 1;
    for(int i = 0; i < LENGTH; i++)
    {
        double gain = sin(2*PI*low*i*interval) + sin(2*PI*high*i*interval);
        if(max < gain)
            max = gain;
        if(gain < min)
            min = gain;
    }
}

void MakeSignal(double max, double min, double low, double high, double interval)
{
    for(int i = 0; i < LENGTH; i+= 1)
    {
        double gain = sin(2*PI*low*i*interval) + sin(2*PI*high*i*interval);
        signal[i] = (char)(255*(gain-min)/(max-min));
    }
}

void I2C_IdleCheck(char mask)
{
    while((SSPCON2 & 0x1f) | (SSPSTAT & mask));
}

void Write(int addr, char data, int n)
{
    char WRITE = 0b10100000;  //Read
    char high = (addr >> 8 ) & 0xff;
    char low = addr & 0xff;
    char sendBuf[4] = {WRITE, high, low, data};
    for(int i = 0; i < n; i++)
    {
        I2C_IdleCheck(0x05);
        SSP1BUF = sendBuf[i];
        while(SSP1IF == 0); //ACK待ち
    }
}

void Read(int addr, int n)
{
    char READ = 0b10100001;  //Read Command
    Write(addr,READ,1);    //controlとaddrはwrite modeで送信
    I2C_IdleCheck(0x05);
    SSPCON2bits.RCEN = 1;   //受信を許可
    
    for(int i = 0; i < ; i++)
    {
        //read data
        I2C_IdleCheck(0x04);
        signal[i] = SSP1BUF;
        I2C_IdleCheck(0x05);
        while(SSP1IF == 0); //ACK待ち
    }
}

void I2C_Start()
{
    
}

void Initialize()
{
    OSCCON = 0b01110000;
    ANSELA = 0x00;
    ANSELC = 0x00;
    TRISA = 0x04;
    TRISC = 0x00;
    WPUA = 0x04;    //weak pull-up
    WPUC = 0x00;    //weak pull-up
    OPTION_REG = 0x00;  //weak pull-up enableなど
    PORTA = 0x00;
    PORTC = 0x00;
    DAC1CON1 = 0;
    DAC1CON0 = 0x80;
    OPA1CON = 0b11010010;
    
    //I2C
    SSPDATPPS = 0x14;   //RC4 as SDA(input)
    RC4PPS = 0x11;  //RC4 as SDA(output)
    RC3PPS = 0x10;  //RC3 as SCL
    SSPCON1 = 0x28;     //master mode, I2C
    SSPSTAT = 0x80;    //100kHz
    SSP1IE = 1;         //割り込み許可
    
    PPSLOCKED = 1;  //PPS selection cannot be changed.
}

void main(void) {
    Initialize();
    
    const double interval = 0.5;    //ms
    
    while(1)
    {
        for(int i = 0; i < N-1; i++)
        {
            double low = LowFreq(phone[i]);
            double high = HighFreq(phone[i]);
            GetGainRange(low,high,interval);    //rewrite max,min
            MakeSignal(max,min,low,high,interval);
            
            LED = 1;
            for(int j = 0; j < LENGTH; j++)
            {
                DAC1CON1 = signal[i];
                __delay_us(500);
            }
            LED = 0;
            
            DAC1CON1 = 0;
            __delay_ms(1000);
        }
    }
    
    return;
}
