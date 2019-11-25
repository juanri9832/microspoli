// CONFIG
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define AN0 0 //Reemplaza AN0 por el valor cero
#define AN1 1 //Reemplaza AN1 por el valor uno
#define AN2 2 //Reemplaza AN2 por el valor dos

//PINES DEL LCD
#define RS RD2 //Define el pin RS del LCD en el Pin D2
#define EN RD3 // Define el pin Enable del LCD en el Pin D3
//Se utiliza ua comunicacion a 4 bits 
#define D4 RD4 // Define el pin D4 del LCD en el Pin D4(puerto D))
#define D5 RD5 // Define el pin D5 del LCD en el Pin D5(puerto D))
#define D6 RD6
#define D7 RD7
#define _XTAL_FREQ 4000000 //Define la frecuencia de oscilacion interna a 4MHz


#include "lcd.h" //Librería de LCD>
#include <stdio.h> //librería ara oder usar función sprintf de C

//Definición de Variables
int sensor1=0;
int sensor2=0;
int sensor3=0;
float temp=0;
float aux;

//Función para inicializar EL ADC
void init(void){
     ADCON0 = 0b10000001;
     ADCON1 = 0b10000000;
}

//Función para Leer una entrana análoga del Microcrontrolador
int conversion(unsigned char canal){
    int resultado=0;
    if(canal>3){  //si el canal seleccionado no es AN0,AN1 ó AN2 retorna cero
        return 0;
    }
    ADCON0 &=0b11000101; //Establece velocidad de conversión del ADC
    ADCON0 |= canal<<3;  //Escoge el Canal que se le realizará la conversion ADC
    __delay_ms(3);   //retardo de 3 milisegundos
    GO_nDONE=1;      //pone Go/done en 1, para a esperar hasta que se complete
    while(GO_nDONE); //espera que se termine de realizar la conversion (Go/done=0)
    resultado= ((ADRESH<<8)+ADRESL); //guarda el resultado de la conversion (10 bits)
    return resultado;      //retorna el resultado
}

 //Función que permite Imprimir Números Flotantes
 void Lcd_flotante( float f){ 
      char s[7];
     sprintf(s, "%.1f", f);    
     Lcd_Write_String(s);
  }
//Función Principal
void main(void) {
    init(); //Se llama a la Función de inicialización del ADC
    TRISA=0xFF; //configuracion de puerto A como Entrada
    TRISD=0x00; //configuracion de Puerto D como Salida
    PORTD=0;    //Inicializacion del puerto en Cero
    Lcd_Init(); //Inicializa el Display LCD
    Lcd_Clear(); // Comando para limpiar pantalla

    while(1){ //ciclo infinito
        sensor1=conversion(AN0); // Lee el NTC
        temp=(sensor1*5.0)/1023.0; //convierte el valor del adc a Voltaje
        aux=temp; //copia de variable temp
        aux=-35.7*(aux*aux*aux)+440.7*aux*aux-1850*aux+2686; //cálculo del polinomio para hallar la temperatura
        Lcd_Set_Cursor(1,1);  //Ubica el cursor del LCD en la linea 1
        Lcd_Write_String("NTC:"); //Imprime en el LCD
        Lcd_flotante(aux);        //Imprime el valor Flotante en el LCD
        
        sensor2=conversion(AN1); // Lee el sensor LM35
        temp=(sensor2*5.0)/1023; //convierte el valor del adc a Voltaje
        temp=(6*temp)+20;        // Convierte el voltaje del LM35 a temperatura
        Lcd_Set_Cursor(1,10);    //Ubica el cursor del LCD en la linea 1, posicion 10
        Lcd_Write_String("LM35:"); //imprime en el LCD
        Lcd_Write_Int((int)temp); //Imprime el valor entero de Temperatura en el LCD
        
        sensor3=conversion(AN2); // Lee el valor de potenciometro
        temp=(sensor3*5.0)/1023; //convierte el valor del adc a Voltaje
        Lcd_Set_Cursor(2,1);    //Ubica el cursor del LCD en la linea 2
        Lcd_Write_String("POT:"); //imprime en el LCD
        Lcd_flotante(temp);     //Imprime el valor Flotante en el LCD      
    }

}
