#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

float temperatura;
unsigned int numar_CAN;
unsigned ora=9, zora=0, sec=0, zsec=5, min=9, zmin=5, indentare_pe_secunda=0;

void setup()
{
	lcd.begin(16, 2);
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (trebuie sa fie <65536)
    // se porneste modul CTC
    TCCR1B |= (1 << WGM12);
    // Se salveaza bitii CS12 si CS10 pentru un prescaler de 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);	
    // de acum se poate compara intreruperea timer-ului
}

void loop()
{
    adc_init();
    numar_CAN = read_adc(0);
    lcd.setCursor(0, 0);
    temperatura  = ((numar_CAN)*(5000.00/1023)-500)/10;
    lcd.print("Temp = ");
    lcd.print(temperatura);
    lcd.write(" `");
    lcd.write(0x43);
    lcd.setCursor(0, 1);
    lcd.print("Ora ");
  
    if(indentare_pe_secunda == 10)  // daca secunda ajunge la 10, se reia de la 0, zsesc devine 1
    {
        zsec++;
        indentare_pe_secunda = 0;
    }
  
    if(zsec == 6)                   // daca cifra zecimalelor de la secunda ajunge la 6, se reia de la 0
    {
        min++;
        zsec=0;
        indentare_pe_secunda = 0;
    }
  
    if(min == 10)					// daca cifra minutelor ajunge la 10, se reia de la 0, zmin devine 1
    {
        zmin++;
        min = 0;
    }
  
    if(zmin == 6)					// ca la secunde, se reia de la 0
    {
        ora++;
        zmin = 0;
        min = 0;
        zsec = 0;
        indentare_pe_secunda = 0;
    }
  
  	if(ora == 10)						// acelasi lucru ca la minute / secunde
    {
        zora++;
        ora = 0;
        zmin = 0;
        min = 0;
        zsec = 0;
        indentare_pe_secunda = 0;
    }
 
  	lcd.print(zora);      
	lcd.print(ora);
    lcd.print(':');
    lcd.print(zmin);
    lcd.print(min);
    lcd.print(':');
    lcd.print(zsec);
    lcd.print(indentare_pe_secunda);
    delay(200);

    lcd.clear();
}

ISR(TIMER1_COMPA_vect)
{
  	indentare_pe_secunda++;
}

void adc_init()  // Initializarea ADC
{
    ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
    ADMUX |= (1<<REFS0);// AVcc + capacitor extern la pinul Atref
    ADCSRA |= (1<<ADSC);	// incepe conversia ADC
}

uint16_t read_adc(uint8_t channel)
{
    ADCSRA |= (1<<ADSC);  // conversia incepe
    while(ADCSRA & (1<<ADSC));  // asteapta cat timp conversia ADC nu este actualizata
    return ADCW;  // citeste si afiseaza tensiunea
}