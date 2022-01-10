#include <LiquidCrystal.h>

#define BUTTON_OK 6
#define BUTTON_CANCEL 7
#define BUTTON_PREV 8
#define BUTTON_NEXT 9

enum Buttons {
  EV_OK,
  EV_CANCEL,
  EV_NEXT,
  EV_PREV,
  EV_NONE,
  EV_MAX_NUM
};

enum Menus {
  MENU_MAIN = 0,
  MENU_START,
  MENU_KP,
  MENU_KI,
  MENU_KD,
  MENU_TEMP,
  MENU_T_INCAL,
  MENU_T_MENT,
  MENU_T_RAC,
  MENU_MAX_NUM,
  MENU_NULL
};

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
unsigned int numar_CAN;
int voltage_value, temperatura_citita;
unsigned int temp;
int t_incal = 5, t_incal_copy;
double timp_set = 50;
int t_ment = 5, t_ment_copy;
int t_rac = 10, t_rac_copy;
double moving_setpoint;
float temp_q = 0;
Menus scroll_menu = MENU_MAIN;
Menus current_menu = MENU_MAIN;
enum Menus menu_anterior = MENU_NULL;
double kp = 20, kp_copy;
double ki = 0.02, ki_copy;
double kd = 0.02, kd_copy;
double afisare;
double eroare = 26;
int suma_erori = 0;
double eroare_anterioara = 0;
double derivativa = 0;
double dt;
double output = 0;
int numar;
unsigned int indentare_pe_secunda = 0;
float temp_anterioara = -999;


void state_machine(enum Menus menu, enum Buttons button);
Buttons GetButtons(void);
void print_menu(enum Menus menu);

typedef void (state_machine_handler_t)(void);

void print_menu(enum Menus menu)
{
  Calcul_Temperatura();
  
    lcd.clear();
  
    switch(menu)
    {
      case MENU_MAIN:
          lcd.print("Meniu principal");
          lcd.setCursor(0,1);
          lcd.print("Alegeti program.");
          break;
      case MENU_START:
          lcd.setCursor(0,0);
          lcd.print("TI ");
          lcd.print(t_incal);
          lcd.print(" ");
          lcd.print("TM ");
          lcd.print(t_ment);
          lcd.print(" ");
          lcd.print("TR ");
          lcd.print(t_rac);
          lcd.setCursor(0,1);
          lcd.print("T ");
          lcd.print(temperatura_citita);
          lcd.print(" ");
          lcd.print("SP ");
      case MENU_KP:
          lcd.print("KP = ");
          lcd.print(kp);
          break;
      case MENU_KI:
          lcd.print("Ki = ");
          lcd.print(ki);
          break;
      case MENU_KD:
          lcd.print("KD = ");
          lcd.print(kd);
          break;
      case MENU_TEMP:
          lcd.print("TEMP = ");
          lcd.print(temp);
          break;
      case MENU_T_INCAL:
          lcd.print("Timp incalz = ");
          lcd.print(t_incal);
          break;
      case MENU_T_MENT:
          lcd.print("Timp ment = ");
          lcd.print(t_ment);
          break;
      case MENU_T_RAC:
          lcd.print("Timp racire = ");
          lcd.print(t_rac);

      /*default:
          lcd.print("PS 2021");
          lcd.setCursor(0,1);
          lcd.print(" Timp scurs:");
          lcd.print(indentare_pe_secunda);
          lcd.print("s");
          break;
      */

    menu_anterior = menu;
  }
  if(current_menu != MENU_MAIN)
  {
  	lcd.setCursor(0,1);
  	lcd.print("ModificaParam.");
  }
}

void enter_menu(void)
{
  current_menu = scroll_menu;
}

void go_home(void)
{
  scroll_menu = MENU_MAIN;
  current_menu = scroll_menu;
}

void go_next(void)
{
  scroll_menu = (Menus) ((int)scroll_menu + 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);
}

void go_prev(void)
{
  scroll_menu = (Menus) ((int)scroll_menu - 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);
}


void increm_kp(void)
{
  kp++;
}

void decrem_kp(void)
{
  kp--;
}

void increm_ki(void)
{
  ki++;
}

void decrem_ki(void)
{
  ki--;
}


void increm_kd(void)
{
  kd++;
}

void decrem_kd(void)
{
  kd--;
}

void save_temp(void)
{
  scroll_menu = MENU_TEMP;
  current_menu = MENU_MAIN;
}

void cancel_temp(void)
{
  scroll_menu = MENU_TEMP;
  current_menu = MENU_MAIN;
}

void increm_temp(void)
{
  temperatura_citita++;
}

void decrem_temp(void)
{
  temperatura_citita--;
}

void increm_t_incal(void)
{
 t_incal++; 
}

void decrem_t_incal(void)
{
 t_incal--; 
}

void increm_t_ment(void)
{
 t_ment++; 
}

void decrem_t_ment(void)
{
 t_ment--; 
}

void increm_t_rac(void)
{
 t_rac++; 
}

void decrem_t_rac(void)
{
 t_rac--; 
}

void cancel_t_rac(void)
{
  t_rac = t_rac_copy;
  scroll_menu = MENU_T_RAC;
  current_menu = MENU_MAIN;
}

void cancel_t_incal(void)
{
  t_incal = t_incal_copy;
  scroll_menu = MENU_T_INCAL;
  current_menu = MENU_MAIN;
}

void cancel_t_ment()
{
  t_ment = t_ment_copy;
  scroll_menu = MENU_T_MENT;
  current_menu = MENU_MAIN;
}
  
void save_t_rac()
{
  scroll_menu = MENU_T_RAC;
  current_menu = MENU_MAIN;
}

void save_t_incal()
{
  scroll_menu = MENU_T_INCAL;
  current_menu = MENU_MAIN;
}

void save_t_ment()
{
  scroll_menu = MENU_T_MENT;
  current_menu = MENU_MAIN;
}

void cancel_kp(void)
{
  kp = kp_copy;
  scroll_menu = MENU_KP;
  current_menu = MENU_MAIN;
}

void cancel_ki(void)
{
  ki = ki_copy;
  scroll_menu = MENU_KI;
  current_menu = MENU_MAIN;
}

void cancel_kd(void)
{
  kd = kd_copy;
  scroll_menu = MENU_KD;
  current_menu = MENU_MAIN;
}

void save_kd(void)
{
  scroll_menu = MENU_KD;
  current_menu = MENU_MAIN;
}

void save_kp(void)
{
  scroll_menu = MENU_KP;
  current_menu = MENU_MAIN;
}

void save_ki()
{
  scroll_menu = MENU_KI;
  current_menu = MENU_MAIN;
}


state_machine_handler_t* sm[MENU_MAX_NUM][EV_MAX_NUM] = 
{ //events: OK(dreapta) , CANCEL(stanga) , NEXT(jos), PREV(sus)
  {enter_menu, go_home, go_next, go_prev},  						// MENU_MAIN
  {enter_menu, go_home, go_next, go_prev},		   					// MENU_START
  {save_kp, cancel_kp, increm_kp, decrem_kp},   					// MENU_KP
  {save_ki, cancel_ki, increm_ki, decrem_ki},						// MENU_KI
  {save_kd, cancel_kd, increm_kd, decrem_kd},						// MENU_KD	
  {save_temp, cancel_temp, increm_temp, decrem_temp},				// MENU_TEMP
  {save_t_incal, cancel_t_incal, increm_t_incal, decrem_t_incal},	// MENU_KI
  {save_t_ment, cancel_t_ment, increm_t_ment, decrem_t_ment},		// MENU_T_MENT
  {save_t_rac, cancel_t_rac, increm_t_rac, decrem_t_rac},			// MENU_T_RAC
};

void state_machine(enum Menus menu, enum Buttons button)
{
  sm[menu][button]();
}

Buttons GetButtons(void)
{
  enum Buttons ret_val = EV_NONE;
  if (digitalRead(BUTTON_OK))
  {
    salvare_val_initiale();
    ret_val = EV_OK;
  }
  else if (digitalRead(BUTTON_CANCEL))
  {
    ret_val = EV_CANCEL;
  }
  else if (digitalRead(BUTTON_NEXT))
  {
    ret_val = EV_NEXT;
  }
  else if (digitalRead(BUTTON_PREV))
  {
    ret_val = EV_PREV;
  }
  
  return ret_val;
}

void salvare_val_initiale(void)
{
  switch(scroll_menu)
  {
    case MENU_KP: 
    	kp_copy = kp;
    	break;
    case MENU_KI:
    	ki_copy = ki;
    	break;
    case MENU_KD: 
    	kd_copy = kd;
    	break;
    case MENU_TEMP:
    	break;
    case MENU_T_INCAL:
    	t_incal_copy = t_incal;
    	break;
    case MENU_T_MENT:
    	t_ment_copy = t_ment;
    	break;
    case MENU_T_RAC:
    	t_rac_copy = t_rac;
    	break;
  }
}

void setup()
{
  Serial.begin(9600);
  lcd.begin(16,2);
  pinMode(6, INPUT);
  digitalWrite(6, LOW); // pull-down
    pinMode(7, INPUT);
  digitalWrite(7, LOW); // pull-down
    pinMode(8, INPUT);
  digitalWrite(8, LOW); // pull-down
    pinMode(9, INPUT);
  digitalWrite(9, LOW); // pull-down
  adc_init();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (trebuie sa fie <65536)
  // se porneste modul CTC
  TCCR1B |= (1 << WGM12);
  // Se salveaza bitii CS12 si CS10 pentru un prescaler de 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);	
  // de acum se poate compara intreruperea timer-ului
  TIMSK1 |= (1 << OCIE1A);
} 
  
void Afisare(int numar)
{
  if (numar < 10 && numar > 0)
    lcd.print('0');
}
  
void Calcul_Temperatura()
{
  numar_CAN = read_adc(4);
  temp = read_adc(5);
  voltage_value = (temp*5000.0)/1024.0;
  temperatura_citita = (voltage_value - 500) / 10;
  if(temperatura_citita != temp_anterioara)
  {
    temp_anterioara = temperatura_citita;
    menu_anterior = MENU_NULL;
  }
}
  
void Numarare_Secunde(void)
{
  double now = millis();
  int total_seconds = now/1000;

  if (total_seconds < t_incal)
  {
    Serial.print("INC = ");
    Serial.println(total_seconds);
    moving_setpoint = temperatura_citita + (timp_set - temperatura_citita)*total_seconds;
    Serial.print("SP = ");
    Serial.println(moving_setpoint);
  }

  else if (total_seconds <= (t_incal + t_ment))
  {
    Serial.print("MEN = ");
    Serial.println(total_seconds);
  }

  else if (total_seconds <= (t_incal + t_ment + t_rac))
  {
    Serial.print("RAC = ");
    Serial.println(total_seconds);
    moving_setpoint = temperatura_citita + (timp_set - temperatura_citita) - (timp_set - temperatura_citita);
    Serial.print("SP = ");
    Serial.println(moving_setpoint);
  }        

  else
  {
    Serial.print("Procesul a fost oprit.");
    Serial.println(total_seconds);
  }

  if (scroll_menu == MENU_START)
  {
    lcd.setCursor(8,1);
    lcd.print(moving_setpoint);
    lcd.print(" t : ");
    lcd.setCursor(14, 1);
    lcd.print(total_seconds);
    lcd.print("s");
  }
}

double pid()
{
  eroare = moving_setpoint - temperatura_citita;
  suma_erori= suma_erori + eroare * dt;
  derivativa = (eroare - eroare_anterioara) / dt;
  output = (kp * eroare) + (ki * suma_erori ) + (kd * derivativa);
  eroare_anterioara = eroare;
  if(output > 255)
    output = 255;
  else if(output < 0)
    output = 0;
}  

void loop()
{
  Numarare_Secunde();
  pid();
  analogWrite(10, 255-output);
  
  volatile Buttons event = GetButtons();
  if (event != EV_NONE)
    state_machine(current_menu, event); 

  print_menu(scroll_menu);
  delay(1000);
}

ISR(TIMER1_COMPA_vect)
{
  	indentare_pe_secunda++;
}

void adc_init()  // Initializarea ADC
{
  ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
  ADMUX |= (1<<REFS0);// AVcc + capacitor extern la pinul Atref
  ADCSRA |= (1<<ADEN);// ADC - enable
  ADCSRA |= (1<<ADSC);	// incepe conversia ADC
}

uint16_t read_adc(uint8_t channel)
{
  ADCSRA |= (1<<ADSC);  // conversia incepe
  while(ADCSRA & (1<<ADSC));  // asteapta cat timp conversia ADC nu este actualizata
  return ADCW;  // citeste si afiseaza tensiunea
}
