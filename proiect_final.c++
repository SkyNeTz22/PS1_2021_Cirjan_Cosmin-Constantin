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
  MENU_KP,
  MENU_TEMP,
  MENU_T_INCALZIRE,
  MENU_T_MENTINERE,
  MENU_T_RACIRE,
  MENU_KI,
  MENU_KD,
  MENU_PID,
  MENU_MAX_NUM,
};

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
double temp = 36.6;
double kp = 9.6;
float temp_q = 0;
double t_incalzire=60;
double t_mentinere=5;
double t_racire=22.5;
double kd=5.44;
double ki=16;
Menus scroll_menu = MENU_MAIN;
Menus current_menu =  MENU_MAIN;
double eroare = 0;
double suma_erori = 0;
double eroare_anterioara = 0;
double derivativa = 0;
double dt = 0.001; // timp de esantionare
double setpoint = 30;
float temperatura;
double temperatura_citita_senzor;
double output;
unsigned int numar_CAN;
double moving_setpoint = 20;
int activat;
unsigned ora=9, zora=0, sec=0, zsec=5, min=9, zmin=5, indentare_pe_secunda=0;

void state_machine(enum Menus menu, enum Buttons button);
Buttons GetButtons(void);
void print_menu(enum Menus menu);

typedef void (state_machine_handler_t)(void);

void afisare_timp(void)
{
  int min = 0;
  int sec  = 0;
  int remaining = 0;
  
  lcd.setCursor(0,1);
  lcd.print("P:");
  lcd.print(moving_setpoint);
  
  double now = millis();
  int total_seconds = now/1000;
  
  if (total_seconds <= t_incalzire)
  {
    Serial.print("Inc: ");
    Serial.println(total_seconds);

    moving_setpoint = moving_setpoint + (temp - moving_setpoint) * total_seconds  / t_incalzire;
	Serial.print("SP: ");
    Serial.println(moving_setpoint);
  }
  
  else if(total_seconds <= (t_incalzire + t_mentinere))
  {
  	Serial.print("Men: ");
    Serial.println(total_seconds);
    moving_setpoint = temp;
  } 
          
  else if(total_seconds <= (t_incalzire + t_mentinere + t_racire))
  {
    Serial.print("Rac: ");
    Serial.println(total_seconds);
    moving_setpoint = moving_setpoint + (temp - moving_setpoint) - (temp - moving_setpoint);
  }
  
  else
  {
    Serial.print("Oprit: ");
    Serial.println(total_seconds);
  }
  
  lcd.print(" t: ");
  lcd.print(total_seconds);
}

void print_menu(enum Menus menu)
{
  lcd.clear();
  switch(menu)
  {
    case MENU_KP:
    	lcd.print("KP = ");
    	lcd.print(kp);
    	break;
    case MENU_TEMP:
    	lcd.print("TEMP = ");
    	lcd.print(temp);
    	break;
    case MENU_T_INCALZIRE:
    	lcd.print("t_incalzire = ");
    	lcd.print(t_incalzire);
    	break;
    case MENU_T_MENTINERE:
    	lcd.print("t_mentinere = ");
    	lcd.print(t_mentinere);
    	break;
    case MENU_T_RACIRE:
    	lcd.print("t_racire = ");
    	lcd.print(t_racire);
    	break;
    case MENU_KI:
    	lcd.print("Ki = ");
    	lcd.print(ki);
    	break;
    case MENU_KD:
    	lcd.print("KD = ");
    	lcd.print(kd);
    	break;
    case MENU_PID:
    	lcd.print("PID");
        pid();
    	break;
    case MENU_MAIN:
    default:
    	lcd.print("PS 2021");
    	lcd.print("  T:");
    	lcd.print(indentare_pe_secunda);
    	lcd.print("s");
   		break;
  }
  if(current_menu != MENU_MAIN)
  {
  	lcd.setCursor(0,1);
  	lcd.print("modifica");
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

void save_kp(void)
{
}

void inc_kp(void)
{
  kp+=0.1;
}

void dec_kp(void)
{
  kp-=0.1;
}

void save_temp(void)
{
}

void inc_temp(void)
{
  temp++;
}

void dec_temp(void)
{
  temp--;
}
void save_t_incalzire(void)
{
 
}
void inc_t_incalzire(void)
{
 t_incalzire++; 
}
void dec_t_incalzire(void)
{
 t_incalzire--; 
}
void save_t_mentinere(void)
{
 
}
void inc_t_mentinere(void)
{
 t_mentinere++; 
}
void dec_t_mentinere(void)
{
 t_mentinere--; 
}
void save_t_racire(void)
{
 
}
void inc_t_racire(void)
{
 t_racire++; 
}
void dec_t_racire(void)
{
 t_racire--; 
}
void save_ki(void)
{
}

void inc_ki(void)
{
  ki++;
}

void dec_ki(void)
{
  ki--;
}
void save_kd(void)
{
}

void inc_kd(void)
{
  kd++;
}

void dec_kd(void)
{
  kd--;
}

void pid_activ(void)
{
  activat=1;
}

//matrice ; inca o linie pentru PID ; buton separat de start
state_machine_handler_t* sm[MENU_MAX_NUM][EV_MAX_NUM] = 
{ //events: OK(dreapta) , CANCEL(stanga) , NEXT(jos), PREV(sus)
  {enter_menu, go_home, go_next, go_prev},  // MENU_MAIN
  {go_home, go_home, inc_kp, dec_kp},       // MENU_KP
  {go_home, go_home, inc_temp, dec_temp},   // MENU_TEMP
  {go_home,go_home,inc_t_incalzire,dec_t_incalzire},//MENU_T_INCALZIRE
  {go_home,go_home,inc_t_mentinere,dec_t_mentinere},//MENU_T_MENTINERE	
  {go_home,go_home,inc_t_racire,dec_t_racire},//MENU_T_RACIRE
  {go_home,go_home,inc_ki,dec_ki},//MENU_KI
  {go_home,go_home,inc_kd,dec_kd},//MENU_KD
  {go_home,go_home,pid_activ,pid_activ},// activare PID
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
  // Serial.print(ret_val);
  return ret_val;
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

void loop()
{
  adc_init();
  numar_CAN = read_adc(0);
  lcd.setCursor(0, 0);
  temperatura  = ((numar_CAN)*(5000.00/1023)-500)/10;
  if(activat == 1)
  {
    pid();
    afisare_timp();
  }
  
  
  volatile Buttons event = GetButtons();
  if (event != EV_NONE)
  {
    state_machine(current_menu, event);
  }
    print_menu(scroll_menu);
    delay(1000);
}

double pid()
{
  eroare = moving_setpoint - temperatura_citita_senzor;
  suma_erori= suma_erori + eroare * dt;
  derivativa = (eroare - eroare_anterioara) / dt;
  output = (kp * eroare) + (ki * suma_erori ) + (kd * derivativa);
  eroare_anterioara = eroare;
  if(output > 255)
    output = 255;
  else if(output < 0)
    output = 0;
  analogWrite(10, output); //0....255
  Serial.print("Output = ");
  Serial.println(output);
  Serial.println();
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
