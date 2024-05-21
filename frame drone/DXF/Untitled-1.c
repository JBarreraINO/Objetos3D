#include <avr/io.h>
#include <avr/interrupt.h>

// Definición de pines y puertos
#define SENSOR_PROXIMITY_A_PIN PD2  // Sensor de proximidad A
#define SENSOR_PROXIMITY_B_PIN PD3  // Sensor de proximidad B
#define START_BUTTON_PIN PD4        // Pulsador de inicio
#define EMERGENCY_STOP_PIN PD5      // Pulsador de parada de emergencia
#define PWM_MOTOR_PIN PD6           // Pin para el motor PWM
#define MIX_LED_PIN PB0             // LED indicador de mezcla
#define STOP_LED_PIN PC0            // LED indicador de STOP
#define SERVO_PIN PC1               // Pin para el servo motor
#define DIGITAL_OUTPUT1_PIN DDC2    // Salida digital 1
#define DIGITAL_OUTPUT2_PIN DDC3    // Salida digital 2
#define BUZZER_PIN PC4              // Buzzer
#define LED_PIN PB5                 // Define el pin al que está conectado el LED

// Estados del proceso
#define IDLE_STATE 0
#define MIXING_STATE 1
#define EMPTYING_STATE 2
int IngredienteA = 0;
int IngredienteB = 0;
// Variables globales



int digital_output_active = 0;  // Variable para rastrear si las señales de control digital están activas
int estado_pulsador_asc = 0;    // estado actual del pulsador
int lastButtonState_asc = 0;    // estado anterior del pulsador


// Función para inicializar periféricos
void init_peripherals() {
  // Configurar el pin del sensor de proximidad A como entrada
  DDRD &= ~(1 << SENSOR_PROXIMITY_A_PIN);
  // Configurar el pin del sensor de proximidad B como entrada
  DDRD &= ~(1 << SENSOR_PROXIMITY_B_PIN);
  // Configurar el pin del botón de inicio como entrada
  DDRD &= ~(1 << START_BUTTON_PIN);
  // Configurar el pin del botón de parada de emergencia como entrada
  DDRD &= ~(1 << EMERGENCY_STOP_PIN);

  //SALIDAS
  DDRD |= (1 << PWM_MOTOR_PIN);  // PWM motor como salida
  DDRB |= (1 << MIX_LED_PIN);    // LED de mezcla como salida

  // Configurar el pin del LED de STOP como salida
  DDRC |= (1 << STOP_LED_PIN);
  DDRB |= (1 << LED_PIN);  // Configura el pin del LED como salida
  // Configurar el pin del servo como salida
  DDRC |= (1 << SERVO_PIN);

  // Configurar el pin de la salida digital 1 como salida
  DDRC |= (1 << DIGITAL_OUTPUT1_PIN);

  // Configurar el pin de la salida digital 2 como salida
  DDRC |= (1 << DIGITAL_OUTPUT2_PIN);

  // Configurar el pin del buzzer como salida
  DDRC |= (1 << BUZZER_PIN);




  // Configurar pull- DOWN PARA LA ENTRADA
  //PORTD &= ~(1 << SENSOR_PROXIMITY_A_PIN) | (1 << SENSOR_PROXIMITY_B_PIN) | (1 << START_BUTTON_PIN) | (1 << EMERGENCY_STOP_PIN);

  // Configurar temporizador para PWM
  TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);  // Modo PWM rápido, no inversor
  TCCR0B |= (1 << CS00);                                  // Sin preescalador
  OCR0A = 0;                                              // Inicializa el PWM en 0




  DDRB |= (1 << PB1);

  // Configura el modo de PWM en el Timer/Counter1
  // Modo Fast PWM de 8 bits, sin inversión
  TCCR1A |= (1 << COM1A1) | (1 << WGM10);
  TCCR1B |= (1 << WGM12) | (1 << CS10);  // Prescaler de 1








  // Configurar Timer1 para temporización
  TCCR1B |= (1 << WGM12);   // Modo CTC
  TIMSK1 |= (1 << OCIE1A);  // Habilitar interrupción de comparación A
  OCR1A = 15624;            // Valor de comparación para temporizador de 1 segundo a 8MHz

  // Configurar interrupciones externas
  EICRA |= (1 << ISC00) | (1 << ISC01);  // Interrupción en cualquier cambio de nivel
  EIMSK |= (1 << INT0) | (1 << INT1);    // Habilitar interrupción externa 0 y 1
}

// Función para activar el motor PWM con un cierto ciclo de trabajo
void set_pwm_duty_cycle(uint8_t duty_cycle) {
  OCR0A = duty_cycle;
}
void PWM_SetDutyCycle(uint8_t dutyCycle) {
  // Ajusta el ciclo de trabajo del PWM
  OCR1A = dutyCycle;
}

// Función para activar el servo en una posición específica (0 a 255)
void set_servo_position(uint8_t position) {
  OCR1A = 125 + (position / 2);
}

// Función para manejar la lógica de control del proceso
void process_control_logic() {

   int state = 0;
  int proximity_A_count;
  int proximity_B_count;

    if (state == 0) {
    if ((PIND & (1 << START_BUTTON_PIN))) {
  
          PORTB |= (1 << MIX_LED_PIN);  // Encender LED de mezcla
          PORTC |= (1 << PORTC2);  // Activar salidas digitales
          PORTC |= (1 << PORTC3);  // Activar salidas digitales
          digital_output_active = 1;
          proximity_A_count = 0;
          proximity_B_count = 0;
          IngredienteA = 0;
          IngredienteB = 0;
          set_pwm_duty_cycle(102);  // 40% de ciclo útil para el motor
          PWM_SetDutyCycle(127);

          state = 1;

    }
   } 

   else if (state == 1){    
     
        if ((PIND & (1 << SENSOR_PROXIMITY_A_PIN))) {

          PORTB |= (1 << LED_PIN);
          lastButtonState_asc = 1;
        }

        if (!(PIND & (1 << SENSOR_PROXIMITY_A_PIN))) {
          PORTB &= ~(1 << LED_PIN);
          estado_pulsador_asc = 1;
        }

        if ((lastButtonState_asc == 1 && estado_pulsador_asc == 1)) {
          proximity_A_count++;
         state = IDLE_STATE;
           lastButtonState_asc = 0;
          estado_pulsador_asc = 0;
        }


        // Verificar si se completaron las cantidades de A y B
        if (proximity_A_count >= 3) {
          // Desactivar señales de control digital
          PORTC &= ~(1 << PORTC3);
          IngredienteA = 1;
        }

        if (proximity_B_count >= 2) {
          // Desactivar señales de control digital
          PORTC &= ~(1 << PORTC2);
          IngredienteB = 1;
        }


        if (IngredienteA == 1 || IngredienteB == 1) {
          //digital_output_active = 0;

          // Aumentar velocidad de agitación
        
            set_pwm_duty_cycle(204);
          // 80% de ciclo útil para el motor
          PWM_SetDutyCycle(204);
          // Iniciar temporizador para el vaciado del tanque
          //TCNT1 = 0;
          state = EMPTYING_STATE;
          //break;
        }
      } 
      else if (state == EMPTYING_STATE) {
       
      PORTB &= ~(1 << MIX_LED_PIN);
      // Esperar 1 minuto antes de detener el motor y vaciar el tanque
      // Esto se maneja en la interrupción del temporizador
    }


 

  
  }
/*
// Función de interrupción para Timer1 (temporizador de 1 segundo)
ISR(TIMER1_COMPA_vect) {
  if (state == EMPTYING_STATE) {
    // Verificar si ha transcurrido 1 minuto
    if (TCNT1 >= 60) {
      // Detener el motor y vaciar el tanque
      set_pwm_duty_cycle(0);         // Detener el motor
      set_servo_position(135);       // Abrir la compuerta
      PORTC |= (1 << STOP_LED_PIN);  // Encender LED de STOP
      TCNT1 = 0;
      state = IDLE_STATE;
    }
  }
}

// Función de interrupción externa para el botón de parada de emergencia

ISR(INT0_vect) {
    // Detener el proceso y activar la alarma
   // state = IDLE_STATE;
    //set_pwm_duty_cycle(0); // Detener el motor
   // set_servo_position(135); // Abrir la compuerta
    //PORTC = (1 << STOP_LED_PIN) | (1 << BUZZER_PIN); // Encender LED de STOP y activar buzzer
    // Apagar todas las salidas excepto PC0
    PORTC = (1 << STOP_LED_PIN);
   // PORTB = 0x00;

}
*/

// Función de interrupción externa para el botón de inicio
/*
ISR(INT1_vect) {
    if (state == IDLE_STATE) {
        // Reiniciar el proceso
       
        PORTB |= (1 << MIX_LED_PIN); // Encender LED de mezcla
        PORTC |= (1 << DIGITAL_OUTPUT1_PIN) | (1 << DIGITAL_OUTPUT2_PIN); // Activar salidas digitales
        digital_output_active = 1;
        proximity_A_count = 0;
        proximity_B_count = 0;
        set_pwm_duty_cycle(102); // 40% de ciclo útil para el motor
         state = MIXING_STATE;
    }
 } */

int main() {
  // Inicializar periféricos
  init_peripherals();

  // Habilitar interrupciones globales
  sei();

  while (1) {
    // Lógica de control del proceso
    process_control_logic();
  }

  //return 0;
}