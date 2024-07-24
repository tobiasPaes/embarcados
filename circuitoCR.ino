#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static TimerHandle_t capacidade = NULL;
static TimerHandle_t showCapacitor = NULL;

static int PV = 34;

float convertido;
int voltagemCapacitor;
float vcc = 0;

void lerCapacitor(TimerHandle_t xTimer){

  //ler o valor
  voltagemCapacitor = analogRead(PV);

  //converter o valor

  convertido = voltagemCapacitor *(vcc/4096);

  //printar o valor no serial
  
  Serial.println(xTaskGetTickCount()/1000.);
  Serial.println(", ");
  Serial.println(String(vcc));
  Serial.println(", ");
  Serial.println(String(convertido));
}

void mostrarCapacitor(TimerHandle_t xTimer){
  digitalWrite(25, HIGH);
  vcc = 3.3;

}

void setup() {
  
  Serial.begin(115200);
  // put your setup code here, to run once:
  capacidade = xTimerCreate(
                      "One-shot timer",     // Name of timer
                      200,            // Period of timer (in ticks)
                      pdTRUE,              // Auto-reload
                      (void *)0,            // Timer ID
                      lerCapacitor);  // Callback function

 showCapacitor = xTimerCreate(
                      "One-shot timer",     // Name of timer
                      8000,            // Period of timer (in ticks)
                      pdFALSE,              // Auto-reload
                      (void *)1,            // Timer ID
                      mostrarCapacitor);  // Callback function

xTimerStart(capacidade, 0);  // Inicia o timer capacidade
  xTimerStart(showCapacitor, 0);  // Inicia o timer showCapacitor
}

void loop() {
  // put your main code here, to run repeatedly:

}
