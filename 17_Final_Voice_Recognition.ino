#include <Picovoice_EN.h>

#include "params.h"

#define MEMORY_BUFFER_SIZE (70 * 1024)
#define RED_LED 22
#define GREEN_LED 23
const int OUT_PIN = 4;

static const char* ACCESS_KEY = "fmgmpjJlXLBO8kk+Ar+MTHhsbgH2788PGTUmnUFF5GK343/3RXp67A=="; //AccessKey string obtained from Picovoice Console (https://picovoice.ai/console/)

static pv_picovoice_t *handle = NULL;

static int8_t memory_buffer[MEMORY_BUFFER_SIZE] __attribute__((aligned(16)));

static const float PORCUPINE_SENSITIVITY = 0.75f;
static const float RHINO_SENSITIVITY = 0.5f;
static const float RHINO_ENDPOINT_DURATION_SEC = 1.0f;
static const bool RHINO_REQUIRE_ENDPOINT = true;

static void out_state(String color, String state){
  /*int32_t pin = 0;

  if(color == ""){
    pin = OUT_PIN;
  } else if(color == "blue"){
    pin = OUT_PIN;
  } else if(color == "green"){
    pin = OUT_PIN;
  }*/
  if(state == "on"){
    digitalWrite(OUT_PIN, HIGH);
  }/*else{
    digitalWrite(OUT_PIN, LOW);
  }*/
  if(state == "off"){
    digitalWrite(OUT_PIN, LOW);
  }
}

static void wake_word_callback(void) {
    Serial.println("Wake word detected!");
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    delay(1000);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
}

static void inference_callback(pv_inference_t *inference) {
    Serial.println("{");
    Serial.print("    is_understood : ");
    Serial.println(inference->is_understood ? "true" : "false");
    if (inference->is_understood) {
        Serial.print("    intent : ");
        Serial.println(inference->intent);
        if (inference->num_slots > 0) {
            Serial.println("    slots : {");
            for (int32_t i = 0; i < inference->num_slots; i++) {
                Serial.print("        ");
                Serial.print(inference->slots[i]);
                Serial.print(" : ");
                Serial.println(inference->values[i]);
            }
            Serial.println("    }");
        }
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
        delay(1000);
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    } else{
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      delay(1000);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, HIGH);
    }
    Serial.println("}\n");
    //pv_inference_delete(inference);
    if(inference->is_understood){
      if(String(inference->intent) == "changeLightState"){
        String color = "";
        String state = "";
        
        for(int32_t i=0; i<inference->num_slots; i++){
          if(String(inference->slots[i]) == "state"){
            state = String(inference->values[i]);
          } else if(String(inference->slots[i]) == "color"){
            color = String(inference->values[i]);
          }
        }
        if(color == "" ){ 
          /*out_state("red", state);
          out_state("green", state);
          out_state("blue", state);*/
          out_state(color, state);
        /*} else{
          out_state(color, state);*/
        }
      }
    }
    pv_inference_delete(inference);
}

void setup() {
    Serial.begin(9600);

    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    pinMode(OUT_PIN, OUTPUT);

    pv_status_t status = pv_audio_rec_init();
    if (status != PV_STATUS_SUCCESS) {
        Serial.print("Audio init failed with ");
        Serial.println(pv_status_to_string(status));
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        delay(1000);
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        while (1);
    }

    status = pv_picovoice_init(
            ACCESS_KEY,
            MEMORY_BUFFER_SIZE,
            memory_buffer,
            sizeof(KEYWORD_ARRAY),
            KEYWORD_ARRAY,
            PORCUPINE_SENSITIVITY,
            wake_word_callback,
            sizeof(CONTEXT_ARRAY),
            CONTEXT_ARRAY,
            RHINO_SENSITIVITY,
            RHINO_ENDPOINT_DURATION_SEC,
            RHINO_REQUIRE_ENDPOINT,
            inference_callback,
            &handle);
    if (status != PV_STATUS_SUCCESS) {
        Serial.print("Picovoice init failed with ");
        Serial.println(pv_status_to_string(status));
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        delay(1000);
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        while (1);
    }

    const char *rhino_context = NULL;
    status = pv_picovoice_context_info(handle, &rhino_context);
    if (status != PV_STATUS_SUCCESS) {
        Serial.print("retrieving context info failed with");
        Serial.println(pv_status_to_string(status));
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        delay(1000);
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        while (1);
    }
    Serial.println("Wake word: 'hey computer'");
    Serial.println(rhino_context);
}

void loop()
{
    const int16_t *buffer = pv_audio_rec_get_new_buffer();
    if (buffer) {
        const pv_status_t status = pv_picovoice_process(handle, buffer);
        if (status != PV_STATUS_SUCCESS) {
            Serial.print("Picovoice process failed with ");
            Serial.println(pv_status_to_string(status));
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);
            delay(1000);
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
            while(1);
        }
    }
}
