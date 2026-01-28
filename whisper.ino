#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "driver/i2s.h

// --- CONFIGURATION PINS ---
#define PIN_BUTTON    2   
#define PIN_SD_CS     3   
#define I2S_MIC_SD    1   
#define I2S_MIC_WS    43  
#define I2S_MIC_SCK   44  
#define I2S_HP_DIN    4   
#define I2S_HP_LRC    5   
#define I2S_HP_BCLK   6   


// === RÉGLAGE DES PARAMÈTRES ===

#define BASE_RATE        16000 
#define MIC_INPUT_GAIN   4      

// --- VOLUMES ---
float volIntroTurn     = 0.5;  
float volBaseRecord    = 0.7;  
float volPlayerRecord  = 3.5;

// --- FILTRE NATUREL (Auto-Wah / LPF Dynamique) ---
bool  useOrganicFilter = true; 
float filterFreq       = 0.6; // Vitesse de la variation (en Hz)
float filterIntensity  = 0.8; // 0.1 (faible) à 0.9 (très étouffé)

// --- OPTIONS ---
bool  useLockSystem    = true;      
unsigned long lockDuration = 60000; 

// Autres filtres (Désactivés pour privilégier le filtre naturel)
bool  useEcho          = false;  
float echoFeedback     = 0.2;
// ================================================================

#define I2S_PORT_MIC     I2S_NUM_0
#define I2S_PORT_HP      I2S_NUM_1
#define BUFFER_SIZE      512

String sessionPath = "";
String currentReferenceAudio = "/RECORD.wav"; 
unsigned long endOfPlayTime = 0, lockStartTime = 0;   
bool waitingForRecording = false, isRecording = false, isPlaying = false, isLocked = false;             

// --- VARIABLES FILTRES ---
float filterPhase = 0;
float lastFilteredSample = 0;
int16_t echoBuffer[2000]; 
int echoIndex = 0;

void setupI2S() {
  i2s_config_t mic_cfg = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), .sample_rate = BASE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 8, .dma_buf_len = BUFFER_SIZE, .use_apll = false };
  i2s_driver_install(I2S_PORT_MIC, &mic_cfg, 0, NULL);
  i2s_pin_config_t mic_p = { .bck_io_num = I2S_MIC_SCK, .ws_io_num = I2S_MIC_WS, .data_out_num = -1, .data_in_num = I2S_MIC_SD };
  i2s_set_pin(I2S_PORT_MIC, &mic_p);

  i2s_config_t hp_cfg = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = BASE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 8, .dma_buf_len = BUFFER_SIZE, .use_apll = false };
  i2s_driver_install(I2S_PORT_HP, &hp_cfg, 0, NULL);
  i2s_pin_config_t hp_p = { .bck_io_num = I2S_HP_BCLK, .ws_io_num = I2S_HP_LRC, .data_out_num = I2S_HP_DIN, .data_in_num = -1 };
  i2s_set_pin(I2S_PORT_HP, &hp_p);
}

void playFile(String path, bool applyFilters, float volume) {
  File file = SD.open(path);
  if (!file) return;

  file.seek(24);
  uint32_t fileSampleRate;
  file.read((uint8_t*)&fileSampleRate, 4);
  i2s_set_sample_rates(I2S_PORT_HP, fileSampleRate);
  
  // Reset pour éviter les bruits résiduels
  i2s_zero_dma_buffer(I2S_PORT_HP);
  memset(echoBuffer, 0, sizeof(echoBuffer));
  lastFilteredSample = 0;
  
  file.seek(44);
  int16_t buffer[BUFFER_SIZE];
  int16_t stereoBuf[BUFFER_SIZE * 2];
  size_t bytes_w;

  // Variables pour le fondu (Fade)
  long totalSamples = (file.size() - 44) / 2;
  long currentSampleCount = 0;
  const int fadeSamples = 800;

  while (file.available()) {
    int bytes_r = file.read((uint8_t*)buffer, sizeof(buffer));
    int samples = bytes_r / 2;

    for (int i = 0; i < samples; i++) {
      float s = (float)buffer[i];
      long globalPos = currentSampleCount + i;

      // --- FONDU ANTI-CLAC (Entrée/Sortie) ---
      float fade = 1.0f;
      if (globalPos < fadeSamples) fade = (float)globalPos / fadeSamples;
      else if (globalPos > (totalSamples - fadeSamples)) fade = (float)(totalSamples - globalPos) / fadeSamples;

      if (applyFilters) {
        // --- FILTRE NATUREL HÉTÉROGÈNE (Low Pass Dynamique) ---
        if (useOrganicFilter) {
          float wave = (sin(filterPhase) + 1.0f) / 2.0f;
          float alpha = 0.05f + (wave * (0.8f - filterIntensity)); 
          s = lastFilteredSample + (alpha * (s - lastFilteredSample));
          lastFilteredSample = s;
          filterPhase += (2.0f * PI * filterFreq) / (float)fileSampleRate;
          if (filterPhase > 2.0f * PI) filterPhase -= 2.0f * PI;
        }

        // --- ECHO ---
        if (useEcho) {
          float echoVal = (float)echoBuffer[echoIndex];
          echoBuffer[echoIndex] = (int16_t)constrain(s + echoVal * echoFeedback, -32000, 32000);
          s += echoVal * 0.4f;
          echoIndex = (echoIndex + 1) % 2000;
        }
      }

      s *= (volume * fade);
      int16_t out = (int16_t)constrain(s, -32000, 32000); 
      stereoBuf[i * 2] = out; stereoBuf[i * 2 + 1] = out;
    }
    i2s_write(I2S_PORT_HP, stereoBuf, samples * 4, &bytes_w, portMAX_DELAY);
    currentSampleCount += samples;
  }
  i2s_zero_dma_buffer(I2S_PORT_HP);
  file.close();
}

void writeWavHeader(File file) {
  byte header[44]; memset(header, 0, 44);
  memcpy(&header[0], "RIFF", 4); memcpy(&header[8], "WAVEfmt ", 8);
  header[16] = 16; header[20] = 1; header[22] = 1;
  uint32_t hz = BASE_RATE; memcpy(&header[24], &hz, 4);
  uint32_t br = BASE_RATE * 2; memcpy(&header[28], &br, 4);
  header[32] = 2; header[34] = 16; memcpy(&header[36], "data", 4);
  file.write(header, 44);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  if (!SD.begin(PIN_SD_CS)) while(1);
  int n = 0; while (SD.exists("/session_" + String(n))) n++;
  sessionPath = "/session_" + String(n);
  SD.mkdir(sessionPath);
  setupI2S();
}

void loop() {
  if (isLocked && (millis() - lockStartTime > lockDuration)) { isLocked = false; Serial.println("Debloque."); }
  if (waitingForRecording && (millis() - endOfPlayTime > 20000)) { waitingForRecording = false; playFile("/BAD.wav", false, volIntroTurn); }

  if (digitalRead(PIN_BUTTON) == LOW) {
    if (isLocked && !isPlaying) {
      isPlaying = true; playFile("/LOCK.wav", false, volIntroTurn); isPlaying = false;
    }
    else if (!isLocked && !waitingForRecording && !isPlaying) {
      isPlaying = true;
      playFile("/Intro_1.wav", false, volIntroTurn);
      float v = (currentReferenceAudio == "/RECORD.wav") ? volBaseRecord : volPlayerRecord;
      playFile(currentReferenceAudio, true, v); 
      playFile("/Turn.wav", false, volIntroTurn);
      endOfPlayTime = millis(); waitingForRecording = true; isPlaying = false;
    } 
    else if (!isLocked && waitingForRecording && !isRecording) {
      unsigned long ps = millis();
      String tmp = sessionPath + "/temp.wav";
      File audioFile = SD.open(tmp, FILE_WRITE);
      if (audioFile) {
        writeWavHeader(audioFile);
        isRecording = true;
        while(digitalRead(PIN_BUTTON) == LOW) {
          int16_t m_buf[BUFFER_SIZE]; size_t b_r;
          i2s_read(I2S_PORT_MIC, m_buf, sizeof(m_buf), &b_r, portMAX_DELAY);
          for (int i = 0; i < b_r/2; i++) {
            int32_t amp = (int32_t)m_buf[i] * MIC_INPUT_GAIN;
            m_buf[i] = (int16_t)constrain(amp, -32000, 32000);
          }
          audioFile.write((uint8_t*)m_buf, b_r);
          if (millis() - ps > 30000) break;
        }
        audioFile.close();
        if (millis() - ps >= 2000) {
          String finalFile = sessionPath + "/rec_" + String(millis()) + ".wav";
          SD.rename(tmp, finalFile);
          currentReferenceAudio = finalFile;
          waitingForRecording = false;
          playFile("/GOOD.wav", false, volIntroTurn);
          if (useLockSystem) { isLocked = true; lockStartTime = millis(); }
        } else { SD.remove(tmp); }
        isRecording = false;
      }
    }
    while(digitalRead(PIN_BUTTON) == LOW);
  }
}