#include <Arduino.h>

#if !defined(CONFIG_IDF_TARGET_ESP32)
  #error "This code is intended exclusively for classic ESP32 (CONFIG_IDF_TARGET_ESP32)! Please check your target board selection in Arduino IDE."
#endif

#include "version.h"

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

#include "esp_mac.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "driver/i2s_std.h"

// Definice pinů
#define I2S_BCLK_PIN    GPIO_NUM_27   // Společný Bit Clock
#define I2S_WS_PIN      GPIO_NUM_25   // Společný Word Select (LRCLK)
#define I2S_DOUT_PIN    GPIO_NUM_26   // Data Out (Vysílání do DAC / BT RX mód)
#define I2S_DIN_PIN     GPIO_NUM_33   // Data In (Příjem z ADC / BT TX mód)

// Globální i2s handle
static i2s_chan_handle_t tx_chan = NULL;
static i2s_chan_handle_t rx_chan = NULL;

// Mutex pro ochranu audioRingBuffer (řeší race condition)
static SemaphoreHandle_t audioBufferMutex = NULL;

// Media global variables
String trackTitle  = "Unknown Title";
String trackArtist = "Unknown Artist";
String trackAlbum  = "Unknown Album";

// FIX PRO ESP32 ARDUINO CORE 3.3.x
extern "C" bool btInUse(void) {
    return true;
}

#ifdef __cplusplus
extern "C" {
#endif
    // Nativní dekodér z ROM/HAL pro ESP32
    uint8_t temprature_sens_read(void);
#ifdef __cplusplus
}
#endif

// ==========================================
// KONFIGURACE A DEFINICE
// ==========================================
#define UART_BAUD_RATE      460800
#define CONFIRM_TIMEOUT_MS  2000
#define RING_BUFFER_SIZE    (64 * 1024) // 64 kB RingBuffer

// Defaultní hodnoty
const char* DEFAULT_MODE = "TX";
const uint8_t DEFAULT_VOLUME = 13; // 10 % z rozsahu 0-127
const float DEFAULT_GAIN = 1.00f;

// Globální stavové proměnné
Preferences prefs;
String btMode = "TX";
String btName = "";
uint8_t volume = DEFAULT_VOLUME;
float gain = DEFAULT_GAIN;
bool isMuted = false;
bool isConnected = false;

// Globální proměnné pro stav spojeného zařízení
uint8_t connected_bda[6] = {0};
String currentDeviceName = "";

// RingBuffer Handle pro audio stream
RingbufHandle_t audioRingBuffer = NULL;

// RTOS Task Handles
TaskHandle_t audioTaskHandle = NULL;

// Proměnné pro 2-fázový Handshake
enum PendingCmd { CMD_NONE, PENDING_REBOOT, PENDING_RESET };
PendingCmd pendingCmd = CMD_NONE;
unsigned long pendingCmdTime = 0;

// Prototypy
void processUartCommand(String cmd);
void initBluetoothStack();
void audioProcessingTask(void *pvParameters);
String generateDefaultName();

#define USE_INMP441_MIC  0 // Set 0 pro vypnutí mikrofonu (např. při příjmu z jiného ESP)
#define AMP_GAIN         3 // +18dB zesílení

// ==========================================
// KONFIGURACE RINGBUFFERU (OPRAVA #6c)
// ==========================================
#define RING_BUFFER_INITIAL_SIZE   (32 * 1024)   // Počáteční 32 kB
#define RING_BUFFER_MAX_SIZE       (128 * 1024)  // Max 128 kB
#define RING_BUFFER_CRITICAL_LEVEL (28 * 1024)   // 87.5% → cleanup

// ==========================================
// DIAGNOSTIKA A LOGOVÁNÍ VÝPADKŮ (OPRAVA #6)
// ==========================================

typedef struct {
    unsigned long mutexTimeoutCount;
    unsigned long ringbufferOverflowCount;
    unsigned long i2sReadFailCount;
    unsigned long silenceFrameCount;
    unsigned long lastMutexTimeoutTime;
    unsigned long lastOverflowTime;
    unsigned long lastI2sFailTime;
    uint32_t audioDataDroppedBytes;
    uint32_t ringbufferMaxUsage;  // Sledování maximální zaplnění
    uint32_t ringbufferCurrentUsage;
} AudioDiagnostics;

static AudioDiagnostics audioDiag = {0};

void printAudioDiagnostics() {
    Serial.printf("DIAG:MUTEX_TIMEOUTS=%lu,OVERFLOW_COUNT=%lu,I2S_FAILS=%lu,SILENCE_FRAMES=%lu,DROPPED_BYTES=%lu\n",
                  audioDiag.mutexTimeoutCount,
                  audioDiag.ringbufferOverflowCount,
                  audioDiag.i2sReadFailCount,
                  audioDiag.silenceFrameCount,
                  audioDiag.audioDataDroppedBytes);
    Serial.printf("DIAG:LAST_MUTEX_TIMEOUT=%lu,LAST_OVERFLOW=%lu,LAST_I2S_FAIL=%lu\n",
                  audioDiag.lastMutexTimeoutTime,
                  audioDiag.lastOverflowTime,
                  audioDiag.lastI2sFailTime);
    Serial.printf("DIAG:RINGBUFFER_MAX_USAGE=%lu,CURRENT_USAGE=%lu\n",
                  audioDiag.ringbufferMaxUsage,
                  audioDiag.ringbufferCurrentUsage);
}

void resetAudioDiagnostics() {
    memset(&audioDiag, 0, sizeof(AudioDiagnostics));
    Serial.println("INFO:DIAGNOSTICS_RESET");
}

// Funkce pro načtení z I2S a úpravu pro BT / další zpracování
int read_and_process_audio(int16_t *out_pcm16_buffer, size_t max_samples) {
#if USE_INMP441_MIC
    static int32_t raw_i2s_buffer[256];
    size_t bytes_read = 0;

    if (i2s_channel_read(rx_chan, raw_i2s_buffer, sizeof(raw_i2s_buffer), &bytes_read, pdMS_TO_TICKS(10)) == ESP_OK) {
        int mono_samples = bytes_read / sizeof(int32_t);
        int out_idx = 0;

        for (int i = 0; i < mono_samples; i++) {
            if (out_idx + 1 >= (int)max_samples) break;
            int32_t sample = raw_i2s_buffer[i] >> 16; 
            sample = sample << AMP_GAIN;
            if (sample > 32767)       sample = 32767;
            else if (sample < -32768) sample = -32768;
            int16_t sample_16 = (int16_t)sample;
            out_pcm16_buffer[out_idx++] = sample_16; 
            out_pcm16_buffer[out_idx++] = sample_16; 
        }
        return out_idx;
    } else {
        // ⚠️ NOVÉ: Logování selhání
        audioDiag.i2sReadFailCount++;
        audioDiag.lastI2sFailTime = millis();
        static unsigned long lastErrorReport = 0;
        if (millis() - lastErrorReport > 5000) { // Log pouze každých 5 sec
            Serial.printf("WARN:I2S_READ_TIMEOUT,COUNT=%lu\n", audioDiag.i2sReadFailCount);
            lastErrorReport = millis();
        }
    }
    return 0;
#else
    static int32_t raw_i2s_32bit_buffer[256];           // bylo 512
    size_t bytes_read = 0;
    
    size_t samples_to_read = max_samples; 
    if (samples_to_read > 256) samples_to_read = 256;   // bylo 512

    if (i2s_channel_read(rx_chan, raw_i2s_32bit_buffer, samples_to_read * sizeof(int32_t), &bytes_read, pdMS_TO_TICKS(10)) == ESP_OK) {
        int samples_count = bytes_read / sizeof(int32_t);

        // ⚠️ DEBUG: Logování I2S čtení
        static unsigned long lastDebugReport = 0;
        if (millis() - lastDebugReport > 5000) {  // Log každých 5 sekund
            Serial.printf("DEBUG:I2S_READ=%lu_bytes,SAMPLES=%d,MAX_SAMPLES=%zu\n", 
                          bytes_read, samples_count, max_samples);
            lastDebugReport = millis();
        }

        for (int i = 0; i < samples_count; i++) {
            int32_t val = raw_i2s_32bit_buffer[i] >> 14; 
            out_pcm16_buffer[i] = (int16_t)val; 
        }
        return samples_count; 
    } else {
        // ⚠️ NOVÉ: Logování selhání
        audioDiag.i2sReadFailCount++;
        audioDiag.lastI2sFailTime = millis();
    }
    return 0;
#endif
}

void initTempSensor() {
}

float getChipTemperature() {
    uint8_t raw = temprature_sens_read();
    if (raw == 0 || raw == 255) {
        return -999.0f; // Neplatná hodnota
    }
    // Přepočet z RAW hodnota na stupně Celsia pro ESP32
    return (float)(raw - 32) / 1.8f;
}

void clearAudioBuffer() {
    if (audioRingBuffer != NULL) {
        size_t dummySize = 0;
        void* item = NULL;
        while ((item = xRingbufferReceiveUpTo(audioRingBuffer, &dummySize, 0, 2048)) != NULL) {
            vRingbufferReturnItem(audioRingBuffer, item);
        }
    }
}

// De-inicializace a uvolnění I2S sběrnice
void i2s_stop_and_deinit() {
    if (tx_chan) {
        i2s_channel_disable(tx_chan);
        i2s_del_channel(tx_chan);
        tx_chan = NULL;
    }
    if (rx_chan) {
        i2s_channel_disable(rx_chan);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
    }
    Serial.println("I2S: Uvolněno");
}

// Inicializace pro TX (Vysílání do DAC / Příjem dat z Bluetooth)
bool i2s_init_tx(uint32_t sample_rate) {
    i2s_stop_and_deinit();

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 256;
    
    if (i2s_new_channel(&chan_cfg, &tx_chan, NULL) != ESP_OK) return false;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    if (i2s_channel_init_std_mode(tx_chan, &std_cfg) != ESP_OK) return false;
    if (i2s_channel_enable(tx_chan) != ESP_OK) return false;

    Serial.println("I2S: TX Inicializován (Master - výstup do DAC)");
    return true;
}

// Inicializace pro RX (Vstup z ADC / Odesílání dat přes Bluetooth)
bool i2s_init_rx(uint32_t sample_rate) {
    i2s_stop_and_deinit();

#if USE_INMP441_MIC
    // KONFIGURACE PRO INMP441 S PINEM L/R NA VDD
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
    slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT; // L/R pin na VDD = RIGHT slot
#else
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
#endif
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 256;
    if (i2s_new_channel(&chan_cfg, NULL, &rx_chan) != ESP_OK) return false;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN_PIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    if (i2s_channel_init_std_mode(rx_chan, &std_cfg) != ESP_OK) return false;
    if (i2s_channel_enable(rx_chan) != ESP_OK) return false;

#if USE_INMP441_MIC
    Serial.println("I2S: RX Inicializován (INMP441 32-bit MONO RIGHT)");
#else
    Serial.println("I2S: RX Inicializovan (Slave RX - vstup z ADC)");
#endif
    return true;
}

// -------------------------------------------------------------
// AVRCP Control Callback
// -------------------------------------------------------------
void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    switch (event) {
        case ESP_AVRC_CT_METADATA_RSP_EVT: {
            uint8_t id = param->meta_rsp.attr_id;
            String text = String((char*)param->meta_rsp.attr_text, param->meta_rsp.attr_length);

            if (id == ESP_AVRC_MD_ATTR_TITLE)  trackTitle = text;
            if (id == ESP_AVRC_MD_ATTR_ARTIST) trackArtist = text;
            if (id == ESP_AVRC_MD_ATTR_ALBUM)  trackAlbum = text;

            if (id == ESP_AVRC_MD_ATTR_TITLE) {
                Serial.printf("MEDIA:TITLE=%s|ARTIST=%s|ALBUM=%s\n", 
                              trackTitle.c_str(), trackArtist.c_str(), trackAlbum.c_str());
            }
            break;
        }

        case ESP_AVRC_CT_PLAY_STATUS_RSP_EVT:
            switch (param->play_status_rsp.play_status) {
                case ESP_AVRC_PLAYBACK_PLAYING:
                    Serial.println("MEDIA:PLAYING");
                    esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM);
                    break;
                case ESP_AVRC_PLAYBACK_PAUSED:
                case ESP_AVRC_PLAYBACK_STOPPED:
                    Serial.println("MEDIA:PAUSED");
                    break;
                default:
                    break;
            }
            break;

        case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
            if (param->psth_rsp.key_state == 0) {
                switch (param->psth_rsp.key_code) {
                    case ESP_AVRC_PT_CMD_PLAY:     Serial.println("EVENT:AVRCP,CMD=PLAY"); break;
                    case ESP_AVRC_PT_CMD_PAUSE:    Serial.println("EVENT:AVRCP,CMD=PAUSE"); break;
                    case ESP_AVRC_PT_CMD_FORWARD:  Serial.println("EVENT:AVRCP,CMD=NEXT"); break;
                    case ESP_AVRC_PT_CMD_BACKWARD: Serial.println("EVENT:AVRCP,CMD=PREV"); break;
                    default: break;
                }
            }
            break;

        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
            if (param->change_ntf.event_id == ESP_AVRC_RN_TRACK_CHANGE) {
                esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM);
            }
            break;

        default:
            break;
    }
}

// -------------------------------------------------------------
// AVRCP Target Callback
// -------------------------------------------------------------
void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    if (event == ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT) {
        uint8_t volume_raw = param->set_abs_vol.volume; 
        uint8_t volume_pct = (volume_raw * 100) / 127;
        Serial.printf("MEDIA:VOLUME,%d\n", volume_pct);
    }
}

// ==========================================
// A2DP DATA & CONNECTION CALLBACKS
// ==========================================
String bdaToString(const uint8_t *bda) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return String(buf);
}

void bt_a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    if (event == ESP_A2D_CONNECTION_STATE_EVT) {
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            // Při připojení nejdříve nastavíme stav, až pak čistíme buffer
            isConnected = true;
            memcpy(connected_bda, param->conn_stat.remote_bda, 6);
            currentDeviceName = "";

            String macStr = bdaToString(connected_bda);
            Serial.println("STATUS:CONNECTED,MAC=" + macStr);

            // Teprve teď čistíme buffer, když jsme si jisti, že jsme připojeni
            clearAudioBuffer();

            if (btMode == "RX") {
                esp_avrc_ct_send_get_play_status_cmd(0);
            } else if (btMode == "TX") {
                // EXPLICITNÍ SPUŠTĚNÍ STREAMU PRO TX (odesílání přes Bluetooth)
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            }

        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            isConnected = false;
            clearAudioBuffer();
            memset(connected_bda, 0, 6);
            currentDeviceName = "";
            Serial.println("STATUS:DISCONNECTED");
        }
    }
    // Přidejte obsluhu media state
    else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
        if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START &&
            param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            Serial.println("INFO: A2DP Media Stream Started");
        }
    }
}

int32_t bt_a2dp_data_cb(uint8_t *data, int32_t len) {
    if (data == NULL || len <= 0) return 0;

    if (isMuted || !isConnected) {
        clearAudioBuffer();
        memset(data, 0, len);
        return len;
    }

    if (audioBufferMutex != NULL && xSemaphoreTake(audioBufferMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        size_t bytesRead = 0;
        void* item = xRingbufferReceiveUpTo(audioRingBuffer, &bytesRead, 0, len);

        if (item != NULL && bytesRead > 0) {
            memcpy(data, item, bytesRead);
            vRingbufferReturnItem(audioRingBuffer, item);
            
            // Track trenutnu upotrebu
            audioDiag.ringbufferCurrentUsage = bytesRead;

            if (bytesRead < (size_t)len) {
                audioDiag.silenceFrameCount++;
                audioDiag.audioDataDroppedBytes += (len - bytesRead);
                memset(data + bytesRead, 0, len - bytesRead);
            }
        } else {
            audioDiag.silenceFrameCount++;
            memset(data, 0, len);
        }
        xSemaphoreGive(audioBufferMutex);
    } else {
        audioDiag.mutexTimeoutCount++;
        audioDiag.lastMutexTimeoutTime = millis();
        memset(data, 0, len);
        
        static unsigned long lastMutexReport = 0;
        if (millis() - lastMutexReport > 10000) {
            Serial.printf("WARN:MUTEX_TIMEOUT,COUNT=%lu\n", audioDiag.mutexTimeoutCount);
            lastMutexReport = millis();
        }
    }

    if (gain != 1.00f) {
        int16_t *samples = (int16_t *)data;
        size_t sampleCount = len / 2;
        for (size_t i = 0; i < sampleCount; i++) {
            int32_t sample = (int32_t)(samples[i] * gain);
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            samples[i] = (int16_t)sample;
        }
    }

    return len;
}

void bt_a2dp_sink_data_cb(const uint8_t *data, uint32_t len) {
    if (data == NULL || len == 0 || isMuted) return;

    static uint8_t workBuffer[512];  // Max A2DP frame
    
    // Kopírovat a aplikovat gain
    uint8_t *procData = (uint8_t*)data;
    if (gain != 1.00f && len <= sizeof(workBuffer)) {
        memcpy(workBuffer, data, len);
        int16_t *samples = (int16_t*)workBuffer;
        size_t sampleCount = len / 2;
        for (size_t i = 0; i < sampleCount; i++) {
            int32_t sample = (int32_t)(samples[i] * gain);
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            samples[i] = (int16_t)sample;
        }
        procData = workBuffer;
    }

    if (audioRingBuffer != NULL && audioBufferMutex != NULL && 
        xSemaphoreTake(audioBufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        xRingbufferSend(audioRingBuffer, procData, len, pdMS_TO_TICKS(10));
        xSemaphoreGive(audioBufferMutex);
    }
}

// ==========================================
// POMOCNÉ SPRÁVY PAMĚTI A SHODY DEFAULTU
// ==========================================
String generateDefaultName() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char nameBuf[64];  // OPRAVA #8: Zvětšen buffer z 32 na 64 pro větší bezpečnost
    snprintf(nameBuf, sizeof(nameBuf), "VTomRadio-MaSo-%02X%02X%02X", mac[3], mac[4], mac[5]);
    return String(nameBuf);
}

void loadNVSConfig() {
    prefs.begin("bt_config", false);
    btMode = prefs.getString("mode", DEFAULT_MODE);
    btName = prefs.getString("bt_name", generateDefaultName());
    volume = prefs.getUChar("volume", DEFAULT_VOLUME);
    gain = prefs.getFloat("gain", DEFAULT_GAIN);
    prefs.end();
}

void resetToDefaults() {
    prefs.begin("bt_config", false);
    prefs.clear();
    // Po prefs.clear() jsou data smazána a lze bezpečně zapsat nové hodnoty
    prefs.putString("mode", DEFAULT_MODE);
    prefs.putString("bt_name", generateDefaultName());
    prefs.putUChar("volume", DEFAULT_VOLUME);
    prefs.putFloat("gain", DEFAULT_GAIN);
    prefs.end();
    Serial.println("INFO:CONFIG_RESET_TO_DEFAULTS");
}

// ==========================================
// INICIALIZACE BT STACKU
// ==========================================
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
            if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
                // OPRAVA #7: Omezení délky jména na max 127 znaků pro bezpečnost
                currentDeviceName = String((char *)param->read_rmt_name.rmt_name).substring(0, 127);
            }
            break;

        case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
            if (param->read_rssi_delta.stat == ESP_BT_STATUS_SUCCESS) {
                Serial.printf("RSSI:%d\n", param->read_rssi_delta.rssi_delta);
            } else {
                Serial.println("ERR:RSSI_FAILED");
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT:
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        default:
            break;
    }
}

void bt_gap_search_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_DISC_RES_EVT) {
        // Nalezeno zařízení v okolí
        Serial.printf("INFO: Nalezeno BT zarizeni: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2],
                      param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);

        // Pokud nejsme připojeni, zkusíme se připojit k tomuto zařízení
        if (!isConnected && btMode == "TX") {
            Serial.println("INFO: Pokus o pripojeni...");
            esp_bt_gap_cancel_discovery(); // Zastavíme skenování
            esp_a2d_source_connect(param->disc_res.bda);
        }
    }
}

void initBluetoothStack() {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        Serial.printf("ERR: nvs_flash_init failed: %s\n", esp_err_to_name(ret));
    }

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_bt_controller_deinit();
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        Serial.printf("ERR: controller_init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        Serial.printf("ERR: controller_enable failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ret = esp_bluedroid_init();
        if (ret != ESP_OK) {
            Serial.printf("ERR: bluedroid_init failed: %s\n", esp_err_to_name(ret));
            return;
        }
    }

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        ret = esp_bluedroid_enable();
        if (ret != ESP_OK) {
            Serial.printf("ERR: bluedroid_enable failed: %s\n", esp_err_to_name(ret));
            return;
        }
    }

    esp_bt_dev_set_device_name(btName.c_str());
    esp_bt_gap_register_callback(bt_app_gap_cb);

    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_avrc_ct_cb);
    esp_avrc_tg_init();
    esp_avrc_tg_register_callback(bt_avrc_tg_cb);

    if (btMode == "TX") {
        esp_a2d_register_callback(bt_a2dp_cb);
        esp_a2d_source_init();
        esp_a2d_source_register_data_callback(bt_a2dp_data_cb);
    } else {
        esp_a2d_register_callback(bt_a2dp_cb);
        esp_a2d_sink_init();
        esp_a2d_sink_register_data_callback(bt_a2dp_sink_data_cb);
    }

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t io_cap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &io_cap, sizeof(uint8_t));

    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK) {
        Serial.printf("ERR: set_scan_mode failed: %s\n", esp_err_to_name(ret));
    } else {
        Serial.println("INFO: BT Scan mode set to CONNECTABLE & DISCOVERABLE");
    }
    if (btMode == "TX") {
        esp_bt_gap_register_callback(bt_gap_search_cb);
        Serial.println("INFO: Spoustim vyhledavani BT sluchatek (Inquiry)...");
        // Skenujeme 10 sekund (10 * 1.28s)
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0); 
    }
}

// OPRAVA #3: Přidání cleanup a lepší inicializace mutex/ringbuffer
// ==========================================
// THREAD PRO AUDIO / 2. JÁDRO (CORE 1)
// ==========================================
void audioProcessingTask(void *pvParameters) {
    audioBufferMutex = xSemaphoreCreateMutex();
    if (audioBufferMutex == NULL) {
        Serial.println("ERR:MUTEX_CREATION_FAILED");
        vTaskDelete(NULL);
        return;
    }

    audioRingBuffer = xRingbufferCreate(RING_BUFFER_INITIAL_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (audioRingBuffer == NULL) {
        Serial.println("ERR:RINGBUFFER_FAILED");
        vSemaphoreDelete(audioBufferMutex);
        audioBufferMutex = NULL;
        vTaskDelete(NULL);
        return;
    }

    static int16_t pcm16Buf[512];
    
    // OPRAVA #6d: Lokální buffer pro sběr dat do 2048 bytů (A2DP frame)
    static uint8_t localBuffer[2048];
    static size_t localBufferUsed = 0;

    for (;;) {
        if (btMode == "TX" && rx_chan != NULL) {
            int total_samples = read_and_process_audio(pcm16Buf, sizeof(pcm16Buf) / sizeof(int16_t));
            
            if (isMuted || !isConnected) {
                clearAudioBuffer();
                localBufferUsed = 0;  // Reset lokálního bufferu
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            if (total_samples > 0 && audioRingBuffer != NULL) {
                size_t bytesToAdd = total_samples * sizeof(int16_t);
                
                // Kopírovat do lokálního bufferu
                if (localBufferUsed + bytesToAdd <= sizeof(localBuffer)) {
                    memcpy(localBuffer + localBufferUsed, pcm16Buf, bytesToAdd);
                    localBufferUsed += bytesToAdd;
                } else {
                    // Lokální buffer je plný - poslat jej do ringbufferu
                    if (xSemaphoreTake(audioBufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        if (xRingbufferSend(audioRingBuffer, localBuffer, localBufferUsed, pdMS_TO_TICKS(2)) != pdTRUE) {
                            audioDiag.ringbufferOverflowCount++;
                            audioDiag.lastOverflowTime = millis();
                            audioDiag.audioDataDroppedBytes += localBufferUsed;
                            
                            static unsigned long lastOverflowReport = 0;
                            if (millis() - lastOverflowReport > 10000) {
                                Serial.printf("WARN:RINGBUFFER_OVERFLOW,COUNT=%lu,DROPPED=%lu\n", 
                                              audioDiag.ringbufferOverflowCount, 
                                              audioDiag.audioDataDroppedBytes);
                                lastOverflowReport = millis();
                            }
                        }
                        xSemaphoreGive(audioBufferMutex);
                    }
                    
                    // Vynulovat lokální buffer a zkopírovat nová data
                    localBufferUsed = 0;
                    if (bytesToAdd <= sizeof(localBuffer)) {
                        memcpy(localBuffer, pcm16Buf, bytesToAdd);
                        localBufferUsed = bytesToAdd;
                    }
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        } else {
            // V RX módu - vyprázdnit lokální buffer
            localBufferUsed = 0;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ==========================================
// KONTROLA TIMEOUTU HANDSHAKE
// ==========================================
void checkHandshakeTimeout() {
    if (pendingCmd != CMD_NONE) {
        if (millis() - pendingCmdTime > CONFIRM_TIMEOUT_MS) {
            if (pendingCmd == PENDING_REBOOT) {
                Serial.println("ERR:REBOOT_TIMEOUT");
            } else if (pendingCmd == PENDING_RESET) {
                Serial.println("ERR:RESET_TIMEOUT");
            }
            pendingCmd = CMD_NONE;
        }
    }
}

// ==========================================
// PARSOVÁNÍ A ZPRACOVÁNÍ UART PŘÍKAZŮ (CORE 0)
// ==========================================
void processUartCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "GET:NAME") {
        Serial.println("NAME:" + btName);
    }
    else if (cmd.startsWith("SET:NAME=")) {
        String newName = cmd.substring(9);
        if (newName.length() >= 1 && newName.length() <= 30) {
            prefs.begin("bt_config", false);
            prefs.putString("bt_name", newName);
            prefs.end();
            btName = newName;
            Serial.println("OK:NAME_SAVED_REBOOT_REQUIRED");
        } else {
            Serial.println("ERR:INVALID_NAME");
        }
    }
    else if (cmd == "GET:VOL") {
        Serial.println("VOL:" + String(volume));
    }
    else if (cmd.startsWith("SET:VOL=")) {
        int val = cmd.substring(8).toInt();
        if (val >= 0 && val <= 127) {
            volume = (uint8_t)val;
            prefs.begin("bt_config", false);
            prefs.putUChar("volume", volume);
            prefs.end();

            // Pokud jsme v RX módu a připojeni, odešleme hlasitost do vysílajícího zařízení
            if (btMode == "RX" && isConnected) {
                esp_avrc_rn_param_t rn_param;
                rn_param.volume = volume;
                esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
            }

            Serial.println("OK:VOL=" + String(volume));
        } else {
            Serial.println("ERR:INVALID_VOL");
        }
    }
    else if (cmd.startsWith("SET:GAIN=")) {
        float val = cmd.substring(9).toFloat();
        if (val >= 0.0f && val <= 2.0f) {
            gain = val;
            prefs.begin("bt_config", false);
            prefs.putFloat("gain", gain);
            prefs.end();
            Serial.println("OK:GAIN=" + String(gain, 2));
        } else {
            Serial.println("ERR:INVALID_GAIN");
        }
    }
    else if (cmd.startsWith("SET:MUTE=")) {
        int val = cmd.substring(9).toInt();
        if (val == 0 || val == 1) {
            isMuted = (val == 1);
            Serial.println("OK:MUTE=" + String(isMuted ? 1 : 0));
        } else {
            Serial.println("ERR:INVALID_MUTE");
        }
    }
    else if (cmd == "GET:STATUS") {
        Serial.println("STATUS:" + String(isConnected ? "CONNECTED" : "DISCONNECTED") + ",MODE:" + btMode);
    }
    else if (cmd == "CMD:DISCONNECT") {
        if (isConnected) {
            clearAudioBuffer();
            if (btMode == "TX") {
                esp_a2d_source_disconnect(connected_bda);
            } else {
                esp_a2d_sink_disconnect(connected_bda);
            }
            isConnected = false;
            Serial.println("OK:DISCONNECTED");
            Serial.println("STATUS:DISCONNECTED");
        } else {
            Serial.println("ERR:NOT_CONNECTED");
        }
    }
    else if (cmd == "GET:REMOTENAME" || cmd == "GETREMOTENAME") {
        if (isConnected) {
            if (currentDeviceName.length() > 0) {
                Serial.println("REMOTENAME:" + currentDeviceName);
            } else {
                esp_bt_gap_read_remote_name(connected_bda);
                Serial.println("REMOTENAME:UNKNOWN");
            }
        } else {
            Serial.println("ERR:NOT_CONNECTED");
        }
    }
    else if (cmd == "CMD:PLAY") {
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_RELEASED);
        Serial.println("OK:PLAY");
    }
    else if (cmd == "CMD:PAUSE") {
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_PRESSED);
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_RELEASED);
        Serial.println("OK:PAUSE");
    }
    else if (cmd == "CMD:NEXT") {
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_STATE_RELEASED);
        Serial.println("OK:NEXT");
    }
    else if (cmd == "CMD:PREV") {
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_BACKWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
        esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_BACKWARD, ESP_AVRC_PT_CMD_STATE_RELEASED);
        Serial.println("OK:PREV");
    }
    else if (cmd == "GET:MODE") {
        Serial.println("MODE:" + btMode);
    }
    else if (cmd.startsWith("SET:MODE=")) {
        String newMode = cmd.substring(9);
        newMode.trim();
        // OPRAVA #5: Správné převedení na velká písmena pomocí změny jednotlivých znaků
        for (int i = 0; i < newMode.length(); i++) {
            newMode[i] = toupper(newMode[i]);
        }
        if (newMode == "TX" || newMode == "RX") {
            if (newMode == btMode) {
                Serial.println("OK:MODE_ALREADY_SET");
            } else {
                prefs.begin("bt_config", false);
                prefs.putString("mode", newMode);
                prefs.end();
                Serial.println("OK:MODE_CHANGED_REBOOTING_TO_" + newMode);
                i2s_stop_and_deinit();
                Serial.flush();
                delay(100);
                ESP.restart();
            }
        } else {
            Serial.println("ERR:INVALID_MODE");
        }
    }
    else if (cmd == "GET:TRACK" || cmd == "GET:METADATA") {
        if (btMode == "RX") {
            Serial.printf("TRACK:%s|%s|%s\n", trackTitle.c_str(), trackArtist.c_str(), trackAlbum.c_str());
        } else {
            Serial.println("ERR:NOT_IN_RX_MODE");
        }
    }
    else if (cmd == "GET:INFO") {
        Serial.printf("INFO:RAM_FREE:%u,MODE:%s\n", ESP.getFreeHeap(), btMode.c_str());
    }
    else if (cmd == "GET:FWINFO") {
        Serial.printf("INFO:%s v%s,DATE:%s,CODE:%ld\n", FW_VERSION_NAME, FW_VERSION_STR, FW_BUILD_DATETIME, FW_VERSION_CODE);
    }
    else if (cmd == "GET:FWCODE") {
        Serial.printf("INFO:%ld\n", FW_VERSION_CODE);
    }
    else if (cmd == "GET:DIAG") {
        printAudioDiagnostics(); 
    }
    else if (cmd == "CMD:RESET-DIAG") {
        resetAudioDiagnostics();
    }
    else if (cmd == "GET:RSSI") {
        if (isConnected) {
            esp_bt_gap_read_rssi_delta(connected_bda);
        } else {
            Serial.println("ERR:NOT_CONNECTED");
        }
    }
    else if (cmd == "GET:TEMP") {
        float temp = getChipTemperature();
        if (temp > -100.0f) {
            Serial.printf("TEMP:%.1f\n", temp);
        } else {
            Serial.println("ERR:TEMP_READ_FAILED");
        }
    }
    else if (cmd == "CMD:REBOOT") {
        pendingCmd = PENDING_REBOOT;
        pendingCmdTime = millis();
        Serial.println("REQ:CONFIRM-REBOOT");
    }
    else if (cmd == "CMD:GO-REBOOT") {
        if (pendingCmd == PENDING_REBOOT) {
            pendingCmd = CMD_NONE;
            Serial.println("OK:REBOOTING");
            Serial.flush();
            delay(100);
            ESP.restart();
        } else {
            Serial.println("ERR:REBOOT_TIMEOUT");
        }
    }
    else if (cmd == "CMD:RESET-DEFAULT") {
        pendingCmd = PENDING_RESET;
        pendingCmdTime = millis();
        Serial.println("REQ:CONFIRM-RESET");
    }
    else if (cmd == "CMD:GO-RESET-DEFAULT") {
        if (pendingCmd == PENDING_RESET) {
            pendingCmd = CMD_NONE;
            resetToDefaults();
            Serial.println("OK:FACTORY_RESET_DONE");
            Serial.flush();
            delay(100);
            ESP.restart();
        } else {
            Serial.println("ERR:RESET_TIMEOUT");
        }
    }
    else {
        Serial.println("ERR:UNKNOWN_COMMAND");
    }
}

// ==========================================
// ARDUINO MAIN SETUP & LOOP (CORE 0)
// ==========================================
void setup() {
    delay(500);
    Serial.begin(UART_BAUD_RATE);
    delay(500);

    Serial.printf("\n\nFirmware: %s v%s\n", FW_VERSION_NAME, FW_VERSION_STR);
    Serial.printf("Build:    %s\n", FW_BUILD_DATETIME);
    Serial.printf("Code:     %ld\n\n", FW_VERSION_CODE);

    // Nastavení krátkého neblokujícího timeoutu pro čtení z UART
    Serial.setTimeout(50); 
    delay(500);

    initTempSensor();
    loadNVSConfig();

    xTaskCreatePinnedToCore(
        audioProcessingTask,
        "AudioTask",
        8192,
        NULL,
        15,
        &audioTaskHandle,
        1
    );

    // OPRAVA #1: Vyjasnění logiky - TX znamená odesílání do BT (RX data z ADC)
    if (btMode == "RX") {
        i2s_init_tx(44100);      // RX mód: data z BT do DAC (TX kanál I2S)
    } else {
        i2s_init_rx(44100);      // TX mód: data z ADC do BT (RX kanál I2S)
    }

    initBluetoothStack();

    Serial.println("INFO:READY,NAME=" + btName + ",MODE=" + btMode);
}

void loop() {
    static String inputBuffer = "";

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n') {
            processUartCommand(inputBuffer);
            inputBuffer = "";
        } else if (c != '\r') { // Ignorujeme CR (Carriage Return)
            inputBuffer += c;
            // Ochrana proti přetečení nevalidním řetězcem
            if (inputBuffer.length() > 128) {
                inputBuffer = "";
                Serial.println("ERR:INPUT_BUFFER_OVERFLOW");
            }
        }
    }

    checkHandshakeTimeout();
    vTaskDelay(pdMS_TO_TICKS(5)); // Krátký odpočinek pro IDLE task na Core 0
}
