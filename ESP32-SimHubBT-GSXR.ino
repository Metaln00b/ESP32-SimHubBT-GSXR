#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "soc/ledc_reg.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled! Please run `make menuconfig` to and enable it"
#endif

#define BT_NAME                 "BT-DASH-GSXR"

#define LEDC_TIMER_RES          LEDC_TIMER_10_BIT
#define DUTY_MAX                ((1 << LEDC_TIMER_10_BIT) -1 )
#define FREQ_MIN_Hz             1 /* Do not decrease it! */
#define SPEED_THRESHOLD_Hz      5

#define NEUTRAL_PIN             33
#define H_BEAM_PIN              2
#define OIL_PIN                 27
#define TURN_PIN                4
#define RPM_PIN                 26
#define SPEED_PIN               25
#define FUEL_A_PIN              16
#define FUEL_B_PIN              17

#define BUF_SIZE                128

char simHubMessageBuf[BUF_SIZE];
BluetoothSerial btSerial;

float revs;
float speed_kmh;
float fuel_percent;
float water_temperature_degC;
int turn_left;
int turn_right;
int brake;
float oil_temperature_degC;

unsigned int temp_duty_cycle = 0;
float speed_Hz = 0;
float rpm_Hz = 0;
unsigned int fuel_duty_cycle = 0;

void ledc_init(uint8_t pin, float freq_Hz, ledc_channel_t channel, ledc_timer_t timer) {
    const char * ME = __func__;

    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);

    uint32_t precision = DUTY_MAX + 1;
    uint32_t div_param = ((uint64_t) LEDC_REF_CLK_HZ << 8) / freq_Hz / precision;
    if (div_param < 256 || div_param > LEDC_DIV_NUM_HSTIMER0_V)
    {
        ESP_LOGE(ME, "requested frequency and duty resolution can not be achieved, try increasing freq_hz or duty_resolution. div_param=%d", (uint32_t ) div_param);
    }

    ledc_channel_config_t ledc_channel = {
      .gpio_num   = pin,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel    = channel,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = timer,
      .duty       = DUTY_MAX,
      .hpoint     = 0         // TODO: AD 10.11.2018: new, does 0 work (0xfffff does not work)
    };
    err = ledc_channel_config(&ledc_channel);
    ESP_LOGD(ME,"ledc_channel_config returned %d",err);
    
    err = ledc_timer_set(LEDC_HIGH_SPEED_MODE, timer, div_param, LEDC_TIMER_RES, LEDC_REF_TICK);
    if (err)
    {
        ESP_LOGE(ME, "ledc_timer_set returned %d",err);
    }
    
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, timer);
    ESP_LOGD(ME, "ledc_timer_set: divider: 0x%05x duty_resolution: %d\n", (uint32_t) div_param, LEDC_TIMER_RES);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    pinMode(NEUTRAL_PIN, OUTPUT);
    pinMode(H_BEAM_PIN, OUTPUT);
    pinMode(OIL_PIN, OUTPUT);
    pinMode(TURN_PIN, OUTPUT);
    pinMode(FUEL_A_PIN, OUTPUT);
    pinMode(FUEL_B_PIN, OUTPUT);

    digitalWrite(NEUTRAL_PIN, LOW);
    digitalWrite(H_BEAM_PIN, LOW);
    digitalWrite(OIL_PIN, LOW);
    digitalWrite(TURN_PIN, LOW);
    digitalWrite(FUEL_A_PIN, HIGH);
    digitalWrite(FUEL_B_PIN, HIGH);

    ledc_init(RPM_PIN, FREQ_MIN_Hz, LEDC_CHANNEL_0, LEDC_TIMER_0);
    ledc_init(SPEED_PIN, FREQ_MIN_Hz, LEDC_CHANNEL_1, LEDC_TIMER_1);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 512);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    
    delay(1000);
    uint32_t dut_0 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    uint32_t dut_1 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

    memset(simHubMessageBuf, 0x0, BUF_SIZE);
    btSerial.begin(BT_NAME);
}

void loop() {
    if (btSerial.available() > 0)
    {
        btSerial.readBytesUntil('{', simHubMessageBuf, BUF_SIZE);
        int readCount = btSerial.readBytesUntil('}', simHubMessageBuf, BUF_SIZE);
        simHubMessageBuf[min(readCount, BUF_SIZE - 1)] = 0x0;
        processMessage();
        memset(simHubMessageBuf, 0x0, BUF_SIZE);
    }
}

void processMessage() {
    sscanf(simHubMessageBuf, "%f&%f&%f&%f&%d&%d&%d&%f",
        &revs,
        &speed_kmh,
        &fuel_percent,
        &water_temperature_degC,
        &turn_left,
        &turn_right,
        &brake,
        &oil_temperature_degC
    );

    rpm_Hz = 0.016 * revs - 1.143;
    speed_Hz = 1.48 * speed_kmh + 3;

    if (fuel_percent > 0.0 && fuel_percent <= 5.0)
    {
        digitalWrite(FUEL_A_PIN, LOW);
        digitalWrite(FUEL_B_PIN, HIGH);
    }
    else if (fuel_percent > 5.0 && fuel_percent <= 10.0)
    {
        digitalWrite(FUEL_A_PIN, LOW);
        digitalWrite(FUEL_B_PIN, LOW);
    }
    else if (fuel_percent > 10.0 && fuel_percent <= 100.0)
    {
        digitalWrite(FUEL_A_PIN, HIGH);
        digitalWrite(FUEL_B_PIN, HIGH);
    }

    if (turn_right != 0 || turn_left != 0)
    {
        digitalWrite(TURN_PIN, HIGH);
    }
    else
    {
        digitalWrite(TURN_PIN, LOW);
    }

    if (rpm_Hz < FREQ_MIN_Hz)
    {
        rpm_Hz = FREQ_MIN_Hz;
    }
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, rpm_Hz);

    if (speed_Hz < SPEED_THRESHOLD_Hz)
    {
        speed_Hz = FREQ_MIN_Hz;
    }
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, speed_Hz);
}
