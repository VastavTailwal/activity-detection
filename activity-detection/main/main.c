#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "gy91.h"
#include "model_predictor.h"


static const char *TAG = "example";

// Feature buffer
float input_data[16];

void app_main(void)
{
    gy91_init();
    gy91_init_magnetometer();

    while (true)
    {
        float ax, ay, az, gx, gy, gz, pressure, mx, my, mz, roll, pitch, yaw, gravity_x, gravity_y, gravity_z;
        
        gy91_read_accel(&ax, &ay, &az);
        gy91_read_gyro(&gx, &gy, &gz);
        gy91_read_magnetometer(&mx, &my, &mz);

        pressure = gy91_read_pressure();
        
        roll = atan2(ay, az) * 180 / M_PI;
        pitch = atan(-ax / sqrt(ay * ay + az * az)) * 180 / M_PI;
        yaw = atan2(my, mx) * 180 / M_PI;

        float pitch_rad = pitch * M_PI / 180.0;
        float roll_rad = roll * M_PI / 180.0;
        
        gravity_x = -g * sin(pitch_rad);
        gravity_y = g * sin(roll_rad) * cos(pitch_rad);
        gravity_z = g * cos(roll_rad) * cos(pitch_rad);

        ESP_LOGI("GY91", "Accel: %.2f, %.2f, %.2f g", ax, ay, az);
        ESP_LOGI("GY91", "Gyro: %.2f, %.2f, %.2f dps", gx, gy, gz);
        ESP_LOGI("GY91", "Magnet: %.2f, %.2f, %.2f uT", mx, my, mz);
        ESP_LOGI("GY91", "Pressure: %.2f Pa", pressure);
        ESP_LOGI("GY91", "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
        ESP_LOGI("GY91", "Gravity: %.2f, %.2f, %.2f", gravity_x, gravity_y, gravity_z);

        input->data.f[0] = roll;
        input->data.f[1] = pitch;
        input->data.f[2] = yaw;
        input->data.f[3] = gravity_x;
        input->data.f[4] = gravity_y;
        input->data.f[5] = gravity_z;
        input->data.f[6] = gx;
        input->data.f[7] = gy;
        input->data.f[8] = gz;
        input->data.f[9] = ax;
        input->data.f[10] = ay;
        input->data.f[11] = az;
        input->data.f[12] = mx;
        input->data.f[13] = my;
        input->data.f[14] = mz;
        input->data.f[15] = pressure;

        if (interpreter->Invoke() != kTfLiteOk)
        {
            printf("Invoke failed!\n");
            continue;
        }

        switch (predict(input_data))
        {
            case 0:
                ESP_LOGI(TAG, "Walking)");
                break;
            case 1:
                ESP_LOGI(TAG, "Jogging");
                break;
            case 2:
                ESP_LOGI(TAG, "Downstairs");
                break;
            case 3:
                ESP_LOGI(TAG, "Upstairs");
                break;
            case 4:
                ESP_LOGI(TAG, "Standing");
                break;
            case 5:
                ESP_LOGI(TAG, "Sitting");
                break;
            case 6:
                ESP_LOGI(TAG, "Sleeping");
                break;
            default:
                ESP_LOGI(TAG, "Unknown");
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
