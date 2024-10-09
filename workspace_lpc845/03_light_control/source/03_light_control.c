#include "board.h"
#include "fsl_swm.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"

// Direccion del BH1750
#define BH1750_ADDR 0x5c

// Definir pin del LED
#define LED_D1 29

// Función map para ajustar el brillo del LED
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Función para generar PWM por software
void pwmSoftwareControl(int brightness) {
    int onTime = brightness;  // Tiempo en que el LED está encendido
    int offTime = 255 - brightness;  // Tiempo en que el LED está apagado

    if (brightness > 0) {
    	GPIO_PinWrite(GPIO, 0, LED_D1, !GPIO_PinRead(GPIO, 1, LED_D1));  // Enciende el LED
        SDK_DelayAtLeastUs(onTime * 10, SystemCoreClock);  // Retardo para mantener el LED encendido
    }

    GPIO_PinWrite(GPIO, 0, LED_D1, !GPIO_PinRead(GPIO, 0, LED_D1));  // Apaga el LED
    SDK_DelayAtLeastUs(offTime * 10, SystemCoreClock);  // Retardo para mantener el LED apagado
}

int main(void) {
    // Arranco de 30 MHz
    BOARD_BootClockFRO30M();

    // Estructura de configuracion para salida
       gpio_pin_config_t Config = { kGPIO_DigitalOutput, 1 };
       // Habilito el clock del GPIO 1
       GPIO_PortInit(GPIO, 0);
       // Configuro el pin 0 del GPIO 1 como salida
       GPIO_PinInit(GPIO, 0, LED_D1, &Config);

       // Inicializo el clock del I2C1
       	CLOCK_Select(kI2C1_Clk_From_MainClk);
           // Asigno las funciones de I2C1 a los pines 26 y 27
       	CLOCK_EnableClock(kCLOCK_Swm);
           SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SDA, kSWM_PortPin_P0_27);
           SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SCL, kSWM_PortPin_P0_26);
           CLOCK_DisableClock(kCLOCK_Swm);

    // Configuracion de master para el I2C con 400 KHz de clock
       i2c_master_config_t config;
       I2C_MasterGetDefaultConfig(&config);
       config.baudRate_Bps = 400000;

       if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
       		// Comando de power on
       		uint8_t cmd = 0x01;
       		I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
       		I2C_MasterStop(I2C1);
       	}
       	if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
       		// Comando de medicion continua a 1 lux de resolucion
       		uint8_t cmd = 0x10;
       		I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
       		I2C_MasterStop(I2C1);
       	}

    while (1) {
        // Lectura del sensor
        if (I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Read) == kStatus_Success) {
            uint8_t res[2] = {0};  // Array para almacenar los datos leídos
            I2C_MasterReadBlocking(I2C1, res, 2, kI2C_TransferDefaultFlag);
            I2C_MasterStop(I2C1);

            // Calculo el valor de lux a partir de los datos leídos
            float lux = ((res[0] << 8) + res[1]) / 1.2;
            PRINTF("LUX : %d \r\n", (uint16_t)lux);

            // Mapeo el valor de lux a un rango de brillo del LED (0-255)
            int brightness = map(lux, 0, 1023, 0, 255);

            // Control del LED con PWM por software
            pwmSoftwareControl(brightness);
        }
    }

    return 0;
}
