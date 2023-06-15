#include <Wire.h>
#include <Adafruit_BME680.h>
#include <SparkFunLSM9DS1.h>

Adafruit_BME680 bme680;
LSM9DS1 imu;

volatile bool timerFlag = false;  // Indicateur d'interruption du Timer1

void setup() {
  Serial.begin(9600);

  // Initialise la communication I2C
  Wire.begin();

  // Initialise le capteur BME680
  if (!bme680.begin(0x77)) {
    Serial.println("Impossible de trouver le capteur BME680. Vérifiez les connexions ou l'adresse I2C.");
    while (1);
  }

  // Initialise le capteur LSM9DS1
  if (!imu.begin()) {
    Serial.println("Impossible de trouver le capteur LSM9DS1. Vérifiez les connexions ou l'adresse I2C.");
    while (1);
  }

  // Configure le Timer1 pour générer une interruption toutes les 1 seconde
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15625;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void loop() {
  if (timerFlag) {
    timerFlag = false;  // Réinitialise l'indicateur d'interruption

    // Effectue les actions requises à chaque interruption du Timer1
    // Par exemple, vous pouvez effectuer une lecture périodique des capteurs ici
    float temperature = bme680.temperature;
    float humidity = bme680.humidity;
    float pressure = bme680.pressure / 100.0;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;

    imu.readAccel();
    imu.readGyro();
    imu.readMag();

    accelX = imu.calcAccel(imu.ax);
    accelY = imu.calcAccel(imu.ay);
    accelZ = imu.calcAccel(imu.az);

    gyroX = imu.calcGyro(imu.gx);
    gyroY = imu.calcGyro(imu.gy);
    gyroZ = imu.calcGyro(imu.gz);

    magX = imu.calcMag(imu.mx);
    magY = imu.calcMag(imu.my);
    magZ = imu.calcMag(imu.mz);

    // Affiche les valeurs lues dans le moniteur série
    Serial.print("Température: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidité: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Pression: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Accélération (X,Y,Z): ");
    Serial.print(accelX);
    Serial.print(", ");
    Serial.print(accelY);
    Serial.print(", ");
    Serial.println(accelZ);

    Serial.print("Gyroscope (X,Y,Z): ");
    Serial.print(gyroX);
    Serial.print(", ");
    Serial.print(gyroY);
    Serial.print(", ");
    Serial.println(gyroZ);

    Serial.print("Magnétomètre (X,Y,Z): ");
    Serial.print(magX);
    Serial.print(", ");
    Serial.print(magY);
    Serial.print(", ");
    Serial.println(magZ);
  }

  // Effectue d'autres tâches dans la boucle principale
}

// Gestionnaire d'interruption pour le Timer1
ISR(TIMER1_COMPA_vect) {
  timerFlag = true;  // Définit l'indicateur d'interruption à true
}